#include "ColumnGeneration.h"
// 添加初始列
void AddInitialColumns(IloEnv env, cmnd_t* instance, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZConstraints, IloRangeArray2 EqualXijkConstraints,
	IloNumVarArray2 x, IloNumVarArray2 y)
{
	// Initial columns to set up algorithm
	// 由于每条arc上的capacity没有限制，所以只需要生成一条path便能产生可行解
	for (int i = 0; i < instance->n_commods; i++)
	{
		IloInt origin = instance->commod_orig_node[i], dest = instance->commod_dest_node[i];
		IloIntArray path(env);
		InitialShortestPath(env, path, i, origin, dest, instance);
		IloNum pathcost = 0;
		IloIntArray pij(env);  // capacity constraints x coeffiences
		pij.add(instance->n_arcs, 0);
		for (int j = 0; j < path.getSize(); j++)
		{
			pathcost += instance->arc_commod_ucost[path[j]][i];
			pij[path[j]] = 1;
		}
		x[i].add(IloNumVar(totalCost(pathcost) + capacityConstraints(pij) + demandConstraints[i](1) + EqualXijkConstraints[i](pij)));

		for (int j = 0; j < n_VehicleTypes; j++)
		{
			IloIntArray cycle(path);
			IloIntArray temp(env);
			InitialShortestPath(env, temp, i, dest, origin, instance);
			cycle.add(temp);
			IloNumArray qij(env);
			IloNumArray zij(env);
			qij.add(instance->n_arcs, 0);
			zij.add(instance->n_arcs, 0);
			for (int k = 0; k < cycle.getSize(); k++)
			{
				qij[cycle[k]] = -vehicleCapacity[j];
				zij[cycle[k]] = 1;
			}

			y[j].add(IloNumVar(totalCost(vehicleFixedcost[j] * cycle.getSize()) + capacityConstraints(qij)
				+ EqualZConstraints[j](zij)));

			temp.end();
			// 加上这一句会出错，说明cycle只是引用了temp和path的值，并没有分配内存；
			//cycle.end();
			qij.end();
			zij.end();
		}
		path.end();
		pij.end();
	}
}

IloNum ColumnGeneration(IloEnv env, IloCplex masterSolver, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZConstraints, IloRangeArray2 EqualXijkConstraints,
	IloNumVarArray2 y, IloNumVarArray2 x, cmnd_t* instance)
{
	// Column generation
	IloNum preColumnNum = 0, ColumnNum = 0;
	IloNum columnObj = -1;
	// Dual values arrays
	IloNumArray capacityDual(env, instance->n_arcs), demandDual(env, instance->n_commods);
	for (;;){

		preColumnNum = ColumnNum;
		masterSolver.solve();
		// 如果目标值不便则停止循环
		//if (IloFloor(columnObj) == IloFloor(masterSolver.getObjValue()))
		if (columnObj == masterSolver.getObjValue())
			break;
		else
			columnObj = masterSolver.getObjValue();

		// Capacity duals
		for (int i = 0; i < instance->n_arcs; i++)
			capacityDual[i] = masterSolver.getDual(capacityConstraints[i]);

		for (int i = 0; i < instance->n_commods; i++)
			demandDual[i] = masterSolver.getDual(demandConstraints[i]);
		// path generate subproblems
		for (int i = 0; i < instance->n_commods; i++)
		{
			IloIntArray path(env);
			PathSubproblem(env, path, i, instance, capacityDual);
			// reduced cost
			IloNum pathReducecost = 0, pathcost = 0;
			IloIntArray pij(env);  // capacity constraints x coeffiences
			pij.add(instance->n_arcs, 0);
			for (int j = 0; j < path.getSize(); j++)
			{
				pathReducecost += instance->arc_commod_ucost[path[j]][i] - capacityDual[path[j]];
				pathcost += instance->arc_commod_ucost[path[j]][i];
				pij[path[j]] = 1;
			}
			// when negetive
			//cout << pathReducecost - demandDual[i] << endl;
			if (pathReducecost - demandDual[i] < -RC_EPS)
			{

				x[i].add(IloNumVar(totalCost(pathcost) + capacityConstraints(pij) + demandConstraints[i](1)
					+ EqualXijkConstraints[i](pij)));
				ColumnNum++;
			}

			path.end();
			pij.end();
		}


		// cycle generate subproblems
		for (int i = 0; i < instance->n_nodes; i++)
		{
			for (int j = 0; j < n_VehicleTypes; j++)
			{
				IloIntArray cycle(env);
				CycleBellmanFord(env, cycle, i, instance, capacityDual, j);
				IloNum cycleReducecost = 0;
				IloNumArray qij(env);
				qij.add(instance->n_arcs, 0);
				IloNumArray zij(env);
				zij.add(instance->n_arcs, 0);
				for (int k = 0; k < cycle.getSize(); k++)
				{
					cycleReducecost += vehicleFixedcost[j] + vehicleCapacity[j] * capacityDual[cycle[k]];
					qij[cycle[k]] = -vehicleCapacity[j];
					zij[cycle[k]] = 1;
				}

				/*cout << cycleReducecost << endl;*/
				if (cycleReducecost < -RC_EPS)
				{
					y[j].add(IloNumVar(totalCost(vehicleFixedcost[j] * cycle.getSize()) + capacityConstraints(qij)
						+ EqualZConstraints[j](zij)));
					ColumnNum++;
				}

				cycle.end();
				qij.end();
				zij.end();
			}
		}

		// When there is no new column to enter the master problem 
		// that isthere is no column whose reduced cost is positive
		// "jump out" the cycle;
		if (preColumnNum == ColumnNum)
			break;
	}


	//masterSolver.setOut(env.getNullStream());
	masterSolver.solve();
	cout << "Linear Master Model Objective Value Solved by CG(Lower Bound): " << masterSolver.getObjValue() << endl;

	// 移除为0的y列
	//cout << "移除前" << endl;
	/*for (int i = 0; i < y.getSize(); i++)
	{
	for (int j = 0; j <y[i].getSize(); j++)
	{
	cout << " " << masterSolver.getValue(y[i][j]);
	}
	cout << endl;
	}*/

	/*for (int i = 0; i < y.getSize(); i++)
	{
	int num = y[i].getSize();
	for (int j = num - 1; j > -1; j--)
	{
	if (masterSolver.getValue(y[i][j]) == 0)
	y[i].remove(j);
	}
	}*/

	//cout << "移除后" << endl;
	//for (int i = 0; i < y.getSize(); i++)
	//{
	//	for (int j = 0; j <y[i].getSize(); j++)
	//	{
	//		cout << " " << masterSolver.getValue(y[i][j]);
	//	}
	//	cout << endl;
	//}

	//// 移除为0的x列
	//for (int i = 0; i < x.getSize(); i++)
	//{
	//	int num = x[i].getSize();
	//	for (int j = num - 1; j > -1; j--)
	//	{
	//		if (masterSolver.getValue(x[i][j]) == 0)
	//			x[i].remove(j);
	//	}
	//}

	capacityDual.end();
	demandDual.end();
	return masterSolver.getObjValue();
}

// 不保留原列，并会修改对偶值进行列生成
IloNum InequalityCG(IloEnv env, IloCplex masterSolver, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZConstraints, IloRangeArray2 EqualXijkConstraints,
	IloRangeArray inequalityCons, IloIntArray2 kijIndex, IloNumVarArray2 y, IloNumVarArray2 x, cmnd_t* instance)
{
	for (int i = 0; i < y.getSize(); i++)
	{
		for (int j = 0; j < y[i].getSize(); j++)
		{
			//y[i].remove(j);
			y[i][j].removeFromAll();
		}
	}
	for (int i = 0; i < x.getSize(); i++)
	{
		for (int j = 0; j < x[i].getSize(); j++)
		{
			//x[i].remove(j);
			x[i][j].removeFromAll();
		}
	}
	AddInitialColumns(env, instance, totalCost, capacityConstraints, demandConstraints, EqualZConstraints, EqualXijkConstraints, x, y);	// Column generation
	IloNum preColumnNum = 0, ColumnNum = 0;
	IloNum columnObj = -1;
	// Dual values arrays
	IloNumArray capacityDual(env, instance->n_arcs), demandDual(env, instance->n_commods);
	// Inequality dual values
	IloNumArray inequalityDual(env, inequalityCons.getSize());
	for (;;){

		preColumnNum = ColumnNum;
		masterSolver.solve();
		// 如果目标值不便则停止循环
		//if (IloFloor(columnObj) == IloFloor(masterSolver.getObjValue()))
		if (columnObj == masterSolver.getObjValue())
			break;
		else
			columnObj = masterSolver.getObjValue();

		// Capacity duals
		for (int i = 0; i < instance->n_arcs; i++)
			capacityDual[i] = masterSolver.getDual(capacityConstraints[i]);

		for (int i = 0; i < instance->n_commods; i++)
			demandDual[i] = masterSolver.getDual(demandConstraints[i]);

		// Inequality duals
		for (int i = 0; i < inequalityCons.getSize(); i++)
			inequalityDual[i] = masterSolver.getDual(inequalityCons[i]);
		// 多加一个0的元素
		inequalityDual.add(0);
		// path generate subproblems
		for (int i = 0; i < instance->n_commods; i++)
		{
			IloIntArray path(env);
			PathSubproblem(env, path, i, instance, capacityDual);
			// reduced cost
			IloNum pathReducecost = 0, pathcost = 0;
			IloIntArray pij(env);  // capacity constraints x coeffiences
			pij.add(instance->n_arcs, 0);
			for (int j = 0; j < path.getSize(); j++)
			{
				pathReducecost += instance->arc_commod_ucost[path[j]][i] - capacityDual[path[j]] - inequalityDual[kijIndex[i][path[j]]];
				pathcost += instance->arc_commod_ucost[path[j]][i];
				pij[path[j]] = 1;
			}
			// when negetive
			//cout << pathReducecost - demandDual[i] << endl;
			if (pathReducecost - demandDual[i] < -RC_EPS)
			{

				x[i].add(IloNumVar(totalCost(pathcost) + capacityConstraints(pij) + demandConstraints[i](1)
					+ EqualXijkConstraints[i](pij)));
				ColumnNum++;
			}

			path.end();
			pij.end();
		}


		// cycle generate subproblems
		for (int i = 0; i < instance->n_nodes; i++)
		{
			for (int j = 0; j < n_VehicleTypes; j++)
			{
				IloIntArray cycle(env);
				CycleBellmanFord(env, cycle, i, instance, capacityDual, j);
				IloNum cycleReducecost = 0;
				IloNumArray qij(env);
				qij.add(instance->n_arcs, 0);
				IloNumArray zij(env);
				zij.add(instance->n_arcs, 0);
				for (int k = 0; k < cycle.getSize(); k++)
				{
					IloNum tempDual = 0;
					for (int a = 0; a < instance->n_commods; a++)
						tempDual += instance->commod_supply[a] * inequalityDual[kijIndex[a][cycle[k]]];
					cycleReducecost += vehicleFixedcost[j] + vehicleCapacity[j] * capacityDual[cycle[k]] + tempDual;
					qij[cycle[k]] = -vehicleCapacity[j];
					zij[cycle[k]] = 1;
				}

				/*cout << cycleReducecost << endl;*/
				if (cycleReducecost < -RC_EPS)
				{
					y[j].add(IloNumVar(totalCost(vehicleFixedcost[j] * cycle.getSize()) + capacityConstraints(qij)
						+ EqualZConstraints[j](zij)));
					ColumnNum++;
				}

				cycle.end();
				qij.end();
				zij.end();
			}
		}

		// When there is no new column to enter the master problem 
		// that isthere is no column whose reduced cost is positive
		// "jump out" the cycle;
		if (preColumnNum == ColumnNum)
			break;
	}


	//masterSolver.setOut(env.getNullStream());
	masterSolver.solve();
	cout << "Linear Master Model Objective Value Solved by Inequality CG(Lower Bound): " << masterSolver.getObjValue() << endl;

	// 移除为0的y列
	//cout << "移除前" << endl;
	/*for (int i = 0; i < y.getSize(); i++)
	{
	for (int j = 0; j <y[i].getSize(); j++)
	{
	cout << " " << masterSolver.getValue(y[i][j]);
	}
	cout << endl;
	}*/

	/*for (int i = 0; i < y.getSize(); i++)
	{
	int num = y[i].getSize();
	for (int j = num - 1; j > -1; j--)
	{
	if (masterSolver.getValue(y[i][j]) == 0)
	y[i].remove(j);
	}
	}*/

	//cout << "移除后" << endl;
	//for (int i = 0; i < y.getSize(); i++)
	//{
	//	for (int j = 0; j <y[i].getSize(); j++)
	//	{
	//		cout << " " << masterSolver.getValue(y[i][j]);
	//	}
	//	cout << endl;
	//}

	//// 移除为0的x列
	//for (int i = 0; i < x.getSize(); i++)
	//{
	//	int num = x[i].getSize();
	//	for (int j = num - 1; j > -1; j--)
	//	{
	//		if (masterSolver.getValue(x[i][j]) == 0)
	//			x[i].remove(j);
	//	}
	//}

	capacityDual.end();
	demandDual.end();
	inequalityDual.end();
	return masterSolver.getObjValue();
}