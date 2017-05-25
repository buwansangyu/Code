#include "UpperBound.h"
void CopyArray(IloIntArray oldArray, IloIntArray newArray)
{
	newArray.clear();
	for (int i = 0; i < oldArray.getSize(); i++)
		newArray.add(oldArray[i]);
}
IloNum LocalSearch(IloEnv env, cmnd_t* instance, IloNumArray2 xijk, IloNumArray2 z, IloIntArray tabuList)
{
	// ��flow����0�Ļ���Ϊ��ѡ����
	IloIntArray candidates(env);
	// A_f: flow����0��arc���ϣ� A_v: �г���arc����
	IloBoolArray A_f(env), A_v(env);
	// ������zfijʱÿ��arc�ϵ�flow, capacity, ʣ��capacity
	IloNumArray flow(env), capacity(env), residualCa(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		// �����������б��е�arc
		if (temp > 0 && !tabuList.contains(i))
			candidates.add(i);
		flow.add(temp);
		// A_f: sum of xijk by k
		if (temp > 0)
			A_f.add(true);
		else
			A_f.add(false);

		IloNum tempz = 0;
		IloNum tempCa = 0;
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			tempz += z[j][i];
			tempCa += z[j][i] * vehicleCapacity[j];
		}
		capacity.add(tempCa);
		residualCa.add(tempCa - temp);
		// A_v: sum of yijf by f
		if (tempz > 0)
			A_v.add(true);
		else
			A_v.add(false);
	}

	// ��arc
	IloIntArray minPathArcs(env);
	IloNum minCost = INFINITY;
	IloInt minArc = -1;
	// Local search֮ǰ���
	//SolvedByCplex(env, instance, A_f, A_v, xijk, z);
	
	// ��candidate arcs�����ŵ�move
	for (int i = 0; i < candidates.getSize(); i++)
	{
		IloIntArray pathArcs(env);
		// ��ѡarc��flowֵ
		IloNum xij = flow[candidates[i]];

		// ȷ��new network�������Ļ�
		IloIntArray arcs(env);
		IloIntArray arcIndex(env);
		for (int a = 0; a < instance->n_arcs; a++)
		{
			// �����������A_v��������
			if (!A_v[a])
				continue;
			// arcs��ֵ--2:ij+, 1:ji- xijk>flow, 0:ji- xijk=flow
			// ������candidate arc
			if (a != candidates[i] && residualCa[a] >= xij)
			{
				arcs.add(2);
				arcIndex.add(a);
			}

			if (flow[a] > xij)
			{
				arcs.add(1);
				arcIndex.add(a);
			}
			else if (flow[a] == xij)
			{
				arcs.add(0);
				arcIndex.add(a);
			}
		}

		IloNum cost = FindCycleBellmanFord(env, pathArcs, candidates[i], instance, A_f, xij, arcs, arcIndex);
		// ���δ�ҵ�����cycle��continue
		if (cost == INFINITY)
		{
			pathArcs.end();
			arcs.end();
			arcIndex.end();
			continue;
		}

		// �ҵ�cycleҲҪ���CMCF�����Ƿ����
		IloBoolArray ij_A_f(env);
		int a;
		for (a = 0; a < instance->n_arcs; a++)
			ij_A_f.add(A_f[a]);
		for (a = 0; a < pathArcs.getSize(); a++)
		{
			// A_f��û�е�arc�ӽ�ȥ
			if (!A_f[pathArcs[a]])
				ij_A_f[pathArcs[a]] = true;
			// �Ż���flowΪ0�Ĵ�A_f��ȥ��
			else if (flow[pathArcs[a]] == xij)
				ij_A_f[pathArcs[a]] = false;
		}
		IloBool Feasible = true;
		for (a = 0; a < instance->n_commods; a++)
		{
			if (!CheckCommodityFlow(env, a, instance, ij_A_f))
			{
				Feasible = false;
				break;
			}
		}
		// ����ҵ������path��A_f��������û��Ӱ��
		if (Feasible && cost < minCost)
		{
			minCost = cost;
			minArc = candidates[i];
			CopyArray(pathArcs, minPathArcs);
		}
		pathArcs.end();
		arcs.end();
		arcIndex.end();
		ij_A_f.end();
	}
	cout << "min arc: " << minArc << endl;
	cout << "min cost: " << minCost << endl;
	IloNum upperbound;
	// û���ҵ����е�move
	if (minArc == -1)
		upperbound = -1;
		//upperbound = SolvedByTwoModel(env, instance, A_f, A_v, xijk, z, capacity);
	else
	{
		// ����arc����
		for (int i = 0; i < minPathArcs.getSize(); i++)
		{
			// A_f��û�е�arc�ӽ�ȥ
			if (!A_f[minPathArcs[i]])
				A_f[minPathArcs[i]] = true;
			// �Ż���flowΪ0�Ĵ�A_f��ȥ��
			else if (flow[minPathArcs[i]] == flow[minArc])
				A_f[minPathArcs[i]] = false;
		}
		// ��֤���ż������ܷ�������Ž�
		//for (int i = 0; i < instance->n_arcs; i++)
		//	A_f[i] = A_v[i] = 1;
		////A_f[0] = A_v[0] = 0;
		//A_f[1] = A_v[1] = 0;
		////A_f[2] = A_v[2] = 0;
		//A_f[3] = A_v[3] = 0;
		////A_f[4] = A_v[4] = 0;
		////A_f[5] = A_v[5] = 0;
		//A_f[6] = A_v[6] = 0;
		////A_f[7] = A_v[7] = 0;
		//A_f[8] = A_v[8] = 0;
		//A_f[10] = A_v[10] = 0;
		//A_f[14] = A_v[14] = 0;

		//upperbound = SolvedByCplex(env, instance, A_f, A_v, xijk, z);
		upperbound = SolvedByTwoModel(env, instance, A_f, A_v, xijk, z, capacity);
		// �ӵ�tabuList��
		tabuList.add(minArc);
	}
	
	minPathArcs.end();
	candidates.end();
	flow.end();
	capacity.end();
	residualCa.end();
	A_f.end();
	A_v.end();
	return upperbound;
}

IloNum SolvedByTwoModel(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z,
	IloNumArray arcCapacity)
{
	// CMCF model
	IloModel CMCFmodel(env);
	IloObjective CMCFcost = IloAdd(CMCFmodel, IloMinimize(env));
	// x_k_ij
	IloNumVarArray2 x(env, instance->n_commods);
	for (int i = 0; i < instance->n_commods; i++)
	{
		x[i] = IloNumVarArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_f[j])
				x[i].add(IloNumVar(CMCFcost(instance->arc_commod_ucost[j][i])));
			else
				x[i].add(IloNumVar(env));

			//x[i].add(IloNumVar(env));
			//// ���arc�ڼ���������뵽ģ����
			//if (A_f[j])
			//	CMCFcost.setLinearCoef(x[i][j], instance->arc_commod_ucost[j][i]);
		}
	}
	// capacity constraints
	for (int i = 0; i < instance->n_arcs; i++)
	{
		if (A_f[i])
		{
			IloExpr left(env);
			for (int j = 0; j < instance->n_commods; j++)
				left += x[j][i];
			CMCFmodel.add(left <= arcCapacity[i]);
			left.end();
		}
	}
	// Demand Constraints	
	for (int i = 0; i < instance->n_nodes; i++)
	{
		for (int j = 0; j < instance->n_commods; j++)
		{
			IloExpr left(env);
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_f[instance->node_outgoing_arc[i][k]])
					left += x[j][instance->node_outgoing_arc[i][k]];
			}

			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_f[instance->node_ingoing_arc[i][k]])
					left -= x[j][instance->node_ingoing_arc[i][k]];
			}
			CMCFmodel.add(left == instance->node_commod_supply[i][j]);
			left.end();
		}
	}
	IloCplex CMCFSolver(CMCFmodel);
	CMCFSolver.setOut(env.getNullStream());
	
	if (!CMCFSolver.solve())
	{
		cout << "CMCF model infeasible!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		// Free memory
		for (int i = 0; i < x.getSize(); i++)
			x[i].end();
		x.end();
		CMCFmodel.end();
		CMCFSolver.end();
		return -1;
	}
	// ����xijk��ֵ
	IloNum CMCFtotalcost = 0;
	cout << " x in local search: " << endl;
	for (int i = 0; i < instance->n_commods; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_f[j])
				xijk[i][j] = CMCFSolver.getValue(x[i][j]);
			else
				xijk[i][j] = 0;
			CMCFtotalcost += xijk[i][j] * instance->arc_commod_ucost[j][i];
			//cout << xijk[i][j] << " ";
		}
		//cout << endl;
	}
	// ����A_f
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		// A_f: sum of xijk by k
		if (temp == 0 && A_f[i] == true)
			A_f[i] = false;
	}


	IloNumArray flowij(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// ����A_v
		if (A_f[i])
		{
			A_v[i] = true;
			IloNum temp = 0;
			for (int j = 0; j < instance->n_commods; j++)
				temp += CMCFSolver.getValue(x[j][i]);
			flowij.add(temp);
		}
		else
			flowij.add(0);

	}
	
	// ��������ģ��
	// Heterogeneous Fleet Assginment model
	IloModel HFAmodel(env);
	IloObjective HFAcost = IloAdd(HFAmodel, IloMinimize(env));

	// y_f_ij
	IloIntVarArray2 y(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		y[i] = IloIntVarArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			// y������A_v
			if (A_v[j])
				y[i].add(IloIntVar(HFAcost(vehicleFixedcost[i])));
			else
				y[i].add(IloIntVar(env));
		}
	}
	
	// capacity constraints
	for (int i = 0; i < instance->n_arcs; i++)
	{
		if (A_f[i])
		{
			IloExpr left(env);
			for (int j = 0; j < n_VehicleTypes; j++)
				left += vehicleCapacity[j] * y[j][i];
			HFAmodel.add(left >= flowij[i]);
			left.end();
		}
	}
	// Design-balanced constraints
	for (int i = 0; i < instance->n_nodes; i++)
	{
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			// dbԼ������A_v
			IloExpr left(env);
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_v[instance->node_outgoing_arc[i][k]])
					left += y[j][instance->node_outgoing_arc[i][k]];
			}


			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_v[instance->node_ingoing_arc[i][k]])
					left -= y[j][instance->node_ingoing_arc[i][k]];
			}
			HFAmodel.add(left == 0);
			left.end();
		}
	}
	IloCplex HFAsolver(HFAmodel);
	HFAsolver.setParam(IloCplex::TiLim, 100);
	HFAsolver.setOut(env.getNullStream());
	if (!HFAsolver.solve())
	{
		cout << "-----------%%%%%%%%%%%%%%Local search fails!!!!!!!!!!!!!!" << endl;
		// Free memory
		for (int i = 0; i < y.getSize(); i++)
			y[i].end();
		for (int i = 0; i < x.getSize(); i++)
			x[i].end();
		y.end();
		x.end();
		flowij.end();
		CMCFmodel.end();
		CMCFSolver.end();
		HFAmodel.end();
		HFAsolver.end();
		return INFINITY;
	}

	IloNum totalCost = CMCFtotalcost + HFAsolver.getObjValue();
	cout << "--------------------The Upper Bound by Local Search----------- :" << totalCost << endl;


	// ����yfij��ֵ
	cout << "y in local search: " << endl;
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_v[j])
				z[i][j] = HFAsolver.getValue(y[i][j]);
			else
				z[i][j] = 0;
			//cout << z[i][j] << " ";
		}
		//cout << endl;
	}

	IloIntArray tabuList(env);
	// ���ж�μ�ǿ�Ż�
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			// �ҵ����Ž������totalCost
			if (inteResult < totalCost)
				totalCost = inteResult;
			//system("pause");
		}
	}

	// ���Ż���������
	// �������Ч����û������
	//HFAModel(env, instance, A_v, xijk);

	// Free memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	for (int i = 0; i < x.getSize(); i++)
		x[i].end();
	y.end();
	x.end();
	flowij.end();
	CMCFmodel.end();
	CMCFSolver.end();
	HFAmodel.end();
	HFAsolver.end();
	tabuList.end();
	return totalCost;
}

// �����������arc�����²���flow�������������ʱ���Գ������Ż�����
IloNum HFAModel(IloEnv env, cmnd_t* instance, IloBoolArray A_v, IloNumArray2 xijk)
{
	IloNumArray flowij(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// ����A_v
		if (!A_v[i])
			continue;

		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		flowij.add(temp);
	}

	// ��������ģ��
	// Heterogeneous Fleet Assginment model
	IloModel HFAmodel(env);
	IloObjective HFAcost = IloAdd(HFAmodel, IloMinimize(env));
	// Capacity Constraints	
	IloNumArray capacityUpperBound(env);
	capacityUpperBound.add(flowij.getSize(), IloInfinity);
	IloRangeArray  capacityConstraints = IloAdd(HFAmodel, IloRangeArray(env, flowij, capacityUpperBound));

	// y_f_ij
	IloIntVarArray2 y(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		y[i] = IloIntVarArray(env);
		IloInt count = 0;
		for (int j = 0; j < instance->n_arcs; j++)
		{
			y[i].add(IloIntVar(env));
			// capacityԼ������A_f
			if (A_v[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y������A_v
			if (A_v[j])
				HFAcost.setLinearCoef(y[i][j], vehicleFixedcost[i]);
		}
	}
	// Design-balanced constraints
	IloRangeArray2 dbConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		dbConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			// dbԼ������A_v
			dbConstraints[i].add(IloRange(env, 0, 0));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_v[instance->node_outgoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_outgoing_arc[i][k]], 1);
			}


			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_v[instance->node_ingoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_ingoing_arc[i][k]], -1);
			}
		}
		HFAmodel.add(dbConstraints[i]);
	}


	IloCplex HFAsolver(HFAmodel);
	HFAsolver.setOut(env.getNullStream());
	if (!HFAsolver.solve())
	{
		cout << "-----------%%%%%%%%%%%%%%Local search fails!!!!!!!!!!!!!!" << endl;
	}
	IloNum totalCost = 0;
	for (int a = 0; a < instance->n_commods; a++)
	{
		for (int b = 0; b < instance->n_arcs; b++)
		{
			totalCost += xijk[a][b] * instance->arc_commod_ucost[b][a];
		}
	}
	totalCost += HFAsolver.getObjValue();
	cout << "--------------------The Upper Bound by Local Search----------- :" << totalCost << endl;
	//printf("\n--------------------The Upper Bound by Local Search-----------: %f \n\n", totalCost);

	// Free memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	y.end();
	capacityUpperBound.end();
	capacityConstraints.end();
	for (int i = 0; i < dbConstraints.getSize(); i++)
		dbConstraints[i].end();
	dbConstraints.end();
	flowij.end();
	HFAmodel.end();
	HFAsolver.end();
	return totalCost;
}
// ��Local Search���һ���Ż���
IloNum Intensification(IloEnv env, cmnd_t* instance, IloNumArray2 xkij, IloNumArray2 yfij, IloIntArray tabuList)
{
	IloNum IntensificationResult = INFINITY;
	// ��װ������͵�arc����ʣ��ռ�����ij
	IloInt maxArc = -1;
	IloNum maxResidualLoad = -1;
	IloNumArray xij(env), yij(env);
	IloNum tempy, tempx;
	for (int i = 0; i < instance->n_arcs; i++)
	{
		tempy = 0;
		for (int j = 0; j < n_VehicleTypes; j++)
			tempy += yfij[j][i] * vehicleCapacity[j];
		yij.add(tempy);
		tempx = 0;
		for (int j = 0; j < instance->n_commods; j++)
			tempx += xkij[j][i];
		xij.add(tempx);
		// ���arc��û������
		if (tempx == 0)
			continue;
		if (tempy - tempx > maxResidualLoad && !tabuList.contains(i))
		{
			maxResidualLoad = tempy - tempx;
			maxArc = i;
		}
	}
	if (maxArc == -1)
	{
		xij.end();
		yij.end();
		return INFINITY;
	}
	// ��maxArc�ӵ�tabuList��
	tabuList.add(maxArc);
	IloIntArray yCycle(env);
	IloIntArray xPath(env);
	// ������С�ĳ���
	for (int i = n_VehicleTypes - 1; i > 0; i--)
	{
		// ��yfij�г������ķ���ֵ
		if (yfij[i][maxArc] > 0)
		{
			// �Ҹó��������γ�cycle��arcs
			IloIntArray tempArcs(env);
			tempArcs.add(maxArc);
			for (int j = 0; j < instance->n_arcs; j++)
			{
				if (j == maxArc)
					continue;
				// �жϿ����γ�cycle��arcs��Ҫ�ڼ��ٳ���������������
				if (yfij[i][j] > 0 && yij[j] - vehicleCapacity[i] + vehicleCapacity[i-1] - xij[j] >= 0)
					tempArcs.add(j);
			}
			// �ҵ�cycle
			IloNum cycleResult = FindCycleIntensification(env, yCycle, maxArc, instance, tempArcs);
			tempArcs.end();
			// ����Ҳ�������������cylce�����ѭ��
			if (cycleResult == INFINITY)
				continue;
			// ����yfij��cycle��arcĿǰ���͵ĳ�������һ��Сһ�ŵĳ��ͳ�������һ
			for (int j = 0; j < yCycle.getSize(); j++)
			{
				yfij[i][yCycle[j]]--;
				yfij[i - 1][yCycle[j]]++;
				// ����yij
				yij[yCycle[j]] = yij[yCycle[j]] - vehicleCapacity[i] + vehicleCapacity[i - 1];
			}
			// �жϷ��Ϸ���������residual arc
			IloBool success = false;
			IloNum residualFlow = xij[maxArc] - yij[maxArc];
			for (int j = 0; j < instance->n_commods; j++)
			{
				if (xkij[j][maxArc] >= residualFlow)
				{
					IloIntArray tempPath(env);
					// ����һ������ķ���arc���Ա��γ�cycle
					tempPath.add(-1);
					for (int k = 0; k < instance->n_arcs; k++)
					{
						// ������ѡ����arc
						if (k == maxArc)
							continue;
						if (yij[k] - xij[k] >= residualFlow)
							tempPath.add(k);
					}
					IloNum pathResult = FindCycleIntensification(env, xPath, maxArc, instance, tempPath);
					tempPath.end();
					if (pathResult == INFINITY)
						continue;
					// ����п��е�path�������xkij��maxArc������path��
					for (int k = 0; k < xPath.getSize(); k++)
					{
						if (xPath[k] != -1)
							xkij[j][xPath[k]] += residualFlow;
						else
							xkij[j][maxArc] -= residualFlow;
					}
					// ������е��˴����ʾ�ɹ��ҵ��Ľ���
					success = true;
					break;
				}
			}
			// ���û���ҵ�xPath�ĸĽ��⣬��Ҫ��ԭ��yfijֵ�ĸı䣬��continue��ѭ��
			if (!success)
			{
				for (int j = 0; j < yCycle.getSize(); j++)
				{
					yfij[i][yCycle[j]]++;
					yfij[i - 1][yCycle[j]]--;
					// ����yij
					yij[yCycle[j]] = yij[yCycle[j]] + vehicleCapacity[i] - vehicleCapacity[i - 1];
				}
				continue;
			}
			// �ɹ��ҵ��Ľ���������ѭ��
			// ���¸Ľ����
			IloNum tempResult = 0;
			//cout << "Intensification x: " << endl;
			for (int a = 0; a < instance->n_commods; a++)
			{
				for (int b = 0; b < instance->n_arcs; b++)
				{
					tempResult += xkij[a][b] * instance->arc_commod_ucost[b][a];
					//cout << xkij[a][b] << " ";
				}
				//cout << endl;
			}
			//cout << "Intensification y: " << endl;
			for (int a = 0; a < n_VehicleTypes; a++)
			{
				for (int b = 0; b < instance->n_arcs; b++)
				{
					tempResult += yfij[a][b] * vehicleFixedcost[a];
					//cout << yfij[a][b] << " ";
				}
				//cout << endl;
			}
			IntensificationResult = tempResult;
			// �Ľ��ɹ���arc���ӵ�tabuList��
			tabuList.remove(tabuList.getSize() - 1);
			break;
		}
	}
	xij.end();
	yij.end();
	yCycle.end();
	xPath.end();
	// ������ʱ���������
	return IntensificationResult;
}



IloNum SolvedByOneModel(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z)
{
	// ����xijk��ֵ
	IloNum CMCFtotalcost = 0;
	cout << " x in local search: " << endl;
	for (int i = 0; i < instance->n_commods; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			CMCFtotalcost += xijk[i][j] * instance->arc_commod_ucost[j][i];
			//cout << xijk[i][j] << " ";
		}
		//cout << endl;
	}

	IloNumArray flowij(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// ����A_v
		/*if (A_f[i])
		A_v[i] = true;
		else
		continue;*/
		if (!A_v[i])
			continue;
		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		flowij.add(temp);
	}


	// ��������ģ��
	// Heterogeneous Fleet Assginment model
	IloModel HFAmodel(env);
	IloObjective HFAcost = IloAdd(HFAmodel, IloMinimize(env));
	// Capacity Constraints	
	IloNumArray capacityUpperBound(env);
	capacityUpperBound.add(flowij.getSize(), IloInfinity);
	IloRangeArray  capacityConstraints = IloAdd(HFAmodel, IloRangeArray(env, flowij, capacityUpperBound));

	// y_f_ij
	IloIntVarArray2 y(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		y[i] = IloIntVarArray(env);
		IloInt count = 0;
		for (int j = 0; j < instance->n_arcs; j++)
		{
			y[i].add(IloIntVar(env));
			// capacityԼ������A_f
			if (A_v[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y������A_v
			if (A_v[j])
				HFAcost.setLinearCoef(y[i][j], vehicleFixedcost[i]);
		}
	}
	// Design-balanced constraints
	IloRangeArray2 dbConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		dbConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			// dbԼ������A_v
			dbConstraints[i].add(IloRange(env, 0, 0));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_v[instance->node_outgoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_outgoing_arc[i][k]], 1);
			}


			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_v[instance->node_ingoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_ingoing_arc[i][k]], -1);
			}
		}
		HFAmodel.add(dbConstraints[i]);
	}


	IloCplex HFAsolver(HFAmodel);
	HFAsolver.setParam(IloCplex::TiLim, 100);
	HFAsolver.setOut(env.getNullStream());
	if (!HFAsolver.solve())
	{
		cout << "-----------%%%%%%%%%%%%%%Local search fails!!!!!!!!!!!!!!" << endl;
		// Free memory
		for (int i = 0; i < y.getSize(); i++)
			y[i].end();
		y.end();
		capacityUpperBound.end();
		capacityConstraints.end();
		for (int i = 0; i < dbConstraints.getSize(); i++)
			dbConstraints[i].end();
		dbConstraints.end();
		flowij.end();
		HFAmodel.end();
		HFAsolver.end();
		return INFINITY;
	}

	IloNum totalCost = CMCFtotalcost + HFAsolver.getObjValue();
	cout << "--------------------The Upper Bound by Local Search----------- :" << totalCost << endl;


	// ����yfij��ֵ
	//cout << "y in local search: " << endl;
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_v[j])
				z[i][j] = HFAsolver.getValue(y[i][j]);
			else
				z[i][j] = 0;
			//cout << z[i][j] << " ";
		}
		//cout << endl;
	}

	IloIntArray tabuList(env);
	// ���ж�μ�ǿ�Ż�
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			// �ҵ����Ž������totalCost
			if (inteResult < totalCost)
				totalCost = inteResult;
			//system("pause");
		}
	}
	// ���Ż���������
	// �������Ч����û������
	//HFAModel(env, instance, A_v, xijk);

	// Free memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	y.end();
	capacityUpperBound.end();
	capacityConstraints.end();
	for (int i = 0; i < dbConstraints.getSize(); i++)
		dbConstraints[i].end();
	dbConstraints.end();
	flowij.end();
	HFAmodel.end();
	HFAsolver.end();
	tabuList.end();
	return totalCost;
}
IloNum SolvedByCplex(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z)
{
	// CMCF model
	IloModel CMCFmodel(env);
	IloObjective CMCFcost = IloAdd(CMCFmodel, IloMinimize(env));
	// x_k_ij
	IloNumVarArray2 x(env, instance->n_commods);
	for (int i = 0; i < instance->n_commods; i++)
	{
		x[i] = IloNumVarArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			x[i].add(IloNumVar(env));
			// ���arc�ڼ���������뵽ģ����
			if (A_f[j])
				CMCFcost.setLinearCoef(x[i][j], instance->arc_commod_ucost[j][i]);
		}
	}
	// Demand Constraints	
	IloRangeArray2 demandConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		demandConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < instance->n_commods; j++)
		{
			demandConstraints[i].add(IloRange(env, instance->node_commod_supply[i][j], instance->node_commod_supply[i][j]));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_f[instance->node_outgoing_arc[i][k]])
					demandConstraints[i][j].setLinearCoef(x[j][instance->node_outgoing_arc[i][k]], 1);
			}

			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_f[instance->node_ingoing_arc[i][k]])
					demandConstraints[i][j].setLinearCoef(x[j][instance->node_ingoing_arc[i][k]], -1);
			}
		}
		CMCFmodel.add(demandConstraints[i]);
	}
	IloCplex CMCFSolver(CMCFmodel);
	CMCFSolver.setOut(env.getNullStream());
	if (!CMCFSolver.solve())
	{
		cout << "CMCF model infeasible!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		// Free memory
		for (int i = 0; i < x.getSize(); i++)
			x[i].end();
		x.end();
		for (int i = 0; i < demandConstraints.getSize(); i++)
			demandConstraints[i].end();
		demandConstraints.end();
		CMCFmodel.end();
		CMCFSolver.end();
		return INFINITY;
	}
	//cout << "CMCF obj: " << CMCFSolver.getObjValue() << endl;

	// ����xijk��ֵ
	cout << " x in local search: " << endl;
	for (int i = 0; i < instance->n_commods; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_f[j])
				xijk[i][j] = CMCFSolver.getValue(x[i][j]);
			else
				xijk[i][j] = 0;
			//cout << xijk[i][j] << " ";
		}
		//cout << endl;
	}
	// ����A_f
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		// A_f: sum of xijk by k
		if (temp == 0 && A_f[i] == true)
			A_f[i] = false;
	}


	IloNumArray flowij(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// ����A_v
		if (A_f[i])
			A_v[i] = true;
		else
			continue;

		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += CMCFSolver.getValue(x[j][i]);
		flowij.add(temp);
	}


	// ��������ģ��
	// Heterogeneous Fleet Assginment model
	IloModel HFAmodel(env);
	IloObjective HFAcost = IloAdd(HFAmodel, IloMinimize(env));
	// Capacity Constraints	
	IloNumArray capacityUpperBound(env);
	capacityUpperBound.add(flowij.getSize(), IloInfinity);
	IloRangeArray  capacityConstraints = IloAdd(HFAmodel, IloRangeArray(env, flowij, capacityUpperBound));

	// y_f_ij
	IloIntVarArray2 y(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		y[i] = IloIntVarArray(env);
		IloInt count = 0;
		for (int j = 0; j < instance->n_arcs; j++)
		{
			y[i].add(IloIntVar(env));
			// capacityԼ������A_f
			if (A_f[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y������A_v
			if (A_v[j])
				HFAcost.setLinearCoef(y[i][j], vehicleFixedcost[i]);
		}
	}
	// Design-balanced constraints
	IloRangeArray2 dbConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		dbConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			// dbԼ������A_v
			dbConstraints[i].add(IloRange(env, 0, 0));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
			{
				if (A_v[instance->node_outgoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_outgoing_arc[i][k]], 1);
			}


			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
			{
				if (A_v[instance->node_ingoing_arc[i][k]])
					dbConstraints[i][j].setLinearCoef(y[j][instance->node_ingoing_arc[i][k]], -1);
			}
		}
		HFAmodel.add(dbConstraints[i]);
	}


	IloCplex HFAsolver(HFAmodel);
	HFAsolver.setParam(IloCplex::TiLim, 100);
	HFAsolver.setOut(env.getNullStream());
	if (!HFAsolver.solve())
	{
		cout << "-----------%%%%%%%%%%%%%%Local search fails!!!!!!!!!!!!!!" << endl;
		// Free memory
		for (int i = 0; i < y.getSize(); i++)
			y[i].end();
		for (int i = 0; i < x.getSize(); i++)
			x[i].end();
		y.end();
		x.end();
		capacityUpperBound.end();
		capacityConstraints.end();
		for (int i = 0; i < demandConstraints.getSize(); i++)
			demandConstraints[i].end();
		demandConstraints.end();
		for (int i = 0; i < dbConstraints.getSize(); i++)
			dbConstraints[i].end();
		dbConstraints.end();
		flowij.end();
		CMCFmodel.end();
		CMCFSolver.end();
		HFAmodel.end();
		HFAsolver.end();
		return INFINITY;
	}
	//cout << "HFA obj: " << HFAsolver.getObjValue() << endl;
	IloNum totalCost = CMCFSolver.getObjValue() + HFAsolver.getObjValue();
	cout << "--------------------The Upper Bound by Local Search----------- :" << totalCost << endl;

	// ����yfij��ֵ
	cout << "y in local search: " << endl;
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			if (A_v[j])
				z[i][j] = HFAsolver.getValue(y[i][j]);
			else
				z[i][j] = 0;
			//cout << z[i][j] << " ";
		}
		//cout << endl;
	}

	IloIntArray tabuList(env);
	// ���ж�μ�ǿ�Ż�
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			totalCost = inteResult;
		}
	}
	// ���Ż���������
	HFAModel(env, instance, A_v, xijk);

	// Free memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	for (int i = 0; i < x.getSize(); i++)
		x[i].end();
	y.end();
	x.end();
	capacityUpperBound.end();
	capacityConstraints.end();
	for (int i = 0; i < demandConstraints.getSize(); i++)
		demandConstraints[i].end();
	demandConstraints.end();
	for (int i = 0; i < dbConstraints.getSize(); i++)
		dbConstraints[i].end();
	dbConstraints.end();
	flowij.end();
	CMCFmodel.end();
	CMCFSolver.end();
	HFAmodel.end();
	HFAsolver.end();
	return totalCost;
}
UpperBound::UpperBound()
{
}


UpperBound::~UpperBound()
{
}
