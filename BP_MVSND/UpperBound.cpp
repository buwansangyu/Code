#include "UpperBound.h"
void CopyArray(IloIntArray oldArray, IloIntArray newArray)
{
	newArray.clear();
	for (int i = 0; i < oldArray.getSize(); i++)
		newArray.add(oldArray[i]);
}
IloNum LocalSearch(IloEnv env, cmnd_t* instance, IloNumArray2 xijk, IloNumArray2 z, IloIntArray tabuList)
{
	// 找flow大于0的弧作为候选对象
	IloIntArray candidates(env);
	// A_f: flow大于0的arc集合， A_v: 有车的arc集合
	IloBoolArray A_f(env), A_v(env);
	// 给定解zfij时每条arc上的flow, capacity, 剩余capacity
	IloNumArray flow(env), capacity(env), residualCa(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		// 不包含禁忌列表中的arc
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

	// 关arc
	IloIntArray minPathArcs(env);
	IloNum minCost = INFINITY;
	IloInt minArc = -1;
	// Local search之前求解
	//SolvedByCplex(env, instance, A_f, A_v, xijk, z);
	
	// 对candidate arcs找最优的move
	for (int i = 0; i < candidates.getSize(); i++)
	{
		IloIntArray pathArcs(env);
		// 候选arc上flow值
		IloNum xij = flow[candidates[i]];

		// 确定new network所包含的弧
		IloIntArray arcs(env);
		IloIntArray arcIndex(env);
		for (int a = 0; a < instance->n_arcs; a++)
		{
			// 如果不包含在A_v中则跳过
			if (!A_v[a])
				continue;
			// arcs的值--2:ij+, 1:ji- xijk>flow, 0:ji- xijk=flow
			// 不包含candidate arc
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
		// 如果未找到可行cycle则continue
		if (cost == INFINITY)
		{
			pathArcs.end();
			arcs.end();
			arcIndex.end();
			continue;
		}

		// 找到cycle也要检查CMCF问题是否可行
		IloBoolArray ij_A_f(env);
		int a;
		for (a = 0; a < instance->n_arcs; a++)
			ij_A_f.add(A_f[a]);
		for (a = 0; a < pathArcs.getSize(); a++)
		{
			// A_f中没有的arc加进去
			if (!A_f[pathArcs[a]])
				ij_A_f[pathArcs[a]] = true;
			// 优化后flow为0的从A_f中去掉
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
		// 如果找到的替代path在A_f集合中则没有影响
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
	// 没有找到可行的move
	if (minArc == -1)
		upperbound = -1;
		//upperbound = SolvedByTwoModel(env, instance, A_f, A_v, xijk, z, capacity);
	else
	{
		// 更新arc集合
		for (int i = 0; i < minPathArcs.getSize(); i++)
		{
			// A_f中没有的arc加进去
			if (!A_f[minPathArcs[i]])
				A_f[minPathArcs[i]] = true;
			// 优化后flow为0的从A_f中去掉
			else if (flow[minPathArcs[i]] == flow[minArc])
				A_f[minPathArcs[i]] = false;
		}
		// 验证最优集合下能否求得最优解
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
		// 加到tabuList中
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
			//// 如果arc在集合中则加入到模型中
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
	// 更新xijk的值
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
	// 更新A_f
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
		// 更新A_v
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
	
	// 车辆分配模型
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
			// y变量用A_v
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
			// db约束中用A_v
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


	// 更新yfij的值
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
	// 进行多次加强优化
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			// 找到较优解则更新totalCost
			if (inteResult < totalCost)
				totalCost = inteResult;
			//system("pause");
		}
	}

	// 最优化车辆分配
	// 加上求解效果并没有提升
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

// 用于求解最优arc集合下并且flow分配最优情况下时，对车辆最优化分配
IloNum HFAModel(IloEnv env, cmnd_t* instance, IloBoolArray A_v, IloNumArray2 xijk)
{
	IloNumArray flowij(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// 更新A_v
		if (!A_v[i])
			continue;

		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += xijk[j][i];
		flowij.add(temp);
	}

	// 车辆分配模型
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
			// capacity约束中用A_f
			if (A_v[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y变量用A_v
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
			// db约束中用A_v
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
// 在Local Search后进一步优化解
IloNum Intensification(IloEnv env, cmnd_t* instance, IloNumArray2 xkij, IloNumArray2 yfij, IloIntArray tabuList)
{
	IloNum IntensificationResult = INFINITY;
	// 找装载率最低的arc，即剩余空间最多的ij
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
		// 如果arc上没有流量
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
	// 把maxArc加到tabuList中
	tabuList.add(maxArc);
	IloIntArray yCycle(env);
	IloIntArray xPath(env);
	// 除了最小的车型
	for (int i = n_VehicleTypes - 1; i > 0; i--)
	{
		// 找yfij中车型最大的非零值
		if (yfij[i][maxArc] > 0)
		{
			// 找该车型下能形成cycle的arcs
			IloIntArray tempArcs(env);
			tempArcs.add(maxArc);
			for (int j = 0; j < instance->n_arcs; j++)
			{
				if (j == maxArc)
					continue;
				// 判断可以形成cycle的arcs，要在减少车辆后仍满足条件
				if (yfij[i][j] > 0 && yij[j] - vehicleCapacity[i] + vehicleCapacity[i-1] - xij[j] >= 0)
					tempArcs.add(j);
			}
			// 找到cycle
			IloNum cycleResult = FindCycleIntensification(env, yCycle, maxArc, instance, tempArcs);
			tempArcs.end();
			// 如果找不到满足条件的cylce则继续循环
			if (cycleResult == INFINITY)
				continue;
			// 更新yfij，cycle中arc目前车型的车辆数减一，小一号的车型车辆数加一
			for (int j = 0; j < yCycle.getSize(); j++)
			{
				yfij[i][yCycle[j]]--;
				yfij[i - 1][yCycle[j]]++;
				// 更新yij
				yij[yCycle[j]] = yij[yCycle[j]] - vehicleCapacity[i] + vehicleCapacity[i - 1];
			}
			// 判断符合分流条件的residual arc
			IloBool success = false;
			IloNum residualFlow = xij[maxArc] - yij[maxArc];
			for (int j = 0; j < instance->n_commods; j++)
			{
				if (xkij[j][maxArc] >= residualFlow)
				{
					IloIntArray tempPath(env);
					// 增加一个虚拟的反向arc，以便形成cycle
					tempPath.add(-1);
					for (int k = 0; k < instance->n_arcs; k++)
					{
						// 不包含选定的arc
						if (k == maxArc)
							continue;
						if (yij[k] - xij[k] >= residualFlow)
							tempPath.add(k);
					}
					IloNum pathResult = FindCycleIntensification(env, xPath, maxArc, instance, tempPath);
					tempPath.end();
					if (pathResult == INFINITY)
						continue;
					// 如果有可行的path，则更新xkij，maxArc分流到path上
					for (int k = 0; k < xPath.getSize(); k++)
					{
						if (xPath[k] != -1)
							xkij[j][xPath[k]] += residualFlow;
						else
							xkij[j][maxArc] -= residualFlow;
					}
					// 如果运行到此处则表示成功找到改进解
					success = true;
					break;
				}
			}
			// 如果没有找到xPath的改进解，则要还原对yfij值的改变，并continue外循环
			if (!success)
			{
				for (int j = 0; j < yCycle.getSize(); j++)
				{
					yfij[i][yCycle[j]]++;
					yfij[i - 1][yCycle[j]]--;
					// 更新yij
					yij[yCycle[j]] = yij[yCycle[j]] + vehicleCapacity[i] - vehicleCapacity[i - 1];
				}
				continue;
			}
			// 成功找到改进解则跳出循环
			// 更新改进结果
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
			// 改进成功的arc不加到tabuList中
			tabuList.remove(tabuList.getSize() - 1);
			break;
		}
	}
	xij.end();
	yij.end();
	yCycle.end();
	xPath.end();
	// 不可行时返回无穷大
	return IntensificationResult;
}



IloNum SolvedByOneModel(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z)
{
	// 更新xijk的值
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
		// 更新A_v
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


	// 车辆分配模型
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
			// capacity约束中用A_f
			if (A_v[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y变量用A_v
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
			// db约束中用A_v
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


	// 更新yfij的值
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
	// 进行多次加强优化
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			// 找到较优解则更新totalCost
			if (inteResult < totalCost)
				totalCost = inteResult;
			//system("pause");
		}
	}
	// 最优化车辆分配
	// 加上求解效果并没有提升
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
			// 如果arc在集合中则加入到模型中
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

	// 更新xijk的值
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
	// 更新A_f
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
		// 更新A_v
		if (A_f[i])
			A_v[i] = true;
		else
			continue;

		IloNum temp = 0;
		for (int j = 0; j < instance->n_commods; j++)
			temp += CMCFSolver.getValue(x[j][i]);
		flowij.add(temp);
	}


	// 车辆分配模型
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
			// capacity约束中用A_f
			if (A_f[j])
			{
				capacityConstraints[count].setLinearCoef(y[i][j], vehicleCapacity[i]);
				count++;
			}
			// y变量用A_v
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
			// db约束中用A_v
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

	// 更新yfij的值
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
	// 进行多次加强优化
	for (int i = 0; i < instance->n_arcs; i++)
	{
		IloNum inteResult = Intensification(env, instance, xijk, z, tabuList);
		if (inteResult != INFINITY)
		{
			cout << "The total cost after intensification is: " << inteResult << endl;
			totalCost = inteResult;
		}
	}
	// 最优化车辆分配
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
