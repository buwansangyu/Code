#include "CutsetInequality.h"
#include <time.h>
#include"stdlib.h"
//取得[a,b)的随机整数
#define random(a,b)(rand()%(b-a) + a);

void AddStrongValidInequality(IloEnv env, cmnd_t* instance, IloNumArray2 zValue, IloCplex masterSolver, 
	IloRangeArray tempStrongCons, IloNumVarArray2 xijk, IloNumVarArray2 z, IloBoolArray2 kij, IloBool rootNode)
{
	for (int i = 0; i < instance->n_commods; i++)
	{
		for (int j = 0; j < instance->n_arcs; j++)
		{
			// 判断强有效不等式是否已添加
			if (kij[i][j])
				break;
			IloNum rightTemp = 0;
			for (int k = 0; k < n_VehicleTypes; k++)
				rightTemp += zValue[k][j];
			if (masterSolver.getValue(xijk[i][j]) > instance->commod_supply[i] * rightTemp)
			{
				IloExpr rightExpr(env);
				for (int k = 0; k < n_VehicleTypes; k++)
					rightExpr += z[k][j] * instance->commod_supply[i];
				rightExpr -= xijk[i][j];
				tempStrongCons.add(0 <= rightExpr);
				cout << "Add strrrrrrrrrrrrrrrong valid inequality@@@@@@@@@@@@@" << endl;
				rightExpr.end();
				// 根节点标记不等式为已添加，其他节点不标记
				if (rootNode)
					kij[i][j] = true;
			}
		}
	}
}

// 返回分数部分的值
IloNum ReturnFractional(IloNum x)
{
	IloNum fractionalValue;
	if (x - IloRound(x) >= 0)
		fractionalValue = x - IloRound(x);
	else
		fractionalValue = x + 1 - IloRound(x);
	return fractionalValue;
}

// 找N的值，找不到的情况下返回-1
IloInt CutsetInequality::FindN(IloIntArray cutsetL, IloIntArray Nbar)
{
	// 补集
	IloIntArray remainsetL(env);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		if (!cutsetL.contains(i))
			remainsetL.add(i);
	}
	// arcs
	IloIntArray itoj(env);
	IloIntArray jtoi(env);
	for (int i = 0; i < instance->n_arcs; i++)
	{
		if (cutsetL.contains(instance->arc_orig_node[i]) && remainsetL.contains(instance->arc_dest_node[i]))
		{
			itoj.add(i);
		}
		if (remainsetL.contains(instance->arc_orig_node[i]) && cutsetL.contains(instance->arc_dest_node[i]))
		{
			jtoi.add(i);
		}
	}

	IloNum max_ij = -1;
	IloNum max_ji = -1;
	IloInt maxIndex_ij, maxIndex_ji;
	// 找出最大值
	for (int i = 0; i < itoj.getSize(); i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < n_VehicleTypes; j++)
			temp += ReturnFractional(z[j][itoj[i]]); //z[f][ij]
		// 保证找到的j在N_bar集合中，存储arc的值
		if (max_ij < temp && Nbar.contains(instance->arc_dest_node[itoj[i]]))
		{
			max_ij = temp;
			maxIndex_ij = itoj[i];
		}
	}
	for (int i = 0; i < jtoi.getSize(); i++)
	{
		IloNum temp = 0;
		for (int j = 0; j < n_VehicleTypes; j++)
			temp += ReturnFractional(z[j][jtoi[i]]); //z[f][ji]
		if (max_ji < temp && Nbar.contains(instance->arc_orig_node[jtoi[i]]))
		{
			max_ji = temp;
			maxIndex_ji = jtoi[i];
		}
	}
	remainsetL.end();
	itoj.end();
	jtoi.end();
	// 返回N
	if (max_ij > max_ji && max_ij != -1)
		return instance->arc_dest_node[maxIndex_ij];
	else if (max_ji != -1)
		return instance->arc_orig_node[maxIndex_ji];
	else
		return -1;
}

// 根据设定的M产生割集
void CutsetInequality::GenerateCutset(IloInt nodeNum)
{
	IloIntArray Nbar(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
		Nbar[i] = i;
	int l = 0;
	int nodeIndex = -1;
	// 初始化第一个cutSet
	cutSet.add(IloIntArray(env));
	while (Nbar.getSize() > 0)
	{
		// 第一个节点随机选择
		if (cutSet[l].getSize() == 0)
		{
			srand(int(time(0)));
			int index = random(0, Nbar.getSize());
			nodeIndex = Nbar[index];
		}
		else
		{
			nodeIndex = FindN(cutSet[l], Nbar);
			// Nbar为空时
			if (nodeIndex < 0)
				break;
		}
		cutSet[l].add(nodeIndex);
		// 删除Nbar中的对应的节点j
		for (int i = 0; i < Nbar.getSize(); i++)
		{
			if (Nbar[i] == nodeIndex)
			{
				Nbar.remove(i);
				break;
			}
		}

		// 如果目前cutset中节点数大于等于M
		if (cutSet[l].getSize() >= nodeNum)
		{
			l++;
			cutSet.add(IloIntArray(env));
		}
	}

	// 求出相关集合
	for (int k = 0; k < cutSet.getSize(); k++)
	{
		// 补集
		remainSet.add(IloIntArray(env));
		for (int i = 0; i < instance->n_nodes; i++)
		{
			if (!cutSet[k].contains(i))
				remainSet[k].add(i);
		}
		// arc
		arcIndex.add(IloIntArray(env));
		for (int i = 0; i < instance->n_arcs; i++)
		{
			if (cutSet[k].contains(instance->arc_orig_node[i]) && remainSet[k].contains(instance->arc_dest_node[i]))
				arcIndex[k].add(i);
		}
		// demand
		IloNum demand = 0;
		for (int i = 0; i < instance->n_commods; i++)
		{
			if (cutSet[k].contains(instance->commod_orig_node[i]) && remainSet[k].contains(instance->commod_dest_node[i]))
				demand += instance->commod_supply[i];
		}
		totalDemand.add(demand);
		// 割集上z之和Z[l][f]
		Z_cutset.add(IloNumArray(env));
		for (int i = 0; i < n_VehicleTypes; i++)
		{
			IloNum tempZ = 0;
			for (int j = 0; j < arcIndex[k].getSize(); j++)
				tempZ += z[i][arcIndex[k][j]];
			Z_cutset[k].add(tempZ);
		}
		//
		r.add(IloNumArray(env));
		for (int i = 0; i < n_VehicleTypes; i++)
			r[k].add(totalDemand[k] - IloCeil(totalDemand[k] / vehicleCapacity[i]) * vehicleCapacity[i]
				+ vehicleCapacity[i]); // IloCeil向上取整
	}

	Nbar.end();
}

void CutsetInequality::OneGenerateCutset()
{
	// 单点割集
	IloBoolArray nodeIn(env);
	nodeIn.add(instance->n_nodes, false);
	for (int i = 0; i < instance->n_commods; i++)
	{
		// Origin
		if (!nodeIn[instance->commod_orig_node[i]])
		{
			cutSet.add(IloIntArray(env));
			cutSet[cutSet.getSize() - 1].add(instance->commod_orig_node[i]);
			nodeIn[instance->commod_orig_node[i]] = true;
		}
		// Destination
		if (!nodeIn[instance->commod_dest_node[i]])
		{
			cutSet.add(IloIntArray(env));
			cutSet[cutSet.getSize() - 1].add(instance->commod_dest_node[i]);
			nodeIn[instance->commod_dest_node[i]] = true;
		}
	}
	nodeIn.end();
	// 求出相关集合
	for (int k = 0; k < cutSet.getSize(); k++)
	{
		// 补集
		remainSet.add(IloIntArray(env));
		for (int i = 0; i < instance->n_nodes; i++)
		{
			if (!cutSet[k].contains(i))
				remainSet[k].add(i);
		}
		// arc
		arcIndex.add(IloIntArray(env));
		for (int i = 0; i < instance->n_arcs; i++)
		{
			if (cutSet[k].contains(instance->arc_orig_node[i]) && remainSet[k].contains(instance->arc_dest_node[i]))
				arcIndex[k].add(i);
		}
		// demand
		IloNum demand = 0;
		for (int i = 0; i < instance->n_commods; i++)
		{
			if (cutSet[k].contains(instance->commod_orig_node[i]) && remainSet[k].contains(instance->commod_dest_node[i]))
				demand += instance->commod_supply[i];
		}
		totalDemand.add(demand);
		// 割集上z之和Z[l][f]
		Z_cutset.add(IloNumArray(env));
		for (int i = 0; i < n_VehicleTypes; i++)
		{
			IloNum tempZ = 0;
			for (int j = 0; j < arcIndex[k].getSize(); j++)
				tempZ += z[i][arcIndex[k][j]];
			Z_cutset[k].add(tempZ);
		}
		//
		r.add(IloNumArray(env));
		for (int i = 0; i < n_VehicleTypes; i++)
			r[k].add(totalDemand[k] - IloCeil(totalDemand[k] / vehicleCapacity[i]) * vehicleCapacity[i]
			+ vehicleCapacity[i]); // IloCeil向上取整
	}
}


void CutsetInequality::AddCutsetInequality(IloRangeArray tempCons, IloNumVarArray2 zVars)
{
	// 暂时只加单点割集的不等式
	for (int b = 1; b < 2; b++)
	{
		// 设定M=ceil(N/3)
		//IloInt nodeNum = IloCeil((double)instance->n_nodes / 3);
		//IloInt nodeNum = 2;
		// 生成割集以及计算相关变量的值
		GenerateCutset(b);
		// 单点割集
		//OneGenerateCutset();
		/*cout << instance->n_nodes << endl;
		cout << "cutset num: " << cutSet.getSize() << endl;*/

		for (int k = 0; k < cutSet.getSize(); k++)
		{
			//tempCons.add(IloRangeArray(env));
			for (int i = 0; i < n_VehicleTypes; i++)
			{
				IloNum leftValue = 0;
				for (int j = 0; j < n_VehicleTypes; j++)
				{
					if (i == j)
						leftValue += Z_cutset[k][j];
					else
						//leftValue += Z_cutset[k][j] * IloMin(IloCeil(vehicleCapacity[j] / vehicleCapacity[i]), vehicleCapacity[j] / r[k][i]);
						//leftValue += Z_cutset[k][j] * vehicleCapacity[j] / r[k][i];
						leftValue += Z_cutset[k][j] * IloCeil(vehicleCapacity[j] / vehicleCapacity[i]);
				}
				// 违反约束则加割
				if (leftValue < IloCeil(totalDemand[k] / vehicleCapacity[i]))
				{
					IloExpr leftExpr(env);
					for (int j = 0; j < n_VehicleTypes; j++)
					{
						if (i == j)
						{
							for (int a = 0; a < arcIndex[k].getSize(); a++)
								leftExpr += zVars[j][arcIndex[k][a]];
						}
						else
						{
							//IloNum coef = IloMin(IloCeil(vehicleCapacity[j] / vehicleCapacity[i]), vehicleCapacity[j] / r[k][i]);
							//IloNum coef = vehicleCapacity[j] / r[k][i];
							IloNum coef = IloCeil(vehicleCapacity[j] / vehicleCapacity[i]);
							for (int a = 0; a < arcIndex[k].getSize(); a++)
								leftExpr += zVars[j][arcIndex[k][a]] * coef;
						}
					}
					// 加入约束

					tempCons.add(leftExpr >= IloCeil(totalDemand[k] / vehicleCapacity[i]));
					cout << "-------------------Add cutset inequalities@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
					cout << tempCons[tempCons.getSize() - 1] << endl;
					leftExpr.end();
					break;
				}
			}
		}


	}
	
}

CutsetInequality::CutsetInequality(IloEnv _env, IloCplex masterSolver, cmnd_t* _instance, IloNumVarArray2 _z)
{
	env = _env;
	instance = _instance;

	z = IloNumArray2(env, _z.getSize());
	for (int i = 0; i < _z.getSize(); i++)
	{
		z[i] = IloNumArray(env);
		masterSolver.getValues(z[i], _z[i]);
	}
	Z_cutset = IloNumArray2(env);
	r = IloNumArray2(env);
	cutSet = IloIntArray2(env);
	remainSet = IloIntArray2(env);
	arcIndex = IloIntArray2(env);
	totalDemand = IloNumArray(env);
}


CutsetInequality::~CutsetInequality()
{
	for (int i = 0; i < cutSet.getSize(); i++)
		cutSet[i].end();
	cutSet.end();
	for (int i = 0; i < remainSet.getSize(); i++)
		remainSet[i].end();
	remainSet.end();
	for (int i = 0; i < arcIndex.getSize(); i++)
		arcIndex[i].end();
	for (int i = 0; i < z.getSize(); i++)
		z[i].end();
	z.end();
	for (int i = 0; i < Z_cutset.getSize(); i++)
		Z_cutset[i].end();
	Z_cutset.end();
	for (int i = 0; i < r.getSize(); i++)
		r[i].end();
	r.end();
	totalDemand.end();
	cout << " ~cutSetInequality " << endl;
}


