#pragma once
#include"LibrariesDefinitions.h"
#include"Instance.h"
// 添加强有效不等式
void AddStrongValidInequality(IloEnv env, cmnd_t* instance, IloNumArray2 zValue, IloCplex masterSolver,
	IloRangeArray tempStrongCons, IloNumVarArray2 xijk, IloNumVarArray2 z, IloBoolArray2 kij, IloBool rootNode);
class CutsetInequality
{
private:


public:
	IloEnv env;
	cmnd_t* instance;
	IloNumArray2 z; //z value
	IloNumArray2 Z_cutset; // Z_cutset[l][f]
	IloNumArray2 r; // r(D,u)
	IloIntArray2 cutSet; // 割集
	IloIntArray2 remainSet; // 补集
	IloIntArray2 arcIndex; // 割集到补集的arcs
	IloNumArray totalDemand; // 割集到补集的总需求
	

	// 生成割集
	IloInt CutsetInequality::FindN(IloIntArray cutset, IloIntArray Nbar);
	// 单点割集
	void CutsetInequality::OneGenerateCutset();
	void CutsetInequality::GenerateCutset(IloInt nodeNum);
	// 添加割集不等式
	void CutsetInequality::AddCutsetInequality(IloRangeArray tempCons, IloNumVarArray2 z);


	CutsetInequality(IloEnv _env, IloCplex masterSolver, cmnd_t* _instance, IloNumVarArray2 _z);
	~CutsetInequality();
};

