#pragma once
#include"LibrariesDefinitions.h"
#include"Instance.h"
// ���ǿ��Ч����ʽ
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
	IloIntArray2 cutSet; // �
	IloIntArray2 remainSet; // ����
	IloIntArray2 arcIndex; // ���������arcs
	IloNumArray totalDemand; // ���������������
	

	// ���ɸ
	IloInt CutsetInequality::FindN(IloIntArray cutset, IloIntArray Nbar);
	// ����
	void CutsetInequality::OneGenerateCutset();
	void CutsetInequality::GenerateCutset(IloInt nodeNum);
	// ��Ӹ����ʽ
	void CutsetInequality::AddCutsetInequality(IloRangeArray tempCons, IloNumVarArray2 z);


	CutsetInequality(IloEnv _env, IloCplex masterSolver, cmnd_t* _instance, IloNumVarArray2 _z);
	~CutsetInequality();
};

