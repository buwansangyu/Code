#pragma once
#include"LibrariesDefinitions.h"
#include"Instance.h"
#include"ShortestPath.h"
IloNum SolvedByCplex(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z);
IloNum HFAModel(IloEnv env, cmnd_t* instance, IloBoolArray A_v, IloNumArray2 xijk);
IloNum SolvedByOneModel(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z);
IloNum SolvedByTwoModel(IloEnv env, cmnd_t* instance, IloBoolArray A_f, IloBoolArray A_v, IloNumArray2 xijk, IloNumArray2 z,
	IloNumArray arcCapacity);
IloNum Intensification(IloEnv env, cmnd_t* instance, IloNumArray2 xkij, IloNumArray2 yfij,IloIntArray tabuList);

IloNum LocalSearch(IloEnv env, cmnd_t* instance, IloNumArray2 xijk, IloNumArray2 z, IloIntArray tabuList);
class UpperBound
{
public:
	UpperBound();
	~UpperBound();
};

