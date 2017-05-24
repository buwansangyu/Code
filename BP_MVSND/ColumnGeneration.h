#pragma once

#include "Instance.h"
#include "ShortestPath.h"
IloNum HeuristicUpperBound(IloEnv env, IloModel masterModel, IloCplex masterSolver, IloNumVarArray2 y, IloNumArray2 yValue);
IloNum HeuristicUpperBoundT(IloEnv env, IloObjective totalcost, IloRangeArray capCons, IloRangeArray demCons, IloNumVarArray2 x,
	IloNumVarArray2 y, IloNumArray2 yValue);
void AddInitialColumns(IloEnv env, cmnd_t* instance, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZContraints, IloRangeArray2 EqualXijkConstraints,
	IloNumVarArray2 x, IloNumVarArray2 y);

IloNum ColumnGeneration(IloEnv env, IloCplex masterSolver, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZContraints, IloRangeArray2 EqualXijkConstraints,
	IloNumVarArray2 y, IloNumVarArray2 x, cmnd_t* instance);


IloNum InequalityCG(IloEnv env, IloCplex masterSolver, IloObjective totalCost, IloRangeArray capacityConstraints,
	IloRangeArray demandConstraints, IloRangeArray2 EqualZConstraints, IloRangeArray2 EqualXijkConstraints,
	IloRangeArray inequalityCons, IloIntArray2 kijIndex, IloNumVarArray2 y, IloNumVarArray2 x, cmnd_t* instance);