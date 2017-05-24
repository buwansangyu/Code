// ------------------------------------------------------------------------------------
// Function prototypes (just declarations).
// ------------------------------------------------------------------------------------

#ifndef funcProt
#define funcProt

IloNum sumArray(IloNumArray redCosts);

IloNum minMemberValue(IloNumArray redCosts);

IloNum relativeImprovement(IloNumArray recentObjValues);

// branching.cpp
IloNum findFractionalZ(IloCplex rmpSolver, IloNumArray2 y, IloIntArray indexOut);
IloNum findFractionalZType(IloCplex rmpSolver, IloNumArray zType, IloIntArray indexOut);
void initializeProblemList(nodeList * tree, IloNum lowerBound);

IloBool branchActiveProblem(nodeList * tree, node * activeNode,
	IloInt mCoord, IloInt tCoord,IloNum parentNodeLB, IloNum branchVarVal,
	IloBool branchNormal, ofstream &treeOutput);

void printBranchingInfo(nodeList * tree, node * activeNode, IloInt itNum,
	clock_t start, time_t start2, ofstream &outFile);

// pseudoCostBranching.cpp
IloNum findBestCandidate(IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown,
	IloIntArray2 branchingCandidates, IloNumArray candidateValues,
	IloIntArray indexOut);

IloNum calculateScore(IloNum qM, IloNum qP);

IloNum calculateScoreAlt2(IloNum qM, IloNum qP);

void updateSingleNodePseudoCost(IloNum activeRmpVal, node * activeNode,
 	IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
	IloIntArray2 nuUp, IloIntArray2 nuDown,
	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown);

void calculatePseudoCosts(IloCplex rmpSolver, IloModel rmpModel, 
	IloObjective rmpObj, IloRangeArray2 maintConEng,
	IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
	IloIntArray2 nuUp, IloIntArray2 nuDown,
	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown,
	IloIntArray2 branchingCandidates, IloNumArray candidateValues);

void updateAllPseudoCosts (IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
	IloIntArray2 nuUp, IloIntArray2 nuDown,
	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown);

void transformIntoConstraint(IloRangeArray2 maintConEng, 
	IloRangeArray * tempRmpCut, 
	IloInt m, IloInt t, IloNum cutVal);

IloBool findBranchingCandidates(IloCplex rmpSolver, 
	IloRangeArray2 maintConEng, IloNumVarArray2 lambda, 
	IloIntArray2 branchingCandidates, IloNumArray candidateValues,
	IloBoolArray2 dodgeDuplicates);




#endif
