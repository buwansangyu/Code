#ifndef nodeDefs
#define nodeDefs

#include "LibrariesDefinitions.h"
// ---------------------------------------------------------------------------------------------------------- //
// Class:	node (IloEnv * pEnv)
//
// Purpose:	Objects represents nodes in the branch and bound/price node tree, where each
//		node contain information about active cuts as well as a link to previous node
//		in the tree (previously added).
// ---------------------------------------------------------------------------------------------------------- //
class node {
	// we might not actually need any private members...
private:

public:
	// --- node data ---
	// 对于0-1决策变量，固定变量取值
	// 对于整形变量，需要添加约束 
	// cuts on y[f][q] integer variable
	IloIntArray2 yCuts;
	IloNumArray yCutValues;

	// cuts on y[m][t] variables (forced to 0 or 1)
	//IloIntArray2 zCuts;
	//IloNumArray zCutValues;
	// explanation:	zCuts[0][i] : m value for i:th cut
	//		zCuts[1][i] : t value for i:th cut
	//		zCutValue[i] : cut value 0/1 for i:th cut

	// a counter
	IloInt i;

	// also a lower bound on optimal solution possible
	// for this node/branch
	// 节点下界
	IloNum lowerBound;

	// link to previous node (in list, not necessarily
	// parent node)
	// 前一个节点
	node * nodeLink;

	// also note the "index number" of the parent node
	// 父节点编号
	IloInt parent;

	// make not which node# this is (1 = root, 2, 3 = root's children, etc)
	// 节点编号属性
	IloInt nodeNumber;

	// make note the value of the branched variable (fractional)
	// 标记分枝变量
	IloNum fracVal;


	// ---------------------- CONSTRUCTOR ----------------------  
	// (initializer list for any node data using env)
	node(IloEnv *, IloNum, IloNum);
	//: zCuts(*pEnv, 2), zCutValues(*pEnv)

	// ---------------------------------------------------------

	// ---------------- METHOD: extractSubCuts() ----------------------
	// given a pointer to an IloRangeArray2 object, extract the cuts
	// described in class variabe zCuts (on form "m, t, 0/1") and transforms
	// into valid Concert constraint form, by adding the corresponding
	// constraints to the IloRangeArray2 object beeing pointed to by first
	// input argument.
	void extractSubCuts(IloRangeArray *, IloNumVarArray2);
	void extractZTypeCuts(IloRangeArray * tempCon, IloNumVarArray2, IloNumVarArray);
	// ---------------------------------------------------------

	// ---------------- METHOD: extractRmpCuts() ----------------------	
	// given z[m][t] = 0/1 cuts, return constraint working with the
	// corresponding lambda variables (forced to 0 or 1).
	// Why this method instead of keeping a constraint IloRangeArray
	// object in each node? With this, the node information on cuts
	// are general, and not locked to specific lambda variables. Hence,
	// we can REMOVE COLUMNS without affected the correctness of node 
	// cut information.
	//void extractRmpCuts(IloRangeArray *, IloRangeArray2 *);

	// ---------------------- DESTRUCTOR ----------------------  
	~node();


};

#endif