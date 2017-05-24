#ifndef nodeListDefs
#define nodeListDefs


#include "LibrariesDefinitions.h"
#include "node.h"
// ---------------------------------------------------------------------------------------------------------- //
// Class:	nodeList (IloEnv * env)
//
// Purpose:	A nodeList object represents the dynamic branch and bound/price tree, with
//		nodes beeing removed and added as branching proceeds.
//
// Methods:	addNode() - create a node object and add to "top of tree" (stack list)
//		addToRmpCuts(...) - for a node in the tree (selected by specifying 'depth' argument),
//			add a cut to the associated rmpCuts object.
//		extractNode() - extract the top node (last added) from the tree, i.e. removing it
//			from the tree and returning associated node pointer to method caller.
//		getListSize() - returns size of problem list/tree (# of nodes in tree) to method caller.
//		getEnv() - returns optimization enviroment to method caller.
// ---------------------------------------------------------------------------------------------------------- //
class nodeList {

private:

	// pointer to node in top of list
	// (most recently added)
	node * topNode;

	// also a pointer to the node last
	// extracted from the node list.
	node * activeNode;

	// pointer to optimization envirnment
	IloEnv * pEnv;

	// keep a counter to total number of nodes added
	// (NOTE: this will never decrease, even if node
	// are removed, as we count only added nodes)
	// 加入树的节点数量
	IloInt nrOfNodesAdded;

	// also keep a counter of the tree size
	int listSize;

public:

	// ---------------------- CONSTRUCTOR ----------------------       
	nodeList(IloEnv *);
	// ---------------------------------------------------------

	// ---------------- METHOD: addNode() ----------------------
	// create a node object and add it to the list/tree.
	// 添加节点
	void addNode(IloNum, IloInt, IloNum);
	// parentNodeLB, parent index, branchVarVal

	// ---------------------------------------------------------

	// ---------------- METHOD: addToZCuts() -----------------
	// add information about a new cut on the form z[m][t] = 0/1
	// in top node (default) or some node deeper down (depth).
	// note that it allows to to enter a depth parameter, telling us
	// how far down in the node list to go 
	// (0 : we use topNode, 1 : we use topNode->nodeLink, etc)
	void addToZCuts(IloInt, IloInt, IloInt, int);
	// ---------------------------------------------------------

	// ---------------- METHOD: extractNode() -----------------
	// extract top node from node list/tree and return to 
	// method caller, while removing same node from node tree.
	node * extractTopNode();
	// ---------------------------------------------------------

	// ---------------- METHOD: extractNode() -----------------
	// extract node in tree with lowest lower bound, and return to 
	// method caller, while removing same node from node tree.
	// 提取全树下界的节点
	node * extractLowestLowerBoundNode();
	// ---------------------------------------------------------

	IloNum getLowerBound();

	// ---------------- METHOD: getListSize() ------------------
	// return size of problem list/tree to method caller
	int getListSize();
	// ---------------------------------------------------------

	// ------------- METHOD: getNrOfNodesAdded() ----------------
	// return getNrOfNodesAdded to tree (total)
	IloInt getNrOfNodesAdded();
	// ---------------------------------------------------------

	// ---------------- METHOD: getEnv() -----------------------
	// return optimization environment to method caller
	IloEnv * getEnv();
	// ---------------------------------------------------------

	// no destructor for now..
	// ~nodeList() { ...

};

#endif