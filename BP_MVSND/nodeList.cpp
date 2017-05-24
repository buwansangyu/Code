#include "nodeList.h"

// ---------------------------------------------------------------------------------------------------------- //
// Methods for class:	nodeList (IloEnv * env)
// ---------------------------------------------------------------------------------------------------------- //

// ---------------------- CONSTRUCTOR ----------------------       
nodeList::nodeList(IloEnv * env) {

	// initially, no nodes in list
	topNode = NULL;

	// and no active node
	activeNode = NULL;

	// set pointer value to opt. env.
	pEnv = env;

	// no nodes added
	nrOfNodesAdded = 0;

	// and list size initially 0
	listSize = 0;

}
// ---------------------------------------------------------

// ---------------- METHOD: addNode() ----------------------
// create a node object and add it to the list/tree.
void nodeList::addNode(IloNum lowerBound, IloInt parentIndex, IloNum branchVarVal) {

	node * newNode;
	newNode = new node(pEnv, lowerBound, branchVarVal);
	//tmp = new node;

	// increase counter ofnumber of nodes in tree
	// 总节点数
	nrOfNodesAdded++;

	// give the node it's node number/index
	newNode->nodeNumber = nrOfNodesAdded;

	// also the parent index
	// 新节点的父节点
	newNode->parent = parentIndex;


	// link to top node and add as new top node
	// 深度优先
	newNode->nodeLink = topNode;
	topNode = newNode;


	// increase list size
	listSize++;

} // END of method addNode()
// ---------------------------------------------------------

// ---------------- METHOD: addToZCuts() -----------------
// add information about a new cut on the form z[m][t] = 0/1
// in top node (default) or some node deeper down (depth).
// note that it allows to to enter a depth parameter, telling us
// how far down in the node list to go 
// (0 : we use topNode, 1 : we use topNode->nodeLink, etc)
void nodeList::addToZCuts(IloInt fCoord, IloInt tCoord, IloInt cutValue, int depth) {

	// check if there's any nodes in the tree
	if (topNode == NULL)
	{
		cout << "\nERROR: NODE TREE EMPTY\n";
		//return NULL;
	}

	// update activeNode as topNode
	// 深度优先
	activeNode = topNode;

	while (depth > 0) {

		if (activeNode->nodeLink == NULL)
			cout << endl << "depth =" << depth << endl;
		// 深度优先
		activeNode = activeNode->nodeLink;
		depth--;
	}

	// add "[f][q]" coordinate for cut
	activeNode->yCuts[0].add(fCoord);
	activeNode->yCuts[1].add(tCoord);

	// add cut value
	// 为分数的y值
	activeNode->yCutValues.add(cutValue);


} // END of method addToRmpCuts()
// ---------------------------------------------------------

// ---------------- METHOD: extractTopNode() -----------------
// extract top node from node list/tree and return to 
// method caller, while removing same node from node tree.
// 在完成剪枝操作后回溯到上一个记点位置
node * nodeList::extractTopNode() {

	// check if there's any nodes in the tree
	if (topNode == NULL)
	{
		cout << "\nERROR: NODE TREE EMPTY\n";
		cin.get();
		return NULL;
	}

	// if not, proceed.. (we should never get an
	// error above as the tree size is controlled
	// by function/class caller..)

	// extract top node pointer, this is now
	// active node
	// 深度优先
	activeNode = topNode;

	// 广度优先，回溯到第一节点
	for (int i = 1; i < listSize; i++)
		activeNode = activeNode->nodeLink;
	

	// update top node pointer
	// (i.e. remove top object in stack)
	// 深度优先
	//topNode = topNode->nodeLink;

	// decrease listSize
	listSize--;

	// return extracted/removed node
	return activeNode;

} // END of method extractTopNode()
// ---------------------------------------------------------

// ---------------- METHOD: extractLowestLowerBoundNode() -----------------
// extract node in tree with lowest lower bound, and return to 
// method caller, while removing same node from node tree.
// 提取最小下界的节点，并从树中移除
node * nodeList::extractLowestLowerBoundNode() {

	// check if there's any nodes in the tree
	if (topNode == NULL)
	{
		cout << "\nERROR: NODE TREE EMPTY\n";
		return NULL;
	}

	// if not, proceed.. (we should never get an
	// error above as the tree size is controlled
	// by function/class caller..)

	// some temporary node pointers
	node * lowestLowerBoundNode, *nextNode, *parentNode;

	// temporary IloNum to hold "lowest lower bound"
	IloNum lowestLB;

	// initializations
	lowestLowerBoundNode = topNode;
	// start at top

	nextNode = lowestLowerBoundNode->nodeLink;
	// next node: link from topNode

	parentNode = NULL;
	// NULL by default (in case topNode holds lowest LB: no parent)

	activeNode = topNode;
	// let activeNode temporary hold "parent node" information,
	// so at the moment, activeNode holds parent of nextNode.

	lowestLB = lowestLowerBoundNode->lowerBound;
	// currently lowest lower bound: from topNode

	// search through full list
	while (nextNode != NULL) {

		// see if "nextNode" has a lower lower bound than
		// the currently lowest
		if (nextNode->lowerBound < lowestLB) {

			// if so, update lowestLB and lowestLowerBoundNode pointer
			lowestLB = nextNode->lowerBound;
			lowestLowerBoundNode = nextNode;

			// also update parent node of lowestLowerBoundNode
			parentNode = activeNode;
		}

		// move to next node: i.e., parent becomes current node, current
		// node becomes next node
		activeNode = nextNode;	// parent of "node we are investigating"
		nextNode = nextNode->nodeLink; // next node in list

	} // continue until we reach nextNode = NULL: end of list.

	// now extract the node pointed to by lowestLowerBoundNode

	// if tempParentNode == NULL, lowestLowerBoundNode = topNode, and
	// we simply extract top node (else statement). Otherwise, extract
	// a node "in between nodes" and link together the "node gap".
	if (parentNode != NULL) {

		// link parent of "lowestLowerBoundNode" to child of "lowestLowerBoundNode".
		parentNode->nodeLink = lowestLowerBoundNode->nodeLink;
	}
	// if topNode, just update topnode info
	else {
		topNode = topNode->nodeLink;
	}

	// decrease listSize
	listSize--;

	// return extracted/removed node
	return lowestLowerBoundNode;

} // END of method extractLowestLowerBoundNode
// ---------------------------------------------------------

// ---------------- METHOD: getLowerBound() -----------------
// extract node in tree with lowest lower bound, and return to 
// method caller, while removing same node from node tree.
// 获得最小下界
IloNum nodeList::getLowerBound() {

	// check if there's any nodes in the tree
	if (topNode == NULL)
	{
		cout << "\nERROR: NODE TREE EMPTY\n";
		return 0;
	}

	// if not, proceed.. (we should never get an
	// error above as the tree size is controlled
	// by function/class caller..)

	// some temporary node pointers
	node * lowestLowerBoundNode, *nextNode;

	// temporary IloNum to hold "lowest lower bound"
	IloNum lowestLB;

	// initializations
	lowestLowerBoundNode = topNode;
	// start at top

	nextNode = lowestLowerBoundNode->nodeLink;
	// next node: link from topNode

	lowestLB = lowestLowerBoundNode->lowerBound;
	// currently lowest lower bound: from topNode

	// search through full list
	while (nextNode != NULL) {

		// see if "nextNode" has a lower lower bound than
		// the currently lowest
		if (nextNode->lowerBound < lowestLB) {

			// if so, update lowestLB and lowestLowerBoundNode pointer
			lowestLB = nextNode->lowerBound;

		}

		// move to next node: 
		nextNode = nextNode->nodeLink; // next node in list

	} // continue until we reach nextNode = NULL: end of list.

	// return LB
	return lowestLB;

} // END of method extractLowerBound
// ---------------------------------------------------------

// ---------------- METHOD: getListSize() ------------------
// return size of problem list/tree to method caller
int nodeList::getListSize() {
	return listSize;
}// END of method getListSize()
// ---------------------------------------------------------

// ---------------- METHOD: getNrOfNodesAdded() ------------------
// return size of problem list/tree to method caller
IloInt nodeList::getNrOfNodesAdded() {
	return nrOfNodesAdded;
}// END of method getListSize()
// ---------------------------------------------------------

// ---------------- METHOD: getEnv() -----------------------
// return optimization environment to method caller
IloEnv * nodeList::getEnv() {
	return pEnv;
}// END of method getEnv
// ---------------------------------------------------------

