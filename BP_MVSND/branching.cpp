// ------------------------------------------------------------------------------------
// Functions associated with the branching procedure (branch and bound/price).
// ------------------------------------------------------------------------------------

// project source files common headers
#include "node.h"
#include "nodeList.h"
#include "LibrariesDefinitions.h"
#include "functionPrototypes.h"

// ---------------------------------------------------------------------------------------------------------- //
// Function:	findFractionalZ(...)
//
// Purpose:	Find the index "coordinate" (m,t) of a fractional z[m][t] variables.
// ---------------------------------------------------------------------------------------------------------- //

// BELOW: Branch on "most fractional". 
// 返回branchVarVal
IloNum findFractionalZ(IloCplex rmpSolver, IloNumArray2 y, IloIntArray indexOut) {

	// extract enviroment
	IloEnv env = rmpSolver.getEnv();

	// variable declarations
	IloInt i, m, mBest, tBest;
	// loop counters...
	// 标记是否找到分量
	IloBool fractionalZFound, mostFractionalZFound;
	// (ilo)boolean variables to see if we find any fractional lambda/z.

	IloNum mostFractional;
	// temporary store value of z vars.

	IloNum yChoosen = -1;
	// temporary store values of RMP (LP) optimal lambdas.	

	// variable initializations
	fractionalZFound = mostFractionalZFound = IloFalse;
	// assume are yet to find fractional variables..

	mostFractional = 2.0;

	// search for fractional lambda (break as soon as we find one)
	// 找为分数的变量作为分枝对象
	// 读取RMP求解后变量结果
	for (m = 0; m < y.getSize(); m++) {

		for (i = 0; i < y[m].getSize(); i++) {
			// 获得分数部分的值
			IloNum fractionalValue = y[m][i] - IloFloor(y[m][i]);

			// fractional ?
			if ((fractionalValue - EPS > 0) && (fractionalValue + EPS < 1)) {

				// "best possible" (in the sense of most fractional) z found?
				// NEW: BEST POSSIBLE = IN THE INTERVAL +- MOST_FRACTIONAL_TOL
				// 找到最合适的分枝变量
				if (abs(fractionalValue - MOST_FRACTIONAL) + EPS < MOST_FRACTIONAL_TOL)
				{
					mostFractionalZFound = IloTrue;
					fractionalZFound = IloTrue;
					mostFractional = fractionalValue;
					mBest = m;
					tBest = i;
					yChoosen = y[m][i];
					break;	// break t loop
				}
				// "just" fractional z for this t?
				else if ((fractionalValue - EPS > 0) && (fractionalValue + EPS < 1)) {

					// see if it's "more fractional" than previously found z
					if (abs(fractionalValue - MOST_FRACTIONAL_SECONDARY) < abs(mostFractional - MOST_FRACTIONAL_SECONDARY))
					{
						fractionalZFound = IloTrue;
						mostFractional = fractionalValue;
						mBest = m;
						tBest = i;
						yChoosen = y[m][i];
					}

				}


			} // end of lambda if statement

			// if we have found best fractional, break i loop
			if (mostFractionalZFound) {
				break;
			}

		} // end of i loop

	}

	// 选的是z分数部分最小的
	if (mostFractionalZFound) {
		cout << endl << "Found most fractional z: z = " << mostFractional << endl;
	}
	else if (fractionalZFound) {
		cout << endl << "Found some fractional z: 0 < z < 0.5 OR 0.5 < z < 1. (m=" << mBest << ", t=" << tBest << ")." << endl;
		cout << endl << "Most fractional = " << mostFractional << endl;
	}
	else {
		cout << endl << "Found no fractional z: solution integer-valued. " << endl;
	}

	// if we found fractional z, return it's "index position", otherwise, return default 
	// index (NR_OF_MODULES, TIME_SPAN) telling caller we have an integer valued solution.
	// 返回找寻分数解的结果，如果没有则找到了整数可行解
	if (fractionalZFound) {
		indexOut[0] = mBest;
		indexOut[1] = tBest;
	}
	// if no fractional z was found, return default (telling caller solution is integer-valued).
	else {
		//for (int i = 0; i < y.getSize(); i++)
		//{
		//	for (int j = 0; j < y[i].getSize(); j++)
		//		cout << y[i][j] << " ";
		//	cout << endl;
		//}

		indexOut[0] = -1;
		indexOut[1] = -1;
	}

	// the result returns implicetely via income pointer indexOut.
	// 返回y（分数）的值
	cout << "Fractional y: " << yChoosen << endl;
	return yChoosen;

}

IloNum findFractionalZType(IloCplex rmpSolver, IloNumArray zType, IloIntArray indexOut) {

	// extract enviroment
	IloEnv env = rmpSolver.getEnv();

	// variable declarations
	IloInt i, mBest, tBest;
	// loop counters...
	// 标记是否找到分量
	IloBool fractionalZFound, mostFractionalZFound;
	// (ilo)boolean variables to see if we find any fractional lambda/z.

	IloNum mostFractional;
	// temporary store value of z vars.

	IloNum yChoosen = -1;
	// temporary store values of RMP (LP) optimal lambdas.	

	// variable initializations
	fractionalZFound = mostFractionalZFound = IloFalse;
	// assume are yet to find fractional variables..

	mostFractional = 2.0;

	// search for fractional lambda (break as soon as we find one)
	// 找为分数的变量作为分枝对象
	// 读取RMP求解后变量结果

	for (i = 0; i < zType.getSize(); i++) {
		// 获得分数部分的值
		IloNum fractionalValue = zType[i] - IloFloor(zType[i]);

		// fractional ?
		if ((fractionalValue - EPS > 0) && (fractionalValue + EPS < 1)) {

			// "best possible" (in the sense of most fractional) z found?
			// NEW: BEST POSSIBLE = IN THE INTERVAL +- MOST_FRACTIONAL_TOL
			// 找到最合适的分枝变量
			if (abs(fractionalValue - MOST_FRACTIONAL) + EPS < MOST_FRACTIONAL_TOL)
			{
				mostFractionalZFound = IloTrue;
				fractionalZFound = IloTrue;
				mostFractional = fractionalValue;
				mBest = -1;
				tBest = i;
				yChoosen = zType[i];
				break;	// break t loop
			}
			// "just" fractional z for this t?
			else if ((fractionalValue - EPS > 0) && (fractionalValue + EPS < 1)) {

				// see if it's "more fractional" than previously found z
				if (abs(fractionalValue - MOST_FRACTIONAL_SECONDARY) < abs(mostFractional - MOST_FRACTIONAL_SECONDARY))
				{
					fractionalZFound = IloTrue;
					mostFractional = fractionalValue;
					mBest = -1;
					tBest = i;
					yChoosen = zType[i];
				}

			}


		} // end of lambda if statement

		// if we have found best fractional, break i loop
		if (mostFractionalZFound) {
			break;
		}

	} // end of i loop


	// 选的是z分数部分最小的
	if (mostFractionalZFound) {
		cout << endl << "Found most fractional zType: zType = " << mostFractional << endl;
	}
	else if (fractionalZFound) {
		cout << endl << "Found some fractional zType: 0 < zType < 0.5 OR 0.5 < zType < 1. (type=" << tBest << ")." << endl;
		cout << endl << "Most fractional = " << mostFractional << endl;
	}
	else {
		cout << endl << "Found no fractional zType: solution integer-valued. " << endl;
	}

	// 返回找寻分数解的结果，如果没有则找到了整数可行解
	if (fractionalZFound) {
		indexOut[0] = mBest;
		indexOut[1] = tBest;
	}
	// if no fractional z was found, return default (telling caller solution is integer-valued).
	else {
		//for (int i = 0; i < y.getSize(); i++)
		//{
		//	for (int j = 0; j < y[i].getSize(); j++)
		//		cout << y[i][j] << " ";
		//	cout << endl;
		//}

		indexOut[0] = -1;
		indexOut[1] = -1;
	}

	// the result returns implicetely via income pointer indexOut.
	// 返回y（分数）的值
	cout << "Fractional zType: " << yChoosen << endl;
	return yChoosen;

}

// ---------------------------------------------------------------------------------------------------------- //
// Function:	initializeProblemList(...)
//
// Purpose:	Initializes problem list by adding the original rmp problem to the list, as
//		a redundant constraint/cut that does not effect the original problem.
// ---------------------------------------------------------------------------------------------------------- //

// ### static void initializeProblemList(IloRangeArray2 problemList)  {
// 初始化
void initializeProblemList(nodeList * tree, IloNum lowerBound) {

	// allocate space in the problem list for the initial problem/node.
	// ### problemList.add(IloRangeArray(env));
	// 加入根节点
	(*tree).addNode(lowerBound, 0, -1);

} // end of initializeProblemList()


// ---------------------------------------------------------------------------------------------------------- //
// Function:	branchActiveProblem(...)
//
// Purpose:	Given the Active Problem (top of problem list), creates two sub-problems using cuts
//		on some fractional z[m][t] variable, and adds to problem list.
//		Note: This function does not remove the active problem from list, this must be done
//		explicitely by funciton caller.
//
// Usage:	branchActiveProblem(problemList, maintConEng[m][t])
// ---------------------------------------------------------------------------------------------------------- //

// 构造两个子问题
// ### static void branchActiveProblem(IloRangeArray2 problemList, IloRange maintConEng, IloInt activeProblem) {
IloBool branchActiveProblem(nodeList * tree, node * activeNode,
	IloInt mCoord, IloInt tCoord, IloNum parentNodeLB, IloNum branchVarVal,
	IloBool branchNormal, ofstream &treeOutput) {

	IloInt i;

	// 如果z的值小于0.1则固定其为0（只分等于0的枝）
	//if (branchVarVal < 0.3)
	//{
	//	// 把分枝的子节点加到list中
	//	(*tree).addNode(parentNodeLB, activeNode->nodeNumber, branchVarVal);
	//	treeOutput << activeNode->nodeNumber << "->" << (*tree).getNrOfNodesAdded() << ";" << endl;
	//	treeOutput << (*tree).getNrOfNodesAdded() << " [color = \"blue\"];" << endl;

	//	for (i = 0; i < (activeNode->yCuts)[0].getSize(); i++) {
	//		// also copy the cut information on form z[m][t] = 0/1 used in subproblem cuts
	//		(*tree).addToZCuts((activeNode->yCuts)[0][i], (activeNode->yCuts)[1][i], (activeNode->yCutValues)[i], 0);
	//	}
	//	IloInt yInteger = IloCeil(branchVarVal);
	//	// 用正负标记分枝方向（[0,yInteger] 正 or [yInteger+1,infinity]负
	//	if (branchNormal) 
	//		(*tree).addToZCuts(mCoord, tCoord, yInteger, 0);

	//	return false;
	//}

	// 把分枝的子节点加到list中
	(*tree).addNode(parentNodeLB, activeNode->nodeNumber, branchVarVal);

	treeOutput << activeNode->nodeNumber << "->" << (*tree).getNrOfNodesAdded() << ";" << endl;
	// new nodes are active in list, so blue:
	treeOutput << (*tree).getNrOfNodesAdded() << " [color = \"blue\"];" << endl;

	(*tree).addNode(parentNodeLB, activeNode->nodeNumber, branchVarVal);
	treeOutput << activeNode->nodeNumber << "->" << (*tree).getNrOfNodesAdded() << ";" << endl;
	treeOutput << (*tree).getNrOfNodesAdded() << " [color = \"blue\"];" << endl;



	// copy cuts/constraints from parent node
	// 从父节点出复制相关约束与割
	// ### for (i = 0; i < problemList[activeProblem].getSize(); i++) {
	for (i = 0; i < (activeNode->yCuts)[0].getSize(); i++) {
		// also copy the cut information on form z[m][t] = 0/1 used in subproblem cuts
		(*tree).addToZCuts((activeNode->yCuts)[0][i], (activeNode->yCuts)[1][i], (activeNode->yCutValues)[i], 1);
		(*tree).addToZCuts((activeNode->yCuts)[0][i], (activeNode->yCuts)[1][i], (activeNode->yCutValues)[i], 0);
	}

	// if branched on fractional variable in span MOST_FRACTIONAL +- ..TOL
	// stack nodes with "0" forcing on top (so will be picked next
	// However, if branched on the MOST_FRACTIONAL_SECONDARY,
	// stack "forced 1" on top.
	// 向上取整，这样可以处理yValue为0的情况
	IloInt yInteger = IloCeil(branchVarVal);

	// 用正负标记分枝方向（[0,yInteger] 正 or [yInteger+1,infinity]负）
	if (branchNormal) {
		// also, for same nodes, add information about the cuts
		// in the class variables zCuts and zCutValues
		(*tree).addToZCuts(mCoord, tCoord, yInteger, 1); // depth = 1
		(*tree).addToZCuts(mCoord, tCoord, -yInteger, 0); // depth = 0
	}
	// 暂时这部分无用
	else {
		(*tree).addToZCuts(mCoord, tCoord, yInteger, 0); // depth = 1
		(*tree).addToZCuts(mCoord, tCoord, -yInteger, 1); // depth = 0
	}

	return true;
} // END of branchActiveProblem()


// ---------------------------------------------------------------------------------------------------------- //
// Function:	printBranchingInfo(...)
//
// Purpose:	Print some info of branching, given branch tree and active node pointers.
// ---------------------------------------------------------------------------------------------------------- //
//void printBranchingInfo(nodeList * tree, node * activeNode, IloInt itNum,
//	clock_t start, time_t start2, ofstream &outFile) {
//
//	// counter
//	IloInt j;
//
//	// temp timers	
//	clock_t end;
//	time_t end2;
//
//	// get current time
//	end = clock();
//	end2 = time(NULL);
//
//	// print to user
//	cout << "===========================================================================================" << endl;
//	cout << left << setw(30) << " " << "DIAGNOSTICS" << endl;
//	cout << left << setw(29) << " " << "-------------" << endl << endl;
//
//	// print to file
//	outFile << "===========================================================================================" << endl;
//	outFile << left << setw(30) << " " << "DIAGNOSTICS" << endl;
//	outFile << left << setw(29) << " " << "-------------" << endl << endl;
//
//	// print to user
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- Current iteration:" << itNum << "." << endl;
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- Current size of node tree:" << (*tree).getListSize() << "." << endl << endl;
//
//	// print to file
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- Current iteration:" << itNum << "." << endl;
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- Current size of node tree:" << (*tree).getListSize() << "." << endl << endl;
//
//	// print to user
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- Time passed since start of branching:" << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl << endl;
////	cout << left << setw(5) << " " << setw(45) 
////		<< "- With alternative timing method:" << end2-start2 << "sec." << endl << endl;
//
//	// print to file
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- Time passed since start of branching:" << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl << endl;
////	outFile << left << setw(5) << " " << setw(45) 
////		<< "- With alternative timing method:" << end2-start2 << "sec." << endl << endl;
//
//	// print to user
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- Total memory usage: " << ((*((*tree).getEnv())).getMemoryUsage())/MEGA << " MB." << endl << endl;
//	
//	// print to file
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- Total memory usage: " << ((*((*tree).getEnv())).getMemoryUsage())/MEGA << "MB." << endl << endl;
//	
//	// print to user
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- Number of cuts in active node:" << (activeNode->zCuts)[0].getSize() << "." << endl;
//	cout << left << setw(5) << " " << setw(45) 
//		<< "- All cuts in active node: " << endl;
//
//	// print to file
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- Number of cuts in active node:" << (activeNode->zCuts)[0].getSize() << "." << endl;
//	outFile << left << setw(5) << " " << setw(45) 
//		<< "- All cuts in active node: " << endl;
//
//	for (j = 0; j < (activeNode->zCuts)[0].getSize(); j++) {
//		// print to user
//		cout << left << setw(10) << " "
//			<< "Cut #" << j << " : (m = " << (activeNode->zCuts)[0][j] 
//			<< ", t = " << (activeNode->zCuts)[1][j] << ")" << setw(17) << " " << "z[" 
//			<< (activeNode->zCuts)[0][j] << "][" 
//			<< (activeNode->zCuts)[1][j] << "] = "
//			<< (activeNode->zCutValues)[j] << "." << endl;
//
//		// print to file
//		outFile << left << setw(10) << " "
//			<< "Cut #" << j << " : (m = " << (activeNode->zCuts)[0][j] 
//			<< ", t = " << (activeNode->zCuts)[1][j] << ")" << setw(17) << " " << "z[" 
//			<< (activeNode->zCuts)[0][j] << "][" 
//			<< (activeNode->zCuts)[1][j] << "] = "
//			<< (activeNode->zCutValues)[j] << "." << endl;
//	}
//
//	// print to user
//	cout << endl << "===========================================================================================" << endl;
//
//	// print to file
//	outFile << endl << "===========================================================================================" << endl;
//
//}
