#include "node.h"

// ---------------------------------------------------------------------------------------------------------- //
// Methods for class:	node (IloEnv * pEnv)
// ---------------------------------------------------------------------------------------------------------- //


// ---------------------- CONSTRUCTOR ----------------------  
// (initializer list for any node data using env)
node::node(IloEnv * pEnv, IloNum parentLB, IloNum branchVarVal) : yCuts(*pEnv, 2), yCutValues(*pEnv)
{
	// generate the "sub-dimension" arrays for the 2-d array 
	// zCuts. That is, the 2 columns |m|t|
	for (i = 0; i < 2; i++) {
		yCuts[i] = IloIntArray(*pEnv);
	}

	// set lower bound, given from solving parent node
	lowerBound = parentLB;

	// 这里的branchVarVal指的是小数部分
	if (branchVarVal > -0.5) {
		fracVal = branchVarVal;
	}
	// just for root node: we have no "parent fractional variable"..
	// 根节点传入的参数branchVarVal为-1
	else {
		fracVal = -1;
	}

}
// ---------------------------------------------------------

// ---------------- METHOD: extractSubCuts() ----------------------
// given a pointer to an IloRangeArray2 object, extract the cuts
// described in class variabe zCuts (on form "m, t, 0/1") and transforms
// into valid Concert constraint form, by adding the corresponding
// constraints to the IloRangeArray2 object beeing pointed to by first
// input argument.
void node::extractSubCuts(IloRangeArray * tempCon, IloNumVarArray2 y) {

	// extract all cuts in zCuts
	for (i = 0; i < yCuts[0].getSize(); i++) {
		if (yCutValues[i] > 0)
			(*tempCon).add(0 <= y[yCuts[0][i]][yCuts[1][i]] <= yCutValues[i] - 1);
		else
			(*tempCon).add(-yCutValues[i] <= y[yCuts[0][i]][yCuts[1][i]]);

		// for tempCon[m] : add cut (0/1 <= z[m][t] <= 0/1) (i.e. z[m][t] = 0/1)
		//(*tempCon)[yCuts[0][i]].add(yCutValues[i] <= z[zCuts[0][i]][zCuts[1][i]] <= zCutValues[i]);
	}
}

void node::extractZTypeCuts(IloRangeArray * tempCon, IloNumVarArray2 y, IloNumVarArray zType) {

	// extract all cuts in zCuts
	
	for (i = 0; i < yCuts[0].getSize(); i++) 
	{
		// 判断加z还是zType
		// yCuts第二维为zType的index
		if (yCuts[0][i] == -1)
		{
			if (yCutValues[i] > 0)
				(*tempCon).add(0 <= zType[yCuts[1][i]] <= yCutValues[i] - 1);
			else
				(*tempCon).add(-yCutValues[i] <= zType[yCuts[1][i]]);
		}
		else
		{
			if (yCutValues[i] > 0)
				(*tempCon).add(0 <= y[yCuts[0][i]][yCuts[1][i]] <= yCutValues[i] - 1);
			else
				(*tempCon).add(-yCutValues[i] <= y[yCuts[0][i]][yCuts[1][i]]);
		}


	}
}
// ---------------------------------------------------------

// ---------------- METHOD: extractRmpCuts() ----------------------	
// given z[m][t] = 0/1 cuts, return constraint working with the
// corresponding lambda variables (forced to 0 or 1).
// Why this method instead of keeping a constraint IloRangeArray
// object in each node? With this, the node information on cuts
// are general, and not locked to specific lambda variables. Hence,
// we can REMOVE COLUMNS without affected the correctness of node 
// cut information.

//void node::extractRmpCuts(IloRangeArray * tempCon, IloRangeArray2 * maintConEng) {
//
//	// optimization enviroment					
//	IloEnv env = (*tempCon).getEnv();
//
//	// variable declarations (local)
//	IloInt j;
//	// another loop counter..
//
//	IloExpr::LinearIterator it;
//	// linear iterator to extract non-zero lambda coefficients from
//	// expression in constraint maintConEng[m][t]
//
//	IloExpr tempExpr(env), tempExpr2(env);
//	// temporary expression
//
//	tempExpr.clear();
//	tempExpr2.clear();
//
//	//cout << endl << "we are here ------------------------------------!!!! " << endl;
//	//cout << endl << "and size of zCuts[0].getSize() = " << zCuts[0].getSize() << endl;
//
//	// extract all cuts in zCuts
//	for (i = 0; i < zCuts[0].getSize(); i++) {
//
//		// now extract all lambdas which have non-zero coefficients in constraint matrix
//		// of row index "coordinate" (m,t)
//
//		// get expression from input constraint with index [m][t]:
//		tempExpr = ((*maintConEng)[zCuts[0][i]][zCuts[1][i]]).getExpr();
//
//		// get linear iterator object for the expression extracted from mainContEng
//		it = tempExpr.getLinearIterator();
//
//		// skip the first member of the expression as it referes to the w_t variable first added
//		// (we're only interested in coefficients for lambda vars).
//		++it;
//
//		// loop over non-zero lambdas
//		for (; it.ok(); ++it) {
//
//			//cout << endl << "And we use our linear iterator..." << endl;
//			//cin.get();
//
//
//			// extract all lambdas with non-zero coefficints to temp expression object
//			tempExpr2 += 1 * it.getVar();
//		}
//
//		// now add this as cuts in the input tempCon
//		(*tempCon).add(zCutValues[i] <= tempExpr2 <= zCutValues[i]);
//
//		// clear tempExpr and let loop repeat
//		tempExpr.end();
//		tempExpr2.clear();
//	}
//
//	// clear memory used for temporary expression
//	//tempExpr.end();
//	tempExpr2.end();
//}

// ---------------------- DESTRUCTOR ----------------------  
node::~node() {

	// end arrays containing node cuts
	// to free memory
	for (i = 0; i < yCuts.getSize(); i++) {
		yCuts[i].end();
	}
	yCuts.end();
	yCutValues.end();
}

