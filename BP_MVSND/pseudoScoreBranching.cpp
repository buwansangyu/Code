//// ------------------------------------------------------------------------------------
//// Functions associated with the pseudocosts used when picking branching variables
//// ------------------------------------------------------------------------------------
//
//// project source files common headers
//#include "node.h"
//#include "nodeList.h"
//#include "LibrariesDefinitions.h"
//#include "functionPrototypes.h"
//
//// FUNCTION: findBestCandidate(..)
//// given pseudocosts, branching candidates and candidate fractional values,
//// return (implicitely via input vector) the coordinate of the candidate
//// with the highest score
//IloNum findBestCandidate(IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown,
//	IloIntArray2 branchingCandidates, IloNumArray candidateValues,
//	IloIntArray indexOut) { 
//
//	// counters
//	IloInt i, m, t;
//
//	// scores
//	IloNum score, bestScore, candidateVal;
//	score = bestScore = candidateVal = 0;
//
////	cout << endl << "Size of candidateValue = " << candidateValues.getSize() << endl;
////	cin.get();
//
//	// calculate score for each candidate, and as soon as "bestScore" is
//	// improved, update implicit output vector with associated variable coordinates.
//	for (i = 0; i < candidateValues.getSize(); i++) {
//
//		// extract cut coordinates
//		m = branchingCandidates[0][i];
//		t = branchingCandidates[1][i];
//		
//		// calculate score
//		score = calculateScore(candidateValues[i] * pseudoCostsDown[m][t],
//			(1 - candidateValues[i]) * pseudoCostsUp[m][t]);
//
//		//score *= calculateScore(candidateValues[i] * pseudoCostsDown[m][t],
//		//	(1 - candidateValues[i]) * pseudoCostsUp[m][t]);
//
//		// some output.. while testing
////		cout << endl << "For branching candidate [m=" << m << ", t=" << t << "]," 
////			<< endl << "with fractional value " << candidateValues[i] << ", the"
////			<< endl << "associated score = " << score << endl;
////		cout << "(best score = " << bestScore << " for [m=" << indexOut[0] << ", t=" << indexOut[1] << "]." << endl << endl;
//		//cin.get();
//
//		// see if we have a new best score
//		if (score > bestScore) {
//
//			// if so, update best score
//			bestScore = score;
//		
//			// and update associated "best variable coordinates"
//			indexOut[0] = m;
//			indexOut[1] = t;
//
//			// also extract fractional value of this candidate
//			candidateVal = candidateValues[i];
//		}
//	}
//
//	// variable with best scoore is return implicitly via input
//	// argument vetcor indexOut
//
//	// return value of variable chosen
//	return candidateVal;
//
//} // end of findBestCandidate()
//
//// FUNCTION: calculate score as:
//// score(qM,qP) := (1 - SCORE_WEIGHT) * min{qM, qP} + SCORE_WEIGHT * max{qM, qP}
//IloNum calculateScore(IloNum qM, IloNum qP) {
//
//	// score
//	IloNum score;
//
//	// calculate score
//	if (qM < qP + EPS) {
//		score = (1 - SCORE_WEIGHT) * qM + SCORE_WEIGHT * qP;
//	}
//	else {
//		score = (1 - SCORE_WEIGHT) * qP + SCORE_WEIGHT * qM;
//	}
//
//	// return score	
//	return score;
//
//} // end of calculateScore
//
//// score(qM,qP) := (1 - SCORE_WEIGHT) * min{qM, qP} + SCORE_WEIGHT * max{qM, qP}
//IloNum calculateScoreAlt2(IloNum qM, IloNum qP) {
//
//	// score
//	IloNum score;
//
//	// calculate score, alternative: score(qM, qP) = max(qM, EPS) * max(qP, EPS), EPS = 1.0e-6
//	if (qM > EPS) {
//		score = qM;
//	}
//	else {
//		score = EPS;
//	}
//
//	if (qP > EPS) {
//		score *= qP;
//	}
//	else {
//		score *= EPS;
//	}
//
//	// return score	
//	return score;
//
//} // end of calculateScore
//
//// FUNCTION: updateSingleNodePseudoCost(...)
//void updateSingleNodePseudoCost(IloNum activeRmpVal, node * activeNode,
// 	IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
//	IloIntArray2 nuUp, IloIntArray2 nuDown,
//	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown) {
//
//	// length of cut vector
//	IloInt nrOfCuts;
//
//	// node coordinates
//	IloInt m, t;
//
//	// value of cut (0 or 1?)
//	IloNum cutValue;
//
//	// objective gain and objective gain per unit change temp var
//	IloNum deltaTemp, objGainTemp;
//
//	// initialization
//	nrOfCuts = (activeNode->zCutValues).getSize();
//	m = (activeNode->zCuts)[0][nrOfCuts-1];
//	t = (activeNode->zCuts)[1][nrOfCuts-1];
//	cutValue = (activeNode->zCutValues)[nrOfCuts-1];
//
//	// UP cut (+)
//	if (cutValue > 0.5) {
//
//		// calculate deltaUp[m][t]
//		deltaTemp = activeRmpVal - (activeNode->lowerBound);
//
//		// and deduce objective gain per unit change
//		objGainTemp = deltaTemp / (1.0 - (activeNode->fracVal));
//
//		// and update sigmaUp[][] vector for [m] [t] entry
//		sigmaUp[m][t] += objGainTemp;
//
//		// also increase nuUp entry
//		nuUp[m][t]++;
//
//		// and finally, compute the new pseudocost
//		pseudoCostsUp[m][t] = sigmaUp[m][t] / nuUp[m][t];
//	}
//	// else: DOWN cut (-)
//	else {
//		
//		// calculate deltaDown[m][t]
//		deltaTemp = activeRmpVal - (activeNode->lowerBound);
//
//		// and deduce objective gain per unit change
//		objGainTemp = deltaTemp / (activeNode->fracVal);
//
//		// and update sigmaDown[][] vector for [m] [t] entry
//		sigmaDown[m][t] += objGainTemp;
//
//		// also increase nuDown entry
//		nuDown[m][t]++;
//
//		// and finally, compute the new pseudocost
//		pseudoCostsDown[m][t] = sigmaDown[m][t] / nuDown[m][t];
//	}
//
//	// no explicit return!
//
//} // END of updateSingleNodePseudoCost
//
//// FUNCTION: calculatePseudoCosts(...)
//void calculatePseudoCosts(IloCplex rmpSolver, IloModel rmpModel, 
//	IloObjective rmpObj, IloRangeArray2 maintConEng,
//	IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
//	IloIntArray2 nuUp, IloIntArray2 nuDown,
//	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown,
//	IloIntArray2 branchingCandidates, IloNumArray candidateValues) {
//
//	// extract enviroment
//	IloEnv env = rmpSolver.getEnv();
//
//	// counters
//	IloInt i, m, t;
//
//	// obj. value of active node
//	IloNum objValOrig = rmpSolver.getValue(rmpObj);
//	
//	// tem obj value, temp obj gain
//	IloNum objValTemp, objGainTemp;
//
//	// up (+) or down (-) cuts?
//	IloNum cutUp, cutDown;
//	cutUp = 1.0;
//	cutDown = 0.0;
//
//	// temp range array
//	IloRangeArray tempRmpCut(env);
//
//	// for each of the candidates, if any of their Up or Down subproblems
//	// have no previously been solved to feasibility (e.g. if nu(candidate) = 0),
//	// do so here
//	for (i = 0; i < candidateValues.getSize(); i++) {
//	
//		// just in case, clear tempRmpCut
//		tempRmpCut.clear();
//
//		// extract cut coordinates
//		m = branchingCandidates[0][i];
//		t = branchingCandidates[1][i];
//
//		// UP cut (+)
//		// have the problem been solved previously?
//		if (nuUp[m][t] < NU_LIMIT - 0.5) {
//			
//			// if so, extract constraint
//			transformIntoConstraint(maintConEng, &tempRmpCut, 
//				m, t, cutUp);
//		
//			// add cuts to rmp
//			rmpModel.add(tempRmpCut);
//
//			// solve rmp (LP relaxation)
//      			if ( !rmpSolver.solve() ) {
//        			env.error() << "Failed to optimize RMP." << endl;
//				cout << "INFEASIBLE BRANCH IN PSEUDOCOST COMPUTATION?!?!" << endl;				
//				cin.get();
//        			throw(-1);
//      			}
//
//			// extract obj. function value
//			objValTemp = rmpSolver.getValue(rmpObj);
//
//			// calc objective gain per unit change
//			objGainTemp = (objValTemp - objValOrig)/(cutUp - candidateValues[i]);
//
//			// add to sum (sigma) over all problems Q where [m][t] has been selected
//			// as a branching variable and Q+[m][t] hass been solved and was feasible.
//			sigmaUp[m][t] += objGainTemp;
//
//			// also increase the counter of these occurences
//			nuUp[m][t]++;
//
//			// remove cuts from rmp:
//			rmpModel.remove(tempRmpCut);
//
//			// clear tempRmpCuts
//			//tempRmpCut.clear();
//			tempRmpCut.clear();
//			// note!! you might have to end here
//		}
//
//		// DOWN cut (-)
//		// have the problem been solved previously?
//		if (nuDown[m][t] < NU_LIMIT - 0.5) {
//			
//			// if so, extract constraint
//			transformIntoConstraint(maintConEng, &tempRmpCut, 
//				m, t, cutDown);
//		
//			// add cuts to rmp
//			rmpModel.add(tempRmpCut);
//
//			// solve rmp (LP relaxation)
//      			if ( !rmpSolver.solve() ) {
//        			env.error() << "Failed to optimize RMP." << endl;
//				cout << "INFEASIBLE BRANCH IN PSEUDOCOST COMPUTATION?!?!" << endl;				
//				cin.get();
//        			throw(-1);
//      			}
//
//			// extract obj. function value
//			objValTemp = rmpSolver.getValue(rmpObj);
//
//			// calc objective gain per unit change
//			objGainTemp = (objValTemp - objValOrig)/(candidateValues[i]);
//
//			// add to sum (sigma) over all problems Q where [m][t] has been selected
//			// as a branching variable and Q+[m][t] hass been solved and was feasible.
//			sigmaDown[m][t] += objGainTemp;
//
//			// also increase the counter of these occurences
//			nuDown[m][t]++;
//
//			// remove cuts from rmp:
//			rmpModel.remove(tempRmpCut);
//
//			// clear tempRmpCuts
//			tempRmpCut.clear();
//		}
//	}
//
//	// now update all pseudocosts
//	updateAllPseudoCosts (sigmaUp, sigmaDown, nuUp, nuDown,
//			pseudoCostsUp, pseudoCostsDown);
//	
//
//} // END of calculatePseudoCosts(..)
//
//	// We have now updated nu and sigma for all branching candidates which had
//	// not been handled before (nu = 0).
//
//	// Next step, for thursday:	in this function, calculate (i.e. update) pseudoCosts
//	// for all the branching candidates, and thereafter calculate average pseudocosts, and --- for all variables with nu = 0 ---
//	// update the pseudocosts for these with the avarage value.
//
//	// now, we move on to calculate/update all pseudoCosts, for ALL variables with nu>0, and
//	// from that, compute the updated avarage pseudocost and set for ALL variables with nu = 0;
//
//// FUNCTION: updateAllPseudoCosts
//void updateAllPseudoCosts (IloNumArray2 sigmaUp, IloNumArray2 sigmaDown,
//	IloIntArray2 nuUp, IloIntArray2 nuDown,
//	IloNumArray2 pseudoCostsUp, IloNumArray2 pseudoCostsDown) {
//
//	// counters
//	IloInt m, t;
//
//	// calculate sum of up and down pseudocosts	
//	IloNum pseudoCostsUpSum, pseudoCostsDownSum, pseudoCostsUpAverage, pseudoCostsDownAverage;
//	IloInt pseudoNumUp, pseudoNumDown;
//		// count number of members in each sum
//
//	// initialize to 0
//	pseudoCostsUpSum = pseudoCostsDownSum = 0.0;
//	pseudoNumUp = pseudoNumDown = 0;
//
//	// --- compute pseudocosts
//	// for all modules
//	for (m = 0; m < NR_OF_MODULES; m++) {
//		
//		// and all times..
//		for (t = 0; t < TIME_SPAN; t++) {
//
//			// nuUp (+)
//			if (nuUp[m][t] > 0.5) {
//
//				// compute pseudocost
//				pseudoCostsUp[m][t] = sigmaUp[m][t]/nuUp[m][t];
//			
//				// add to sum and increase counter
//				pseudoCostsUpSum += pseudoCostsUp[m][t];
//				pseudoNumUp++;
//			}
//
//			// nuDown (-)
//			if (nuDown[m][t] > 0.5) {
//
//				// compute pseudocost
//				pseudoCostsDown[m][t] = sigmaDown[m][t]/nuDown[m][t];
//
//				// add to sum and increase counter
//				pseudoCostsDownSum += pseudoCostsDown[m][t];
//				pseudoNumDown++;
//			}
//
//		} // end of time loop
//
//	} // end of module loop
//
//	// compute up and down avarage
//	pseudoCostsUpAverage = pseudoCostsUpSum/pseudoNumUp;
//	pseudoCostsDownAverage = pseudoCostsDownSum/pseudoNumDown;
//
//	// --- for all non-initialized pseudocosts, set avarage
//	// for all modules
//	for (m = 0; m < NR_OF_MODULES; m++) {
//		
//		// and all times..
//		for (t = 0; t < TIME_SPAN; t++) {
//
//			// nuUp (+)
//			if (nuUp[m][t] < 0.5) {
//
//				// set avarage pseudocost
//				pseudoCostsUp[m][t] = pseudoCostsUpAverage;
//
//			}
//
//			// nuDown (-)
//			if (nuDown[m][t] < 0.5) {
//
//				// set avarage pseudocost
//				pseudoCostsDown[m][t] = pseudoCostsDownAverage;
//			}
//
//		} // end of time loop
//
//	} // end of module loop
//
//	// next step is a score function, calculating the score given pseudocost and candidateValues.
//
//} // end of updateAllPseudoCosts()
//
//// HELP FUNCTION: [m][t] cut -> IloRangeArray constraint.
//void transformIntoConstraint(IloRangeArray2 maintConEng, 
//	IloRangeArray * tempRmpCut, 
//	IloInt m, IloInt t, IloNum cutVal) {
//	
//	// extract enviroment
//	IloEnv env = maintConEng.getEnv();
//
//	IloExpr::LinearIterator it;
//		// linear iterator to extract non-zero lambda coefficients from
//		// expression in constraint maintConEng[m][t]
//
//	IloExpr tempExpr(env), tempExpr2(env);
//		// temporary expressions
//
//	tempExpr.clear();
//	tempExpr2.clear();
//
//	// get expression from input constraint with index [m][t]:
//	tempExpr = (maintConEng[m][t]).getExpr();
//
//	// get linear iterator object for the expression extracted from mainContEng
//	it = tempExpr.getLinearIterator();
//
//	// skip the first member of the expression as it referes to the w_t variable first added
//	// (we're only interested in coefficients for lambda vars).
//	++it;
//
//	// loop over non-zero lambdas
//	for (; it.ok(); ++it) {
//			
//		// extract all lambdas with non-zero coefficints to temp expression object
//       		tempExpr2 += 1*it.getVar();
//	}						
//		
//	// now add this as cuts in the input tempRmpCut
//	(*tempRmpCut).add(cutVal <= tempExpr2 <= cutVal);
//
//	// clear tempExpr and let loop repeat
//	tempExpr.end();
//	tempExpr2.end();
//
//} // end of extract...
//
//
//// FUNCTION: findBranchingCandidates()
//IloBool findBranchingCandidates(IloCplex rmpSolver, 
//	IloRangeArray2 maintConEng, IloNumVarArray2 lambda, 
//	IloIntArray2 branchingCandidates, IloNumArray candidateValues,
//	IloBoolArray2 dodgeDuplicates) {
//	
//	// extract enviroment
//	IloEnv env = rmpSolver.getEnv();
//
//	// variable declarations
//	IloInt i, m, t, epsCount;
//		// loop counters...
//
//	IloBool fractionalZFound;
//		// (ilo)boolean variables to see if we find any fractional lambda/z.
//
//	IloNum zTempVal;
//		// temporary store value of z vars.
//
//	IloNumArray lambdaValues(env);
//		// temporary store values of RMP (LP) optimal lambdas.
//
//	IloExpr exprZ(env);
//		// temporary store expressions (rows) from constraint maintConEng.
//
//	IloExpr::LinearIterator it2;
//		// linear iterator to extract non-zero lambda coefficients from expression above.			
//
//	// variable initializations
//	fractionalZFound = IloFalse;
//		// assume are yet to find fractional variables..
//
//	// reset dodgeDuplicates array
//	for (m = 0; m < NR_OF_MODULES; m++) {
//		for (t = 0; t < TIME_SPAN; t++) {
//			dodgeDuplicates[m][t] = IloFalse;
//		}
//	}
//
//	// also, reset branchingCandidates and candidate values
//	branchingCandidates[0].clear();
//	branchingCandidates[1].clear();
//	candidateValues.clear();
//
//	// first search for fractionals within [EPS2 -> 1-EPS2], EPS1 = 0.01
//	// (secondly, do an actual (0, 1) search, IF none are found withing first)
//	for (epsCount = 0; epsCount < 2; epsCount++) {
//
//		// search through all fractional lambda
//		for (m = 0; m < NR_OF_MODULES; m++) {
//			rmpSolver.getValues(lambdaValues, lambda[m]);
//
//			//cout << endl << "memUsage = " << (env.getMemoryUsage())/MEGA << "MB, m = " << m << endl;
//			//cin.get();
//
//			for (i = 0; i < lambdaValues.getSize(); i++) {
//
//				//env.out() << endl << "Current lambda value (m=" << m << ", col=" << i << ") = " << lambdaValues[i] << endl;
//
//				//cout << endl << "memUsage = " << (env.getMemoryUsage())/MEGA << "MB, i = " << i << endl;
//
//				// fractional lambda?
//				if ((lambdaValues[i] - EPS > 0) && (lambdaValues[i] + EPS < 1)) {
//				
//					// so now we have a module (sub-columnpol) in which the optimal lambdas describe a fractional z value, for at least one value of t.
//					// we search for this value t (or first occurence of such a t)
//					for (t = 0; t < TIME_SPAN; t++) {
//
//						//cout << endl << "memUsage = " << (env.getMemoryUsage())/MEGA << "MB, t = " << t << endl;
//
//						// only proceed if this z[m][t] variables has not already been handled 
//						if (!dodgeDuplicates[m][t]) {
//
//							// make note that we handle this variable
//							dodgeDuplicates[m][t] = IloTrue;
//
//							// get expression from specific constraint:
//							exprZ = maintConEng[m][t].getExpr();
//
//							// get linear iterator object for the expression extracted from mainContEng
//							it2 = exprZ.getLinearIterator();
//
//							// skip the first member of the expression as it referes to the w_t variable first added
//							// (we're only interested in coefficients for lambda vars).
//							++it2;
//
//							// reset zTempVal
//							zTempVal = 0.0;
//
//		
//							// loop over non-zero lambdas
//							for (; it2.ok(); ++it2) {
//		
//								// accumulate the corresponding value of the z[m][t] variable described 
//								// by these lambdas (and associated columns)
//	       							zTempVal += rmpSolver.getValue(it2.getVar());
//							}
//
//		
//							// fractional z for this t?
//							// if epsCount = 0, do "rough search:
//							if (epsCount < 0.5) {
//								if ((zTempVal - EPS2 > 0) && (zTempVal + EPS2 < 1)) {
//
//									// add to input argument vector branchingCandidates
//									branchingCandidates[0].add(m);
//									branchingCandidates[1].add(t);
//									candidateValues.add(zTempVal);
//										// also add candidate value
//
//									// make note that we found a fractional z
//									fractionalZFound = IloTrue;
//								}
//							}
//							// otherwise, fine search							
//							else {
//								if ((zTempVal - EPS > 0) && (zTempVal + EPS < 1)) {
//
//									// add to input argument vector branchingCandidates
//									branchingCandidates[0].add(m);
//									branchingCandidates[1].add(t);
//									candidateValues.add(zTempVal);
//										// also add candidate value
//
//									// make note that we found a fractional z
//									fractionalZFound = IloTrue;
//								}
//
//							}		
//
//							exprZ.end();
//	
//						} // end of dodgeDuplicates if statement. 
//					
//					} // end of t loop
//
//				//exprZ.end();
//
//				//cout << endl << "!!!!memUsage = " << (env.getMemoryUsage())/MEGA << "MB!" << endl;
//				//cin.get();
//
//				} // end of lambda if statement
//	
//			} // end of i loop
//
//			lambdaValues.clear();
//
//		} // end of m loop
//
//		// if fractional lambda has been found, break, otherwise
//		// re-do search but with very small lambda
//		if (fractionalZFound) {
//			break;
//		}
//
//	} // end of eps loop
//
//	if (fractionalZFound) {
//		cout << endl << "Found some fractional z." << endl;
//	}
//	else {
//		cout << endl << "Found no fractional z: solution integer-valued. " << endl;
//	}													
//		
//	// clear memory allocated for temp var exprZ
//	//exprZ.end();
//
//	// clear memory allocated for IloNumArray lambdaValues
//	lambdaValues.end();
//
//	// return: did we find any fractional z's?
//	return fractionalZFound;
//
//} // end of findBranchingCandidates()
//
