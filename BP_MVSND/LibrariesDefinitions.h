#ifndef LibariesDefs
#define LibariesDefs
// libraries
#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstring>
#include <cmath>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

using namespace std;



// typedef array objects for 2-dim and 3-dim arrays.
typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloNumVarArray2> IloNumVarArray3;

typedef IloArray<IloBoolArray> IloBoolArray2;

typedef IloArray<IloModel> IloModel2;
typedef IloArray<IloCplex> IloCplex2;
typedef IloArray<IloObjective> IloObjective2;
typedef IloArray<IloRangeArray> IloRangeArray2;

//typedef IloArray<IloCplex::BasisStatusArray> BasisStatusArray2;



// constants
//#define TIME_SPAN 35
//#define NR_OF_MODULES 7
//#define NR_OF_ACTIVITIES 7

// misc. constants..
#define NEW_COL 1
#define ACTIVE_PROBLEM 0
#define MEGA 1.0e6

// "machine epsilon", to make sure comparisons don't fail
// due to machine rounding
#define EPS 1.0e-6

#define EPS2 0.005
// orig 1.0e-2

// cap in reduced cost (when no subproblems generate columns with RC < RC_EPS: break
#define RC_EPS 1.0e-5
// -5 BEST

// Key factor #1: breaking column generation when relative improvement (seen in history
// of last REL_CHANGE_SPAN colgen iterations) is smaller than REL_CHANGE
#define REL_CHANGE 1.0e-4
// -4 and 75 BEST
#define REL_CHANGE_SPAN 75
// best: 1.0e-4 and 10 for "normal branching"


// Key factor #2: Which fractional z[m][t] variable to choose:
// Choose first found in span MOST_FRACTIONAL +- MOST_FRACTIONAL_TOL
// If none is found in this span, pick the fractional variable closes to MOST_FRACTIONAL_SECONDARY.
#define MOST_FRACTIONAL 0.26
#define MOST_FRACTIONAL_TOL 0.25999
#define MOST_FRACTIONAL_SECONDARY 0.999

//#define MOST_FRACTIONAL 0.5
//#define MOST_FRACTIONAL_TOL 0.4999
//#define MOST_FRACTIONAL_SECONDARY 0.999

// best combo of good results and good times: most_frac = 0.35, tol = 0.3, secondary 0.999
// Note: fractional = 0.1 and tol = 0.099 gave very good results but took VERY long time..

// Key factor #3: How heavy do we weight "best dual variables" in the weighted column generation?
// (even a change from 2 -> 2.1 makes alot of difference..)
#define DW_WEIGHT 2
// best: 2

// When calculating scores from pseudocosts, weight with SCORE_WEIGHT:
#define SCORE_WEIGHT 0.1667
// 0.1667 ~ 1/6, from SCIP.

// how many times shall each "fractiona variable problem" be solved as strong branching?
#define NU_LIMIT 1



// After how many consecutive prunes to we use "pick lowest lb node" instead of "depth first" 
// when picking nodes from the tree? (prune repeat > (strict) PRUNE_REPEAT)
#define PRUNE_REPEAT 0
// best = 2 or 0

// define a search deph limit for depth-first search
#define DEPTH_LIMIT 5

// scale the up score as we want to branch down, generally..
#define SCALE_UP_SCORE 1

// define for which problem we should perform FULL column generation first ... iterations (no capping)
#define PROB_SPAN_DOWN 3.5
// 5.5
#define PROB_SPAN_UP 4.5
// 2.5
#define FULl_CG_NUM 500
// 50

// when do we switch to best-node/bound node picking only?
#define BOUND_SWITCH 1.03


#endif