// ------------------------------------------------------------------------------------
// Main source file code for the project. Apart from connecting all essential
// algorithmic parts (building models, column generation, etc), it also contains
// main loop for the branch and bound/price procedure. 
// ------------------------------------------------------------------------------------

// project source files common headers
#include "Instance.h"
#include "BrachAndBound.h"


// ---------------------------------------------------------------------------------------------------------- //
// Function:	main(...)
//
// Purpose:	main function of program. (connecting all program segments).
// ---------------------------------------------------------------------------------------------------------- //
int main(int argc, char **argv)
{
	// create the optimization enviroment	
	IloEnv env;
	try {
		//char* datafile = "../../../data/R/r01.1.dow";
		//char* datafile = "../../../data/C/c36.dow";
		//char* datafile = "../../../data/CPlus/c100_400_30_V_T_10.dow";
		char* datafile = "../../../data/S/S03.txt";
		cmnd_t instance;
		instance = cmnd_create(datafile);
		//SolvedByCplex(env, &instance);
		BrachAndBound(env, &instance);


		cmnd_destroy(&instance);
	}
	// catch (known Concert) exception by _reference_ (catching by value might
	// end up in loss of information).
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;

		// free any memory that may be used by arrays (or expressions) of the
		// exception thrown.
		e.end();
	}
	catch (...) {
		cerr << "Unknown exception caught" << endl;
	}

	// close the optimization enviroment
	env.end();
	system("pause");
	// end of main...
	return 0;
}

