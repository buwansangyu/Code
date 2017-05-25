#ifndef CMND_H
#define CMND_H
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <float.h>
#include <ilcplex/ilocplex.h>
#include <time.h>
#include <iostream>
using namespace std;
typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloIntVarArray> IloIntVarArray2;
typedef IloArray<IloRangeArray> IloRangeArray2;

//const int n_VehicleTypes = 10;
//const double vehicleCapacity[10] = { 3, 5, 7, 10, 13, 15, 18, 21, 24, 28 };
//const double vehicleFixedcost[10] = { 101, 121, 145, 168, 189, 214, 234, 256, 275, 297 };


const int n_VehicleTypes = 5;
 //车辆容量[10,30]
//const double vehicleFixedcost[5] = { 101, 145, 168, 189, 214 };
 //车辆容量[2,15]
const double vehicleCapacity[5] = { 3, 7, 10, 13, 15 };


const double vehicleFixedcost[5] = { 97, 145, 189, 256, 297 };
//const double vehicleFixedcost[5] = { 14, 29, 36, 45, 50 };
//const double vehicleFixedcost[5] = { 11, 15, 19, 24, 29 };
//const double vehicleCapacity[5] = { 10, 15, 20, 25, 30 };
//const double vehicleFixedcost[5] = { 100, 150, 200, 250, 300 };

//const double vehicleCapacity[5] = { 20, 30, 40, 50, 60 };
//const double vehicleFixedcost[5] = { 100, 150, 200, 250, 300 };
//const double vehicleFixedcost[5] = { 50, 70, 90, 110, 130 };

#define RC_EPS 1.0e-6  // 作为判断reduced cost是否为非负数的一个临界值
//#define RC_EPS 0  // 作为判断reduced cost是否为非负数的一个临界值

#ifndef INFINITY
#define INFINITY DBL_MAX
#endif


static const double inf = INFINITY;

typedef uint8_t IPC_header_t;

static const IPC_header_t IPC_error = 0;
static const IPC_header_t IPC_nothing = 1;
static const IPC_header_t IPC_end_search = 2;
static const IPC_header_t IPC_new_z_ub = 3;
static const IPC_header_t IPC_design = 4;
static const IPC_header_t IPC_solution = 5;
// 50 + 5 * X
//static const int NumVehType = 5;
//static const double vehicleCa[5] = { 20, 30, 40, 50, 60 }; //不同的容量
//static const double vhicleFC[5] = { 150, 200, 250, 300, 350 };//不同的固定成本

typedef struct
{
	char name[256];
	int n_nodes;
	int n_arcs;
	int n_commods;

	
	// arc attributes
	int* arc_orig_node;
	int* arc_dest_node;
	double* arc_capacity;
	double* arc_fcost; // fixed cost
	double** arc_commod_ucost; // unit cost
	double** arc_commod_bound; // min(arc capacity, commodity supply)

	// commodity attributes
	int* commod_orig_node; // source
	int* commod_dest_node; // sink
	double* commod_supply;
	double* commod_overflow_ucost; // unit cost of artificial arc linking source to sink (unit cost = bigM, fixed cost = 0, capacity = inf)

	// node attributes
	int* node_n_outgoing_arcs;
	int* node_n_ingoing_arcs;
	int** node_outgoing_arc;
	int** node_ingoing_arc;
	double** node_commod_supply; // +supply if source, -supply if sink, 0 otherwise

	// statistical info
	double avg_arc_fcost;
	double min_arc_fcost;
	double max_arc_fcost;
} cmnd_t;


cmnd_t cmnd_create(char* filename);
void cmnd_destroy(cmnd_t* inst);

#endif
