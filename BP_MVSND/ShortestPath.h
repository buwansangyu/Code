#pragma once
#include "Instance.h"
// Definition of graph
typedef struct
{
	double* arc_cost;
	int* node_flag; // required by Dijkstra's algorithm
	double* dist; // distance from node predecessor
	int* pred_node; // node predecessor
	int* pred_arc; // arc from node predecessor
} path_graph;

typedef struct // neighboring node data structure
{
	int node; // node index
	int arc; // arc index
	double* distp; // pointer to arc cost value
} resnode_t;


typedef struct
{
	double* arc_cost; // arc cost for adding flow
	double* arc_cost_inv; // arc cost for subtracting flow

	resnode_t** node_outward; // accessible neighboring nodes
	int* node_n_outward;

	int list_size; // required by Ford-Bellman algorithm
	int* list; // idem

	double* dist; // distance from node predecessor
	int* pred_node; // node predecessor
	int* pred_arc; // arc from node predecessor
} residual_graph_t;

void InitialShortestPath(IloEnv env, IloIntArray pathArcs, IloInt commodityIndex, IloInt origin, IloInt dest, cmnd_t* instance);
void PathSubproblem(IloEnv env, IloIntArray pathArcs, IloInt commodityIndex, cmnd_t* instance, IloNumArray capacityDual);
void CycleBellmanFord(IloEnv env, IloIntArray pathArcs, IloInt nodeIndex, cmnd_t* instance, IloNumArray capacityDual, IloInt vehicleType);
// Local Search求最短路问题
// 给定一条要关闭的arc，找出替代的最短路径
IloNum FindShortestCycle(IloEnv env, IloIntArray pathArcs, IloInt arcIndex, cmnd_t* instance, IloBoolArray openOrnot, IloNum xij);
IloNum FindCycleBellmanFord(IloEnv env, IloIntArray pathArcs, IloInt candidateArc, cmnd_t* instance, IloBoolArray openOrnot, IloNum xij,
	IloIntArray arcs, IloIntArray arcIndex);
IloBool CheckCommodityFlow(IloEnv env, IloInt commodityIndex, cmnd_t* instance, IloBoolArray A_f);
IloNum FindCycleIntensification(IloEnv env, IloIntArray pathArcs, IloInt candidateArc, cmnd_t* instance, IloIntArray arcs);
void SolvedByCplex(IloEnv env, cmnd_t* instance);
