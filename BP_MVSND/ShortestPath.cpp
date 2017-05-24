#include "ShortestPath.h"

void SolvedByCplex(IloEnv env, cmnd_t* instance)
{
	IloModel arcModel(env);
	IloObjective totalcost = IloAdd(arcModel, IloMinimize(env));

	// Capacity Constraints	
	IloNumArray capacityLowerBound(env);
	IloNumArray capacityUpperBound(env);
	capacityLowerBound.add(instance->n_arcs, -IloInfinity);
	capacityUpperBound.add(instance->n_arcs, 0);
	IloRangeArray  capacityConstraints = IloAdd(arcModel, IloRangeArray(env, capacityLowerBound, capacityUpperBound));

	IloNumVarArray2 x(env, instance->n_commods);
	for (int i = 0; i < instance->n_commods; i++)
	{
		x[i] = IloNumVarArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			x[i].add(IloNumVar(env));
			totalcost.setLinearCoef(x[i][j], instance->arc_commod_ucost[j][i]);
			capacityConstraints[j].setLinearCoef(x[i][j], 1);
		}
	}

	IloIntVarArray2 y(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		y[i] = IloIntVarArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			y[i].add(IloIntVar(env));
			totalcost.setLinearCoef(y[i][j], vehicleFixedcost[i]);
			capacityConstraints[j].setLinearCoef(y[i][j], -vehicleCapacity[i]);
		}
	}

	// Demand Constraints	
	IloRangeArray2 demandConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		demandConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < instance->n_commods; j++)
		{
			demandConstraints[i].add(IloRange(env, instance->node_commod_supply[i][j], instance->node_commod_supply[i][j]));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
				demandConstraints[i][j].setLinearCoef(x[j][instance->node_outgoing_arc[i][k]], 1);
			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
				demandConstraints[i][j].setLinearCoef(x[j][instance->node_ingoing_arc[i][k]], -1);
		}
		arcModel.add(demandConstraints[i]);
	}
	// Design-balanced constraints
	IloRangeArray2 dbConstraints(env, instance->n_nodes);
	for (int i = 0; i < instance->n_nodes; i++)
	{
		dbConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < n_VehicleTypes; j++)
		{
			dbConstraints[i].add(IloRange(env, 0, 0));
			for (int k = 0; k < instance->node_n_outgoing_arcs[i]; k++)
				dbConstraints[i][j].setLinearCoef(y[j][instance->node_outgoing_arc[i][k]], 1);
			for (int k = 0; k < instance->node_n_ingoing_arcs[i]; k++)
				dbConstraints[i][j].setLinearCoef(y[j][instance->node_ingoing_arc[i][k]], -1);
		}
		arcModel.add(dbConstraints[i]);
	}

	IloCplex arcSolver(arcModel);
	arcSolver.setOut(env.getNullStream());
	/*for (int k = 0; k < n_VehicleTypes; k++)
	{
		arcModel.add(IloConversion(env, y[k], ILOFLOAT));
	}*/
	arcSolver.solve();
	printf("\n\nArc-based formulation is solved by Cplex: %f \n\n", arcSolver.getObjValue());

	// 输出y的值
	cout << " y in arc based model: " << endl;
	for (int i = 0; i < y.getSize(); i++)
	{
		for (int j = 0; j < y[i].getSize(); j++)
			cout << arcSolver.getValue(y[i][j]) << " ";
		cout << endl;
	}
	cout << " x in arc based model: " << endl;
	for (int i = 0; i < x.getSize(); i++)
	{
		for (int j = 0; j < x[i].getSize(); j++)
			cout << arcSolver.getValue(x[i][j]) << " ";
		cout << endl;
	}


	// Free memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	for (int i = 0; i < x.getSize(); i++)
		x[i].end();
	y.end();
	x.end();
	capacityLowerBound.end();
	capacityUpperBound.end();
	capacityConstraints.end();
	demandConstraints.end();
	dbConstraints.end();
}



void InitialShortestPath(IloEnv env, IloIntArray pathArcs, IloInt commodityIndex, IloInt origin, IloInt dest, cmnd_t* instance)
{
	// 初始化graph
	path_graph graph;
	int n_nodes = instance->n_nodes;
	int n_arcs = instance->n_arcs;
	graph.arc_cost = (double*)malloc(n_arcs * sizeof(double));
	graph.dist = (double*)malloc(n_nodes * sizeof(double));
	graph.node_flag = (int*)malloc(n_nodes * sizeof(int));
	graph.pred_node = (int*)(malloc(n_nodes * sizeof(int)));
	graph.pred_arc = (int*)(malloc(n_nodes * sizeof(double)));
	// 给定arc cost为：Max_capacity - arc_capacity
	double max_capacity = 0;
	for (int i = 0; i < instance->n_arcs; i++)
	{
		if (instance->arc_capacity[i] > max_capacity)
			max_capacity = instance->arc_capacity[i];
	}


	for (int i = 0; i < instance->n_arcs; i++)
		//graph.arc_cost[i] = max_capacity - instance->arc_capacity[i];
		graph.arc_cost[i] = instance->arc_commod_ucost[i][0];

	for (int j = 0; j < instance->n_nodes; j++) {
		graph.node_flag[j] = 1;
		graph.dist[j] = inf;
		graph.pred_node[j] = -1;
		graph.pred_arc[j] = -1;
	}

	// sets the artificial arc as linking the source and the sink
	// indentified by the value instance->n_arcs, which is not a valid arc index
	// this remains so until a better path is found
	int src, snk;
	src = origin;
	snk = dest;
	graph.dist[src] = 0.0;
	//标记orgin，用于回溯
	graph.pred_node[src] = -2;

	// applies Dijkstra's algorithm
	int i, j, k, l, c = -1;
	double cdist, alt;
	for (l = 0; l < instance->n_nodes; l++) {
		cdist = inf;
		c = -1;
		for (j = 0; j < instance->n_nodes; j++)
		if (graph.node_flag[j] && graph.dist[j] < cdist) {
			c = j;
			cdist = graph.dist[j];
		}

		if (c == -1)
			break;
		// 如果刚刚被标记的点为destination则跳出循环
		if (c == snk)
			break;
		graph.node_flag[c] = 0;
		for (k = 0; k < instance->node_n_outgoing_arcs[c]; k++) {
			i = instance->node_outgoing_arc[c][k];
			j = instance->arc_dest_node[i];
			alt = cdist + graph.arc_cost[i];
			if (alt < graph.dist[j]) {
				graph.dist[j] = alt;
				graph.pred_node[j] = c;
				graph.pred_arc[j] = i;
			}
		}
	}
	// 回溯找到path, 逆向存储在IloIntArray中
	pathArcs.add(instance->n_arcs + 1, -1);
	int flag = c, count = 0;
	while (graph.pred_node[flag] != -2)
	{
		pathArcs[instance->n_arcs - count] = graph.pred_arc[flag];
		flag = graph.pred_node[flag];
		count++;
	}
	// 最后一个不移除
	pathArcs.remove(0, instance->n_arcs + 1 - count);

	//cout << "dijkstra dist: " << graph.dist[snk] << endl;
	//cout << "src: " << src << " sin: " << snk <<endl;
	//cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	//cout << "path length: " << pathArcs.getSize() << endl;
	// 释放内存
	free(graph.arc_cost);
	free(graph.dist);
	free(graph.node_flag);
	free(graph.pred_node);
	free(graph.pred_arc);
}


// 产生path，cycle时可能出现重复添加cycle，path的问题，如果避免重复？？？？
void PathSubproblem(IloEnv env, IloIntArray pathArcs, IloInt commodityIndex, cmnd_t* instance, IloNumArray capacityDual)
{
	// 初始化graph
	path_graph graph;
	int n_nodes = instance->n_nodes;
	int n_arcs = instance->n_arcs;
	graph.arc_cost = (double*)malloc(n_arcs * sizeof(double));
	graph.dist = (double*)malloc(n_nodes * sizeof(double));
	graph.node_flag = (int*)malloc(n_nodes * sizeof(int));
	graph.pred_node = (int*)(malloc(n_nodes * sizeof(int)));
	graph.pred_arc = (int*)(malloc(n_nodes * sizeof(double)));
	// 给定arc cost为：c_ij_k - capacityDual_ij
	for (int i = 0; i < instance->n_arcs; i++)
	{
		graph.arc_cost[i] = instance->arc_commod_ucost[i][commodityIndex] - capacityDual[i];
		assert(graph.arc_cost[i] >= 0);
	}

	for (int j = 0; j < instance->n_nodes; j++) {
		graph.node_flag[j] = 1;
		graph.dist[j] = inf;
		graph.pred_node[j] = -1;
		graph.pred_arc[j] = -1;
	}

	// sets the artificial arc as linking the source and the sink
	// indentified by the value instance->n_arcs, which is not a valid arc index
	// this remains so until a better path is found
	int src, snk;
	src = instance->commod_orig_node[commodityIndex];
	snk = instance->commod_dest_node[commodityIndex];
	graph.dist[src] = 0.0;
	//标记orgin，用于回溯
	graph.pred_node[src] = -2;

	// applies Dijkstra's algorithm
	int i, j, k, l, c = -1;
	double cdist, alt;
	for (l = 0; l < instance->n_nodes; l++) {
		cdist = inf;
		c = -1;
		for (j = 0; j < instance->n_nodes; j++)
		if (graph.node_flag[j] && graph.dist[j] < cdist) {
			c = j;
			cdist = graph.dist[j];
		}

		if (c == -1)
			break;
		// 如果刚刚被标记的点为destination则跳出循环
		if (c == snk)
			break;
		graph.node_flag[c] = 0;
		for (k = 0; k < instance->node_n_outgoing_arcs[c]; k++) {
			i = instance->node_outgoing_arc[c][k];
			j = instance->arc_dest_node[i];
			alt = cdist + graph.arc_cost[i];
			if (alt < graph.dist[j]) {
				graph.dist[j] = alt;
				graph.pred_node[j] = c;
				graph.pred_arc[j] = i;
			}
		}
	}
	// 回溯找到path, 逆向存储在IloIntArray中
	pathArcs.add(instance->n_arcs + 1, -1);
	int flag = c, count = 0;
	while (graph.pred_node[flag] != -2)
	{
		pathArcs[instance->n_arcs - count] = graph.pred_arc[flag];
		flag = graph.pred_node[flag];
		count++;
	}

	// 最后一个不移除
	pathArcs.remove(0, instance->n_arcs + 1 - count);

	//cout << "dijkstra dist: " << graph.dist[snk] << endl;
	//cout << "src: " << src << " sin: " << snk <<endl;
	//cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	//cout << "path length: " << pathArcs.getSize() << endl;
	// 释放内存
	free(graph.arc_cost);
	free(graph.dist);
	free(graph.node_flag);
	free(graph.pred_node);
	free(graph.pred_arc);
}



int is_in_list(residual_graph_t* graphp, int node_maybe_in_list)
// returns true if node_maybe_in_list is in visited nodes list
{
	int i;

	for (i = 0; i < graphp->list_size; i++)
	if (graphp->list[i] == node_maybe_in_list)
		return 1;

	return 0;
}


int is_in_path(residual_graph_t* graphp, int node_current, int node_maybe_in_path)
// returns true if node_maybe_in_path is in the path leading from the source to node_current
{
	int c;
	// -1改为-2，便于回溯
	for (c = node_current; c != -2; c = graphp->pred_node[c])
	if (node_maybe_in_path == c)
		return 1;

	return 0;
}
int resnode_cmp(const void* a, const void* b)
// comparison function for qsort, comparing neighbors, preferring those linked with the cheapest arcs
{
	resnode_t* rsap = (resnode_t*)a;
	resnode_t* rsbp = (resnode_t*)b;

	if (*rsap->distp == *rsbp->distp)
		return 0;
	else
		return ((*rsap->distp < *rsbp->distp) ? -1 : 1);
}



void CycleBellmanFord(IloEnv env, IloIntArray pathArcs, IloInt nodeIndex, cmnd_t* instance, IloNumArray capacityDual, IloInt vehicleType)
{

	residual_graph_t graphp;
	int j;
	int n_nodes = instance->n_nodes;
	int n_arcs = instance->n_arcs;

	graphp.list_size = 0;
	graphp.list = (int*)(malloc(n_nodes * sizeof(int)));
	// 添加一个虚拟的node作为destination
	graphp.arc_cost = (double*)(malloc(n_arcs * sizeof(double)));
	graphp.arc_cost_inv = (double*)(malloc(n_arcs * sizeof(double)));
	graphp.dist = (double*)(malloc((n_nodes + 1) * sizeof(double)));
	graphp.pred_node = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.pred_arc = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_n_outward = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_outward = (resnode_t**)(malloc(n_nodes * sizeof(resnode_t*)));

	for (j = 0; j < instance->n_nodes; j++) {
		graphp.node_n_outward[j] = 0;
		graphp.node_outward[j] = (resnode_t*)(malloc(2 * n_nodes * sizeof(resnode_t)));
	}

	int k, o, d;
	resnode_t resnode;
	// 初始化cost, h_ij_f + u_f * capacityDual_ij
	for (int i = 0; i < instance->n_arcs; i++)
		graphp.arc_cost[i] = graphp.arc_cost_inv[i] = vehicleFixedcost[vehicleType] + vehicleCapacity[vehicleType] * capacityDual[i];

	// 初始化node_outward
	for (int i = 0; i < instance->n_arcs; i++) {
		o = instance->arc_orig_node[i];
		d = instance->arc_dest_node[i];

		// 判断如果destination node为起始点的话则标记为后增加的虚拟origin node
		if (d == nodeIndex)
			d = n_nodes;

		if (graphp.arc_cost[i] != inf) {
			resnode.node = d;
			resnode.arc = i;
			resnode.distp = &graphp.arc_cost[i];
			assert(graphp.node_n_outward[o] < 2 * instance->n_nodes);
			graphp.node_outward[o][(graphp.node_n_outward[o])++] = resnode;
		}

		//if (graphp.arc_cost_inv[i] != inf) {
		//	resnode.node = o;
		//	resnode.arc = i;
		//	resnode.distp = &graphp.arc_cost_inv[i];
		//	assert(graphp.node_n_outward[d] < 2 * instance->n_nodes);
		//	graphp.node_outward[d][(graphp.node_n_outward[d])++] = resnode;
		//}
	}

	// for each node, sort the neighbors in order of ascending distance
	for (j = 0; j < instance->n_nodes; j++)
		qsort(graphp.node_outward[j], graphp.node_n_outward[j], sizeof(resnode_t), resnode_cmp);



	// reset precedence information
	for (j = 0; j < instance->n_nodes + 1; j++) {
		graphp.dist[j] = inf;
		graphp.pred_node[j] = -1;
		graphp.pred_arc[j] = -1;
	}

	graphp.list_size = 1;
	int src = nodeIndex;
	graphp.list[0] = src;
	graphp.dist[src] = 0.0;
	graphp.pred_node[src] = -2;

	// apply Ford-Bellman
	while (graphp.list_size > 0) {
		o = graphp.list[--graphp.list_size];
		for (k = 0; k < graphp.node_n_outward[o]; k++) {
			resnode = graphp.node_outward[o][k];
			if (graphp.dist[resnode.node] > graphp.dist[o] + (*resnode.distp) && !is_in_path(&graphp, o, resnode.node)) {
				graphp.dist[resnode.node] = graphp.dist[o] + (*resnode.distp);
				graphp.pred_node[resnode.node] = o;
				graphp.pred_arc[resnode.node] = resnode.arc;

				if (!is_in_list(&graphp, resnode.node))
					graphp.list[graphp.list_size++] = resnode.node;
			}
		}
	}
	// dest的index为n_nodes
	int dest = n_nodes;
	IloNum dist = graphp.dist[dest];

	pathArcs.add(instance->n_arcs + 1, -1);
	int flag = dest, count = 0;
	while (graphp.pred_node[flag] != -2)
	{
		pathArcs[instance->n_arcs - count] = graphp.pred_arc[flag];
		flag = graphp.pred_node[flag];
		count++;
	}
	// 最后一个不移除
	pathArcs.remove(0, instance->n_arcs + 1 - count);

	//cout << "src: " << src << " sin: " << nodeIndex << endl;
	//cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	//for (int i = 0; i < pathArcs.getSize(); i++)
	//	cout << instance->arc_orig_node[pathArcs[i]] << "->" << instance->arc_dest_node[pathArcs[i]] << endl;
	//cout << "path length: " << pathArcs.getSize() << endl;


	for (j = 0; j < instance->n_nodes; j++)
		free(graphp.node_outward[j]);

	free(graphp.list);
	free(graphp.arc_cost);
	free(graphp.arc_cost_inv);
	free(graphp.dist);
	free(graphp.pred_node);
	free(graphp.pred_arc);
	free(graphp.node_n_outward);
	free(graphp.node_outward);

}

// Local Search求最短路问题
// 给定一条要关闭的arc，找出替代的最短路径
IloNum FindCycleBellmanFord(IloEnv env, IloIntArray pathArcs, IloInt candidateArc, cmnd_t* instance, IloBoolArray openOrnot, IloNum xij,
	IloIntArray arcs, IloIntArray arcIndex)
{
	pathArcs.clear();
	residual_graph_t graphp;
	int j;
	int n_nodes = instance->n_nodes;
	int n_arcs = arcs.getSize();
	IloInt nodeIndex = instance->arc_orig_node[candidateArc];
	graphp.list_size = 0;
	graphp.list = (int*)(malloc(n_nodes * sizeof(int)));
	// 添加一个虚拟的node作为destination
	graphp.arc_cost = (double*)(malloc(n_arcs * sizeof(double)));
	graphp.dist = (double*)(malloc((n_nodes + 1) * sizeof(double)));
	graphp.pred_node = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.pred_arc = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_n_outward = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_outward = (resnode_t**)(malloc(n_nodes * sizeof(resnode_t*)));

	for (j = 0; j < instance->n_nodes; j++) {
		graphp.node_n_outward[j] = 0;
		graphp.node_outward[j] = (resnode_t*)(malloc(2 * n_nodes * sizeof(resnode_t)));
	}

	int k, o, d;
	resnode_t resnode;
	// 初始化arc cost
	IloNum fixcost = 0;
	for (int i = 0; i < n_VehicleTypes; i++)
		fixcost += vehicleFixedcost[i];
	// 平均车辆车本
	fixcost /= n_VehicleTypes;
	//fixcost = 0;
	for (int i = 0; i < n_arcs; i++)
	{
		// ji-(candidate arc)必须包含在cycle中
		// 可能存在ji-中不包含candidate arc反向弧的情况！！！！
		if (arcIndex[i] == candidateArc)
		{
			graphp.arc_cost[i] = -10000;
			continue;
		}

		// ij+
		if (arcs[i] == 2)
		{
			if (openOrnot[arcIndex[i]])
				graphp.arc_cost[i] = instance->arc_commod_ucost[arcIndex[i]][0] * xij;
			/*else
				graphp.arc_cost[i] = instance->arc_commod_ucost[arcIndex[i]][0] * xij + fixcost;*/
		}
		// ji-: xijk > flow
		else if (arcs[i] == 1)
		{
			graphp.arc_cost[i] = -instance->arc_commod_ucost[arcIndex[i]][0] * xij;
		}
		// ji-: xijk = flow
		else
		{
			graphp.arc_cost[i] = -instance->arc_commod_ucost[arcIndex[i]][0] * xij - fixcost;
		}

	}

	// 初始化node_outward
	for (int i = 0; i < n_arcs; i++) {
		// ij+
		if (arcs[i] == 2)
		{
			o = instance->arc_orig_node[arcIndex[i]];
			d = instance->arc_dest_node[arcIndex[i]];
		}
		// ji-
		else
		{
			d = instance->arc_orig_node[arcIndex[i]];
			o = instance->arc_dest_node[arcIndex[i]];
		}

		// 判断如果destination node为起始点的话则标记为后增加的虚拟origin node
		if (d == nodeIndex)
			d = n_nodes;

		if (graphp.arc_cost[i] != inf) {
			resnode.node = d;
			resnode.arc = i;
			resnode.distp = &graphp.arc_cost[i];
			assert(graphp.node_n_outward[o] < 2 * instance->n_nodes);
			graphp.node_outward[o][(graphp.node_n_outward[o])++] = resnode;
		}
	}

	// for each node, sort the neighbors in order of ascending distance
	for (j = 0; j < instance->n_nodes; j++)
		qsort(graphp.node_outward[j], graphp.node_n_outward[j], sizeof(resnode_t), resnode_cmp);



	// reset precedence information
	for (j = 0; j < instance->n_nodes + 1; j++) {
		graphp.dist[j] = inf;
		graphp.pred_node[j] = -1;
		graphp.pred_arc[j] = -1;
	}

	graphp.list_size = 1;
	int src = nodeIndex;
	graphp.list[0] = src;
	graphp.dist[src] = 0.0;
	graphp.pred_node[src] = -2;

	// apply Ford-Bellman
	while (graphp.list_size > 0) {
		o = graphp.list[--graphp.list_size];
		for (k = 0; k < graphp.node_n_outward[o]; k++) {
			resnode = graphp.node_outward[o][k];
			if (graphp.dist[resnode.node] > graphp.dist[o] + (*resnode.distp) && !is_in_path(&graphp, o, resnode.node)) {
				graphp.dist[resnode.node] = graphp.dist[o] + (*resnode.distp);
				graphp.pred_node[resnode.node] = o;
				graphp.pred_arc[resnode.node] = resnode.arc;

				if (!is_in_list(&graphp, resnode.node))
					graphp.list[graphp.list_size++] = resnode.node;
			}
		}
	}
	// dest的index为n_nodes
	int dest = n_nodes;
	IloNum dist = graphp.dist[dest];
	if (dist != INFINITY)
	{
		pathArcs.add(n_arcs + 1, -1);
		int flag = dest, count = 0;
		while (graphp.pred_node[flag] != -2)
		{
			pathArcs[n_arcs - count] = graphp.pred_arc[flag];
			flag = graphp.pred_node[flag];
			count++;
		}
		// 最后一个不移除
		pathArcs.remove(0, n_arcs + 1 - count);

		// index的值转换为原始index
		for (int i = 0; i < pathArcs.getSize(); i++)
			pathArcs[i] = arcIndex[pathArcs[i]];
	}
	

	/*cout << "src: " << src << " sin: " << nodeIndex << endl;
	cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	for (int i = 0; i < pathArcs.getSize(); i++)
	cout << instance->arc_orig_node[pathArcs[i]] << "->" << instance->arc_dest_node[pathArcs[i]] << endl;
	cout << "path length: " << pathArcs.getSize() << endl;*/



	for (j = 0; j < instance->n_nodes; j++)
		free(graphp.node_outward[j]);

	free(graphp.list);
	free(graphp.arc_cost);
	free(graphp.dist);
	free(graphp.pred_node);
	free(graphp.pred_arc);
	free(graphp.node_n_outward);
	free(graphp.node_outward);
	return dist;
}

IloBool CheckCommodityFlow(IloEnv env, IloInt commodityIndex, cmnd_t* instance, IloBoolArray A_f)
{
	// 初始化graph
	path_graph graph;
	int n_nodes = instance->n_nodes;
	int n_arcs = instance->n_arcs;
	graph.arc_cost = (double*)malloc(n_arcs * sizeof(double));
	graph.dist = (double*)malloc(n_nodes * sizeof(double));
	graph.node_flag = (int*)malloc(n_nodes * sizeof(int));
	graph.pred_node = (int*)(malloc(n_nodes * sizeof(int)));
	graph.pred_arc = (int*)(malloc(n_nodes * sizeof(double)));
	// 给定arc cost为：1
	IloNum fixcost = 0;
	for (int i = 0; i < n_VehicleTypes; i++)
		fixcost += vehicleFixedcost[i];
	fixcost /= instance->n_arcs;
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// A_f不包含的弧cost无穷大
		if (A_f[i])
			graph.arc_cost[i] = 1;
		else
			graph.arc_cost[i] = INFINITY;
	}

	for (int j = 0; j < instance->n_nodes; j++) {
		graph.node_flag[j] = 1;
		graph.dist[j] = inf;
		graph.pred_node[j] = -1;
		graph.pred_arc[j] = -1;
	}

	// sets the artificial arc as linking the source and the sink
	// indentified by the value instance->n_arcs, which is not a valid arc index
	// this remains so until a better path is found
	int src, snk;
	src = instance->commod_orig_node[commodityIndex];
	snk = instance->commod_dest_node[commodityIndex];
	graph.dist[src] = 0.0;
	//标记orgin，用于回溯
	graph.pred_node[src] = -2;

	// applies Dijkstra's algorithm
	int i, j, k, l, c = -1;
	double cdist, alt;
	for (l = 0; l < instance->n_nodes; l++) {
		cdist = inf;
		c = -1;
		for (j = 0; j < instance->n_nodes; j++)
		if (graph.node_flag[j] && graph.dist[j] < cdist) {
			c = j;
			cdist = graph.dist[j];
		}

		if (c == -1)
			break;
		// 如果刚刚被标记的点为destination则跳出循环
		if (c == snk)
			break;
		graph.node_flag[c] = 0;
		for (k = 0; k < instance->node_n_outgoing_arcs[c]; k++) {
			i = instance->node_outgoing_arc[c][k];
			j = instance->arc_dest_node[i];
			alt = cdist + graph.arc_cost[i];
			if (alt < graph.dist[j]) {
				graph.dist[j] = alt;
				graph.pred_node[j] = c;
				graph.pred_arc[j] = i;
			}
		}
	}

	IloNum totalcost = graph.dist[snk];
	//cout << "dijkstra dist: " << graph.dist[snk] << endl;
	//cout << "src: " << src << " sin: " << snk <<endl;

	// 释放内存
	free(graph.arc_cost);
	free(graph.dist);
	free(graph.node_flag);
	free(graph.pred_node);
	free(graph.pred_arc);

	if (totalcost < INFINITY)
		return true;
	else
		return false;
}


// Intensification函数中找cycle
// 第一个arc必须包含在cycle中
IloNum FindCycleIntensification(IloEnv env, IloIntArray pathArcs, IloInt candidateArc, cmnd_t* instance, IloIntArray arcs)
{
	pathArcs.clear();
	residual_graph_t graphp;
	int j;
	int n_nodes = instance->n_nodes;
	int n_arcs = arcs.getSize();
	IloInt nodeIndex = instance->arc_orig_node[candidateArc];
	graphp.list_size = 0;
	graphp.list = (int*)(malloc(n_nodes * sizeof(int)));
	// 添加一个虚拟的node作为destination
	graphp.arc_cost = (double*)(malloc(n_arcs * sizeof(double)));
	graphp.dist = (double*)(malloc((n_nodes + 1) * sizeof(double)));
	graphp.pred_node = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.pred_arc = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_n_outward = (int*)(malloc((n_nodes + 1) * sizeof(int)));
	graphp.node_outward = (resnode_t**)(malloc(n_nodes * sizeof(resnode_t*)));

	for (j = 0; j < instance->n_nodes; j++) {
		graphp.node_n_outward[j] = 0;
		graphp.node_outward[j] = (resnode_t*)(malloc(2 * n_nodes * sizeof(resnode_t)));
	}

	int k, o, d;
	resnode_t resnode;
	// 初始化arc cost
	// 第一个arc必须包含在cycle中，所以设为负值，其他为正值
	graphp.arc_cost[0] = -100;
	for (int i = 1; i < n_arcs; i++)
			graphp.arc_cost[i] = 1;

	// 初始化node_outward
	for (int i = 0; i < n_arcs; i++) {
		//如果有反向arc则o d相反
		if (arcs[i] == -1)
		{
			d = instance->arc_orig_node[candidateArc];
			o = instance->arc_dest_node[candidateArc];
		}
		else
		{
			o = instance->arc_orig_node[arcs[i]];
			d = instance->arc_dest_node[arcs[i]];
		}
		

		// 判断如果destination node为起始点的话则标记为后增加的虚拟origin node
		if (d == nodeIndex)
			d = n_nodes;

		if (graphp.arc_cost[i] != inf) {
			resnode.node = d;
			// 存储instance中的arc index值
			resnode.arc = arcs[i];
			resnode.distp = &graphp.arc_cost[i];
			assert(graphp.node_n_outward[o] < 2 * instance->n_nodes);
			graphp.node_outward[o][(graphp.node_n_outward[o])++] = resnode;
		}
	}

	// for each node, sort the neighbors in order of ascending distance
	for (j = 0; j < instance->n_nodes; j++)
		qsort(graphp.node_outward[j], graphp.node_n_outward[j], sizeof(resnode_t), resnode_cmp);



	// reset precedence information
	for (j = 0; j < instance->n_nodes + 1; j++) {
		graphp.dist[j] = inf;
		graphp.pred_node[j] = -1;
		graphp.pred_arc[j] = -1;
	}

	graphp.list_size = 1;
	int src = nodeIndex;
	graphp.list[0] = src;
	graphp.dist[src] = 0.0;
	graphp.pred_node[src] = -2;

	// apply Ford-Bellman
	while (graphp.list_size > 0) {
		o = graphp.list[--graphp.list_size];
		for (k = 0; k < graphp.node_n_outward[o]; k++) {
			resnode = graphp.node_outward[o][k];
			if (graphp.dist[resnode.node] > graphp.dist[o] + (*resnode.distp) && !is_in_path(&graphp, o, resnode.node)) {
				graphp.dist[resnode.node] = graphp.dist[o] + (*resnode.distp);
				graphp.pred_node[resnode.node] = o;
				graphp.pred_arc[resnode.node] = resnode.arc;

				if (!is_in_list(&graphp, resnode.node))
					graphp.list[graphp.list_size++] = resnode.node;
			}
		}
	}
	// dest的index为n_nodes
	int dest = n_nodes;
	IloNum dist = graphp.dist[dest];
	// dist不为无穷大表明找到了cycle
	if (dist != INFINITY)
	{
		pathArcs.add(n_arcs + 1, -1);
		int flag = dest, count = 0;
		while (graphp.pred_node[flag] != -2)
		{
			pathArcs[n_arcs - count] = graphp.pred_arc[flag];
			flag = graphp.pred_node[flag];
			count++;
		}
		// 最后一个不移除
		pathArcs.remove(0, n_arcs + 1 - count);
	}


	/*cout << "src: " << src << " sin: " << nodeIndex << endl;
	cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	for (int i = 0; i < pathArcs.getSize(); i++)
	cout << instance->arc_orig_node[pathArcs[i]] << "->" << instance->arc_dest_node[pathArcs[i]] << endl;
	cout << "path length: " << pathArcs.getSize() << endl;*/



	for (j = 0; j < instance->n_nodes; j++)
		free(graphp.node_outward[j]);

	free(graphp.list);
	free(graphp.arc_cost);
	free(graphp.dist);
	free(graphp.pred_node);
	free(graphp.pred_arc);
	free(graphp.node_n_outward);
	free(graphp.node_outward);
	return dist;
}

IloNum FindShortestCycle(IloEnv env, IloIntArray pathArcs, IloInt arcIndex, cmnd_t* instance, IloBoolArray openOrnot, IloNum xij)
{
	// 初始化graph
	path_graph graph;
	int n_nodes = instance->n_nodes;
	int n_arcs = instance->n_arcs;
	graph.arc_cost = (double*)malloc(n_arcs * sizeof(double));
	graph.dist = (double*)malloc(n_nodes * sizeof(double));
	graph.node_flag = (int*)malloc(n_nodes * sizeof(int));
	graph.pred_node = (int*)(malloc(n_nodes * sizeof(int)));
	graph.pred_arc = (int*)(malloc(n_nodes * sizeof(double)));
	// 给定arc cost为：c_ij_k * xij + openOrnot * fixedcost
	IloNum fixcost = 0;
	for (int i = 0; i < n_VehicleTypes; i++)
		fixcost += vehicleFixedcost[i];
	fixcost /= instance->n_arcs;
	for (int i = 0; i < instance->n_arcs; i++)
	{
		// 关闭的arc cost设为大值
		if (i == arcIndex)
		{
			graph.arc_cost[i] = 10e6;
			continue;
		}
		if (openOrnot[i])
			graph.arc_cost[i] = instance->arc_commod_ucost[i][0] * xij;
		else
			graph.arc_cost[i] = instance->arc_commod_ucost[i][0] * xij + fixcost;
		assert(graph.arc_cost[i] >= 0);
	}

	for (int j = 0; j < instance->n_nodes; j++) {
		graph.node_flag[j] = 1;
		graph.dist[j] = inf;
		graph.pred_node[j] = -1;
		graph.pred_arc[j] = -1;
	}

	// sets the artificial arc as linking the source and the sink
	// indentified by the value instance->n_arcs, which is not a valid arc index
	// this remains so until a better path is found
	int src, snk;
	src = instance->arc_orig_node[arcIndex];
	snk = instance->arc_dest_node[arcIndex];
	graph.dist[src] = 0.0;
	//标记orgin，用于回溯
	graph.pred_node[src] = -2;

	// applies Dijkstra's algorithm
	int i, j, k, l, c = -1;
	double cdist, alt;
	for (l = 0; l < instance->n_nodes; l++) {
		cdist = inf;
		c = -1;
		for (j = 0; j < instance->n_nodes; j++)
		if (graph.node_flag[j] && graph.dist[j] < cdist) {
			c = j;
			cdist = graph.dist[j];
		}

		if (c == -1)
			break;
		// 如果刚刚被标记的点为destination则跳出循环
		if (c == snk)
			break;
		graph.node_flag[c] = 0;
		for (k = 0; k < instance->node_n_outgoing_arcs[c]; k++) {
			i = instance->node_outgoing_arc[c][k];
			j = instance->arc_dest_node[i];
			alt = cdist + graph.arc_cost[i];
			if (alt < graph.dist[j]) {
				graph.dist[j] = alt;
				graph.pred_node[j] = c;
				graph.pred_arc[j] = i;
			}
		}
	}
	// 回溯找到path, 逆向存储在IloIntArray中
	pathArcs.add(instance->n_arcs + 1, -1);
	int flag = c, count = 0;
	while (graph.pred_node[flag] != -2)
	{
		pathArcs[instance->n_arcs - count] = graph.pred_arc[flag];
		flag = graph.pred_node[flag];
		count++;
	}

	// 最后一个不移除
	pathArcs.remove(0, instance->n_arcs + 1 - count);
	IloNum totalcost = graph.dist[snk];
	//cout << "dijkstra dist: " << graph.dist[snk] << endl;
	//cout << "src: " << src << " sin: " << snk <<endl;
	//cout << instance->arc_orig_node[pathArcs[0]] << " " << instance->arc_dest_node[pathArcs[pathArcs.getSize() - 1]] << endl;
	//cout << "path length: " << pathArcs.getSize() << endl;
	// 释放内存
	free(graph.arc_cost);
	free(graph.dist);
	free(graph.node_flag);
	free(graph.pred_node);
	free(graph.pred_arc);
	return totalcost;
}

