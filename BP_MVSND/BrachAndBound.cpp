#include "BrachAndBound.h"
#include "CutsetInequality.h"
#include "UpperBound.h"
void BrachAndBound(IloEnv env, cmnd_t* instance)
{

#pragma region initial column generation
	//
	//masterModel problem
	//
	IloModel masterModel(env);
	IloObjective totalCost = IloAdd(masterModel, IloMinimize(env));
	IloNumVarArray2 y(env, n_VehicleTypes);
	IloNumVarArray2 x(env, instance->n_commods);
	for (int i = 0; i < n_VehicleTypes; i++)
		y[i] = IloNumVarArray(env);
	for (int i = 0; i < instance->n_commods; i++)
		x[i] = IloNumVarArray(env);

	// Capacity Constraints	
	IloNumArray capacityLowerBound(env);
	IloNumArray capacityUpperBound(env);
	capacityLowerBound.add(instance->n_arcs, -IloInfinity);
	capacityUpperBound.add(instance->n_arcs, 0);
	IloRangeArray  capacityConstraints = IloAdd(masterModel, IloRangeArray(env, capacityLowerBound, capacityUpperBound));
	// Demand Constraints	
	IloRangeArray demandConstraints(env);
	for (int i = 0; i < instance->n_commods; i++)
		demandConstraints.add(IloRange(env, instance->commod_supply[i], instance->commod_supply[i]));
	masterModel.add(demandConstraints);

	// ÿһ�ֳ��͵ĳ�����Z_f
	IloNumVarArray zType(env, n_VehicleTypes);
	IloRangeArray zTypeEqualConstrains(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		zType[i] = IloNumVar(env);
		zTypeEqualConstrains[i] = IloRange(env, 0, 0);
		zTypeEqualConstrains[i].setLinearCoef(zType[i], -1);
	}
	masterModel.add(zTypeEqualConstrains);


	// ���ÿ������ÿ�ֳ��������ı���z_ij_f
	IloNumVarArray2 z(env, n_VehicleTypes);
	IloRangeArray2 EqualZConstraints(env, n_VehicleTypes);
	for (int i = 0; i < n_VehicleTypes; i++)
	{
		z[i] = IloNumVarArray(env);
		EqualZConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			z[i].add(IloNumVar(env));
			EqualZConstraints[i].add(IloRange(env, 0, 0));
			EqualZConstraints[i][j].setLinearCoef(z[i][j], -1);
			zTypeEqualConstrains[i].setLinearCoef(z[i][j], 1);
		}
		masterModel.add(EqualZConstraints[i]);
	}


	// x_ij_k ���ڲ���ǿ��Ч����ʽ
	IloNumVarArray2 xijk(env, instance->n_commods);
	IloRangeArray2 EqualXijkConstraints(env, instance->n_commods);
	for (int i = 0; i < instance->n_commods; i++)
	{
		xijk[i] = IloNumVarArray(env);
		EqualXijkConstraints[i] = IloRangeArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
		{
			xijk[i].add(IloNumVar(env));
			EqualXijkConstraints[i].add(IloRange(env, 0, 0));
			EqualXijkConstraints[i][j].setLinearCoef(xijk[i][j], -1);
		}
		masterModel.add(EqualXijkConstraints[i]);
	}

	//��ӳ�ʼ��
	AddInitialColumns(env, instance, totalCost, capacityConstraints, demandConstraints, EqualZConstraints, EqualXijkConstraints, x, y);

	IloCplex masterSolver(masterModel);
	//masterSolver.setParam(IloCplex::RootAlg, IloCplex::Primal);
	masterSolver.setOut(env.getNullStream());

#pragma endregion initial column generation


#pragma region Branch-and-bound
	// ---------------------------------------------------------------------------------------------------------- //
	// 	LP-based branch and bound (e.g. no updating of column pool, which would be dynamic pricing)
	// ---------------------------------------------------------------------------------------------------------- //

	// �洢z��ֵ
	IloNumArray zTypeValue(env, n_VehicleTypes);

	IloNumArray2 zValue(env, z.getSize());
	for (int i = 0; i < z.getSize(); i++)
	{
		zValue[i] = IloNumArray(env);
	}

	IloNumArray2 yValue(env, y.getSize());
	for (int i = 0; i < y.getSize(); i++)
	{
		yValue[i] = IloNumArray(env);
	}

	IloNumArray2 xijkValue(env, xijk.getSize());
	for (int i = 0; i < xijk.getSize(); i++)
	{
		xijkValue[i] = IloNumArray(env);
	}

	// ���ǿ��Ч����ʽ�Ƿ������
	IloBoolArray2 boolxij(env, instance->n_commods);
	for (int i = 0; i < boolxij.getSize(); i++)
	{
		boolxij[i] = IloBoolArray(env);
		for (int j = 0; j < instance->n_arcs; j++)
			boolxij[i].add(false);
	}

	// ǿ��Ч����ʽ
	IloRangeArray tempStrongCons(env);



	// iteration (e.g. nodes visited) counter for bnb
	IloInt itNum = 0;

	nodeList tree(&env);
	node * activeNode = NULL;
	IloNum nodeLowerBound, treeLowerBound;
	// numeric to hold bounds. Note: upper bound is best integer solution,
	// declared and initialized after init heirustic (nodeUpperBound)
	// �̶����������Ͻ�
	IloNum nodeUpperBound = 10e10;
	// ȫ���Ͻ�
	IloNum TreeUpperBound = nodeUpperBound;

	// ���ɵĸ�
	IloRangeArray tempRmpCuts(env);
	// make note if rmp is feasible (during each iteraiton)
	IloBool rmpFeasible;

	// ����Ϊ�˼ӿ�����ٶȿ���ȥ��������
	// open file to print problen list size to
	ofstream outFileAnalysis("results/branchOutput.dat");
	ofstream treeOutput("results/treeOutput.dat");

	// ���ڵ����
	// initialize problem list (original problem: root node)
	initializeProblemList(&tree, -1);
	// start timers
	time_t start, start2;
	start = clock();
	start2 = time(NULL);
	time_t nodeTimeStart, nodeTimeEnd;

	// read problem list size
	int treeListSize = tree.getListSize();
	treeLowerBound = tree.getLowerBound();

	IloBool intSolFound, firstIntSolFound;
	intSolFound = firstIntSolFound = IloFalse;
	IloInt pruneCounter = 0;

	// ��֦�����ķ�������
	IloNum branchedZval;
	// temporary store "coordinate" (f,ij) of fractional z vars found.
	IloIntArray indexOut(env, 2);
	indexOut[0] = -1;
	indexOut[1] = -1;

	// remember node number of best integer node
	// ���Žڵ�
	IloInt nodeNumberOfBestInteger = 0;
	// save the iteration count for the LAST improvement in best integer
	IloInt itNumFinalImprovement = 0;

	// keep track of node depth (in tree)
	// �ڵ������е�λ����Ϣ
	//IloInt currentDepth = 0;
	IloInt randomCounter = 0;

	// treat problemList[ACTIVE_PROBLEM] as long as the list contains subproblems
	while (treeListSize > 0) {
		nodeTimeStart = clock();
		itNum++;
		// assume feasibility
		rmpFeasible = IloTrue;
		// reset
		branchedZval = 0;
		indexOut[0] = -1;
		indexOut[1] = -1;

		if (tree.getListSize() == 0)
			break;
		// ѡ��active node��Ϊ��ǰҪ����node
		if (firstIntSolFound && intSolFound) {
			outFileAnalysis << endl << "LB NODE CHOSEN BY LOWEST... " << endl;
			//activeNode = tree.extractLowestLowerBoundNode();
			activeNode = tree.extractTopNode();
			// for this "good node", perform depth-first strategy until
			// another integer solution is found
			// This is usually default:
			intSolFound = IloFalse;
		}
		else {
			// extract topNode, making it our active node
			outFileAnalysis << endl << "LB NODE CHOSEN BY TOPNODE... " << endl;
			activeNode = tree.extractTopNode();
		}

		//currentDepth++;
		//// ����Ľڵ�
		//// update mostRecentNodeNumber
		//mostRecentNodeNumber = activeNode->nodeNumber;
		cout << "***********************************************Current node number:----------------- " << activeNode->nodeNumber << endl;


		// see if this node can even possibly containg a better integer solution
		// (pruning by bound: but bound from parent node)
		// �����֮ǰ�����½��жϣ��������Ŀǰ�Ͻ������ֱ�Ӽ�֦�������
		// �½翳֦���½����Ŀǰ���Ͻ���н⣩
		if (activeNode->lowerBound > TreeUpperBound)
		{
			env.out() << endl << "Upper (integer) bound less or equal to active problem's LP optimal value."
				<< endl << "--> Pruning node by bound." << endl;
			pruneCounter++;
			if (pruneCounter > PRUNE_REPEAT) {
				// also return (or continue) with "best-lb-node" if a node has
				// been pruned
				intSolFound = IloTrue;
			}
		}
		// if not - perform usual strategy	
		// �½粻�����Ͻ�ʱ����֦
		else
		{
			// ��ȡ��Ӧ�ĸ�ӵ�RMP�в����
			/*activeNode->extractSubCuts(&tempRmpCuts, z);*/
			activeNode->extractZTypeCuts(&tempRmpCuts, z, zType);
			masterModel.add(tempRmpCuts);
			// �����������Ƿ���еı�Ҫ�ԣ�����������������������������
			/*if (!masterSolver.solve()) {
				env.out() << endl << "Failed to optimize active problem: infeasible?" << endl;
				rmpFeasible = IloFalse;
			}*/

			// �Ӹ��RMP����ʱ�������������
			// if we solve problem (i.e. RMP feasible), extract objective value as current node lower bound
			if (rmpFeasible)
			{
				// �������������
				// generate new columns/variables and get lower bound on full MP
				// LP, and hence lower bound for all subnodes in this branch
				nodeLowerBound = ColumnGeneration(env, masterSolver, totalCost, capacityConstraints, demandConstraints,
					EqualZConstraints, EqualXijkConstraints, y, x, instance);
				if (activeNode->nodeNumber == 1)
					treeLowerBound = nodeLowerBound;

				masterSolver.getValues(zTypeValue, zType);

				for (int i = 0; i < z.getSize(); i++)
					masterSolver.getValues(zValue[i], z[i]);
				for (int i = 0; i < y.getSize(); i++)
					masterSolver.getValues(yValue[i], y[i]);
				for (int i = 0; i < xijk.getSize(); i++)
					masterSolver.getValues(xijkValue[i], xijk[i]);
				if (activeNode->nodeNumber == 1)
				{
					AddStrongValidInequality(env, instance, zValue, masterSolver, tempStrongCons, xijk, z, boolxij, true);
					// �����Υ����ǿ��Ч����ʽ
					if (tempStrongCons.getSize() > 0)
					{
						// ��ǲ���ʽԼ����k��ij
						// ������ԭ�У������޸Ķ�żֵ
						//IloIntArray2 kijIndex(env);
						//IloInt tempIndex = 0;
						//for (int a = 0; a < instance->n_commods; a++)
						//{
						//	kijIndex.add(IloIntArray(env));
						//	for (int b = 0; b < instance->n_arcs; b++)
						//	{
						//		if (boolxij[a][b])
						//		{
						//			kijIndex[a].add(tempIndex);
						//			tempIndex++;
						//		}
						//		else
						//			// û�м����cons��indexΪ�����Լ������
						//			kijIndex[a].add(tempStrongCons.getSize());
						//	}
						//}
						//masterModel.add(tempStrongCons);
						//treeLowerBound = InequalityCG(env, masterSolver, totalCost, capacityConstraints, demandConstraints,
						//	EqualZConstraints, EqualXijkConstraints, tempStrongCons, kijIndex, y, x, instance);
						//kijIndex.end();

						// ����ԭ��
						masterModel.add(tempStrongCons);
						masterSolver.solve();
						treeLowerBound = masterSolver.getObjValue();


						// ��������¹��ò���
						nodeLowerBound = treeLowerBound;
						cout << "Linear Master Model Objective Value Solved by Strong Inequalities(LB): "
							<< masterSolver.getObjValue() << endl;
						for (int i = 0; i < z.getSize(); i++)
							masterSolver.getValues(zValue[i], z[i]);
						for (int i = 0; i < y.getSize(); i++)
							masterSolver.getValues(yValue[i], y[i]);
						for (int i = 0; i < xijk.getSize(); i++)
							masterSolver.getValues(xijkValue[i], xijk[i]);
						tempStrongCons.clear();
					}

					// ��cutSet�����ģ��
					IloRangeArray tempCons(env);
					CutsetInequality cutset(env, masterSolver, instance, z);
					cutset.AddCutsetInequality(tempCons, z);
					if (tempCons.getSize() > 0)
					{
						masterModel.add(tempCons);
						masterSolver.solve();
						cout << "Linear Master Model Objective Value Solved by Cutsets Inequalities(LB): " << masterSolver.getObjValue() << endl;
						//masterModel.remove(tempCons);
					}
					tempCons.end();
					//system("pause");

				}
				else
				{
					//// ����Ŀǰ�����ǲ������������ţ��������ӽڵ���û��Ҫ�Ӳ���ʽ�����Ͻ�

					//AddStrongValidInequality(env, instance, zValue, masterSolver, tempStrongCons, xijk, z, boolxij, false);
					//if (tempStrongCons.getSize() > 0)
					//{
					//	masterModel.add(tempStrongCons);
					//	masterSolver.solve();
					//	// �Ǹ��ڵ���¸ýڵ��LB
					//	nodeLowerBound = masterSolver.getObjValue();
					//	cout << "Linear Master Model Objective Value Solved by Strong Inequalities(LB): "
					//		<< masterSolver.getObjValue() << endl;
					//	for (int i = 0; i < z.getSize(); i++)
					//		masterSolver.getValues(zValue[i], z[i]);
					//	for (int i = 0; i < y.getSize(); i++)
					//		masterSolver.getValues(yValue[i], y[i]);
					//	for (int i = 0; i < xijk.getSize(); i++)
					//		masterSolver.getValues(xijkValue[i], xijk[i]);
					//	// �Ǹ��ڵ�������Ƴ�����ʽ
					//	masterModel.remove(tempStrongCons);
					//	tempStrongCons.clear();
					//}
					//// ��cutSet�����ģ��
					//IloRangeArray tempCons(env);
					//CutsetInequality cutset(env, masterSolver, instance, z);
					//cutset.AddCutsetInequality(tempCons, z);
					//if (tempCons.getSize() > 0)
					//{
					//	masterModel.add(tempCons);
					//	masterSolver.solve();
					//	cout << "Linear Master Model Objective Value Solved by Cutsets Inequalities(LB): " << masterSolver.getObjValue() << endl;
					//	//masterModel.remove(tempCons);
					//}
					//tempCons.end();
					////system("pause");
				}

				// �����б����������Ч��
				IloIntArray tabuList(env);
				// �洢����ѭ���е������Ͻ�
				IloNum tempUB = INFINITY;
				for (int it = 0; it < 20; it++)
				{
					// CMCF������ʱ��δ�ҵ��Ľ��ⶼ����-1
					IloNum UB = LocalSearch(env, instance, xijkValue, zValue, tabuList);
					IloNum gap = (tempUB - UB) / UB;
					cout << "gap: " << gap << endl;
					// û�����ԸĽ�������ѭ��
					if (UB == -1 || IloRound(tempUB) == IloRound(UB))
						break;
					else if (UB < tempUB)
						tempUB = UB;
				}
				//system("pause");
				tabuList.end();
				nodeUpperBound = tempUB;
				// ���������Ͻ�
				if (nodeUpperBound < TreeUpperBound)
					TreeUpperBound = nodeUpperBound;

				// ����ýڵ���½�С�ڸ��ڵ���½磬�����½�ӦΪ���ڵ���½�
				if (nodeLowerBound < (activeNode->lowerBound))
					nodeLowerBound = (activeNode->lowerBound);
			}


			// I. Is solution feasible? if not, prune node by infeasibility
			// �������⣿����������������MP��������ֱ�Ӽ�֦
			//if (!masterSolver.isPrimalFeasible()) {
			if (!rmpFeasible){
				env.out() << endl << "No feasible solutions." << endl
					<< "--> Pruning node by infeasibility." << endl;
				pruneCounter++;
				if (pruneCounter > PRUNE_REPEAT) {
					intSolFound = IloTrue;
				}
			}
			// II. Is lower bound < upper bound (integer). If not, prune by bound
			// ����Ӧ����nodeUpperBoundt��treeLowerBound������������������������С�ڸ��ڵ���½磿��������������
			else if (nodeUpperBound <= nodeLowerBound) {
				//else if (nodeUpperBound <= treeLowerBound) {
				env.out() << endl << "Upper (integer) bound less or equal to active problem's LP optimal value."
					<< endl << "--> Pruning node by bound." << endl;
				pruneCounter++;
				if (pruneCounter > PRUNE_REPEAT) {
					intSolFound = IloTrue;
				}
			}
			// If none of the above, continue to branch of prune by optimality.			
			else {

				// reset prunecounter
				pruneCounter = 0;
				// ���ط���
				branchedZval = findFractionalZType(masterSolver, zTypeValue, indexOut);
				//branchedZval = findFractionalZ(masterSolver, zValue, indexOut);

				// solution integer valued and best so far
				// �ҵ������Ⲣ�ȵ�ǰ���н��
				//if ((indexOut[0] == -1) && (indexOut[1] == -1)
				//	&& (masterSolver.getValue(totalCost) < TreeUpperBound)) {
				// ��Ϊ
				if ((indexOut[0] == -1) && (indexOut[1] == -1) && nodeLowerBound < TreeUpperBound) {
					env.out() << endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
						<< endl << "Solution integer valued (no fractional z found)"
						<< endl << "--> Updating upper (integer) bound."
						<< endl << "--> Pruning by optimality." << endl;

					// ��z��������������z��֦
					/*branchedZval = findFractionalZ(masterSolver, zValue, indexOut);
					if (indexOut[0] != -1)
					{
						env.out() << endl << "No pruning possible."
							<< endl << "--> Branching binary on variable z[m][t] (m = " << indexOut[0] << ", t = "
							<< indexOut[1] << ")," << endl
							<< "    into two subproblems (z[m][t] = 0/1)." << endl;
						branchActiveProblem(&tree, activeNode, indexOut[0], indexOut[1], nodeLowerBound,
							branchedZval, IloTrue, treeOutput);
					}
					else*/
					{

						// update upper bound: best integer solution
						// ���ڲ��ܱ�֤�����⣬�������ﲻ�����Ͻ�
						//TreeUpperBound = nodeLowerBound;

						// make note that we have found an integer solution: we switch from "depth-first" node
						// picking strategy to "best-node-first" (node with lowest lower bound) strategy.
						// �ҵ�������֮����������Ȳ��Ա�Ϊ���Žڵ�����
						firstIntSolFound = IloTrue;
						// ���м�֦������������������
						//intSolFound = IloTrue;
						pruneCounter++;
						if (pruneCounter > PRUNE_REPEAT)
						{
							intSolFound = IloTrue;
						}

						randomCounter++;

						// update nodeNumberOfBestInteger
						nodeNumberOfBestInteger = activeNode->nodeNumber;
						// update iteration number for most recent improvement
						itNumFinalImprovement = itNum;
					}
				}
				// solution integer valued but worse than currently best
				// ������û�е�ǰ���н��
				else if ((indexOut[0] == -1) && (indexOut[1] == -1)) {

					// ��z��������������z��֦
					/*branchedZval = findFractionalZ(masterSolver, zValue, indexOut);
					if (indexOut[0] != -1)
					{
						env.out() << endl << "No pruning possible."
							<< endl << "--> Branching binary on variable z[m][t] (m = " << indexOut[0] << ", t = "
							<< indexOut[1] << ")," << endl
							<< "    into two subproblems (z[m][t] = 0/1)." << endl;
						branchActiveProblem(&tree, activeNode, indexOut[0], indexOut[1], nodeLowerBound,
							branchedZval, IloTrue, treeOutput);
					}
					else*/
					{

						env.out() << endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
							<< endl << "Solution integer valued (no fractional z found), but corresponding RMP"
							<< endl << "objective value worse than currently  best integer solution, i.e., upper bound."
							<< endl << "--> Pruning node by bound." << endl;
						pruneCounter++;
						if (pruneCounter > PRUNE_REPEAT) {
							intSolFound = IloTrue;
						}
					}
				}

				// IV. None of the above? Branch on fractional z var found in III.
				// �����м�֦������������֦
				else {
					env.out() << endl << "No pruning possible."
						<< endl << "--> Branching binary on variable z[m][t] (m = " << indexOut[0] << ", t = "
						<< indexOut[1] << ")," << endl
						<< "    into two subproblems (z[m][t] = 0/1)." << endl;
					// call branchActiveProblem to perform brancing.
					// ���������ӽڵ�����
					branchActiveProblem(&tree, activeNode, indexOut[0], indexOut[1], nodeLowerBound,
						branchedZval, IloTrue, treeOutput);
				}
			}

			// the active node has now been treated, so remove the cuts from it and thereafter
			//remove the problem from the problem list.
			// ��֦������ɺ��Ƴ���
			masterModel.remove(tempRmpCuts);
			tempRmpCuts.clear();

			// delete activeNode, it's been processed..
			delete activeNode;

			// update lower bound in remaining tree
			// �ж�Lower bound�Ƿ�Ϊ0��Ϊ0�򲻸���
			// ���ȫ������С�½�û�����壡����������������
			/*if (tree.getLowerBound() > 0)
				treeLowerBound = tree.getLowerBound();*/
			// update stop of node timer
			nodeTimeEnd = clock();
			// print current best integer solution value
			env.out() << endl << "Current lower bound = " << treeLowerBound << endl;
			env.out() << "Current upper bound = " << TreeUpperBound << endl;
			env.out() << "Gap = " << (TreeUpperBound - treeLowerBound) * 100 / treeLowerBound << endl;
			env.out() << "Nodes visited = " << itNum << endl;
			env.out() << endl << "Nodes visited Duration: " << (double)(nodeTimeEnd - nodeTimeStart) / CLOCKS_PER_SEC << endl;
			env.out() << "Nodes visited time " << (double)nodeTimeEnd / CLOCKS_PER_SEC << endl;
			// read problem list size
			treeListSize = tree.getListSize();
			env.out() << endl << "Current size of problem list = " << treeListSize << endl;
			env.out() << endl << "-----------------------------------------------------------------------" << endl;

			// termination criteria: does it possibly exist any better integer solutions in the tree?
			// ��ֹѭ���������½� > �Ͻ磿��������������
			if (treeLowerBound >= TreeUpperBound) {
				outFileAnalysis << endl << "TERMINATING: lower bound on remaining tree branches greater than currently best integer solution."
					<< endl;
				//cin.get();
				break;
			}
		}
	}
	// close problem list size output file
	outFileAnalysis.close();

	// end program timers
	time_t end, end2;
	end = clock();
	end2 = time(NULL);
	cout << " Iteration No.: " << itNum << endl;
	cout << "Time: " << (double)(end - start) / CLOCKS_PER_SEC << " s" << endl;
#pragma endregion Branch-and-bound



	// Clean memory
	for (int i = 0; i < y.getSize(); i++)
		y[i].end();
	for (int i = 0; i < x.getSize(); i++)
		x[i].end();
	y.end();
	x.end();
	tempRmpCuts.end();
	zTypeValue.end();
	for (int i = 0; i < zValue.getSize(); i++)
		zValue[i].end();
	zValue.end();
	for (int i = 0; i < yValue.getSize(); i++)
		yValue[i].end();
	yValue.end();
	for (int i = 0; i < xijkValue.getSize(); i++)
		xijkValue[i].end();
	xijkValue.end();
	tempStrongCons.end();
	capacityLowerBound.end();
	capacityUpperBound.end();
	capacityConstraints.end();
	demandConstraints.end();
}
