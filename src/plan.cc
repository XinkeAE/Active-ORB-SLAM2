/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

ob::PlannerPtr plan_slam::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si, MD, FloorMap, feature_thres);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si, MD, FloorMap, feature_thres);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr plan_slam::getPathLengthObjective(const ob::SpaceInformationPtr& si)
{

    OMPL_INFORM("Loading path length optimization objective.");

    return ob::OptimizationObjectivePtr(new LengthObjective(si, MD, FloorMap));
}

/** Return an optimization objective which attempts to minimiaze turn angle. */
ob::OptimizationObjectivePtr plan_slam::getMyObjective(const ob::SpaceInformationPtr& si)
{
    OMPL_INFORM("Loading custom optimization objective.");

    return ob::OptimizationObjectivePtr(new CameraObjective(si, MD, FloorMap));
}

/** Create an optimization objective equivalent to the one returned by
    getBalancedObjective1(), but use an alternate syntax. */
ob::OptimizationObjectivePtr plan_slam::getWeightedObjective(const ob::SpaceInformationPtr& si)
{
    OMPL_INFORM("Loading multi-objective optimization.");

    ob::OptimizationObjectivePtr lengthObj(new LengthObjective(si, MD, FloorMap));
    ob::OptimizationObjectivePtr customObj(new CameraObjective(si, MD, FloorMap));

    return 1.0*lengthObj + customObj;
}

ob::OptimizationObjectivePtr plan_slam::allocateObjective(ob::SpaceInformationPtr si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_MINE:
            return getMyObjective(si);
            break;
        case OBJECTIVE_WEIGHT:
        	return getWeightedObjective(si);
        	break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

void plan_slam::getPath(ob::ProblemDefinitionPtr pdef, ppMatrix &M) {

	og::PathGeometric Path( dynamic_cast< const og::PathGeometric& >( *pdef->getSolutionPath()));
	const std::vector< ob::State* > &states = Path.getStates();
	ob::State *state;
	Vector q(3);
	for( size_t i = 0 ; i < states.size( ) ; ++i ) {
		state = states[i]->as< ob::State >();
		q[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
		q[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
		q[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
		M.push_back(q);
	}
}

void plan_slam::UpdateMap(ppMatrix Map, Vector upper_bound, Vector lower_bound, Vector max_dist, Vector min_dist, Vector foundRatio) {

	MD.Map = Map;
	MD.UB = upper_bound;
	MD.LB = lower_bound;

	MD.maxDist = max_dist;
	MD.minDist = min_dist;

	MD.foundRatio = foundRatio;
}

int plan_slam::AdvanceStepCamera(ppMatrix M, int thres) {

	StateValidChecker svc(MD, FloorMap, thres);

	int i = 0;
	while (i < M.size() && svc.IsStateVisiblilty(M[i][0],M[i][1],M[i][2],thres))
		i++;
	i--;

	return i;
}

void plan_slam::set_FloorMap(ppMatrix Map) {
	FloorMap = Map;
}


void plan_slam::set_featureThreshold(int num) {
	feature_thres = num;
}

bool plan_slam::isApproximate() {
	if (solved ==  ob::PlannerStatus::APPROXIMATE_SOLUTION)
		return true;
	if (solved ==  ob::PlannerStatus::EXACT_SOLUTION)
		return false;

	return true;
}

bool plan_slam::plan(Vector q_start, Vector q_goal, double runtime, plannerType p_type, planningObjective o_type) {

	// construct the state space we are planning in
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(n)); // A-space - state space of the rod - R^6

	// set the bounds for the A=R^6
	ob::RealVectorBounds Qbounds(n);
	Qbounds.setLow(0, MIN_X); // x_min
	Qbounds.setHigh(0, MAX_X); // x_max
	Qbounds.setLow(1, MIN_Y); // y_min
	Qbounds.setHigh(1, MAX_Y); // y_max
	Qbounds.setLow(2, -PI); // theta_min
	Qbounds.setHigh(2, PI); // theta_max

	// set the bound for the space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Qspace(Q);

	 // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Qspace));

	 // set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.03); // 3% ???

	// create a random start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Qspace);
	for (int i = 0; i < n; i++)
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_start[i];

	 // create a random goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Qspace);
	for (int i = 0; i < n; i++)
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = q_goal[i];

	 // create a problem instance
	 ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	 // set the start and goal states
	 pdef->setStartAndGoalStates(start, goal);

	 // If this is an optimizing planner, set the optimization objective
	 if (p_type==PLANNER_RRTSTAR) {
		 // Create the optimization objective specified by our command-line argument.
		 // This helper function is simply a switch statement.
		 pdef->setOptimizationObjective(allocateObjective(si, o_type));
	 }

	 // create a planner for the defined space
	 // To add a planner, the #include library must be added above
	 ob::PlannerPtr planner = allocatePlanner(si, p_type);

	 // set the problem we are trying to solve for the planner
	 planner->setProblemDefinition(pdef);

	 // perform setup steps for the planner
	 planner->setup();

	 //planner->printSettings(std::cout); // Prints some parameters such as range
	 //planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	 // print the settings for this space
	 si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	 //si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	 // print the problem settings
	 //pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	 // attempt to solve the problem within one second of planning time
	 clock_t st = clock();
	 solved = planner->solve(runtime);
	 double Ttime = double(clock() - st) / CLOCKS_PER_SEC;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution in " << Ttime << " seconds." << std::endl;

		Path.clear();
		getPath(pdef, Path);
		//StateValidityChecker svc;
		//svc.printMatrix(Path);

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("./paths/path.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();
	}
	 else
	 std::cout << "No solution found" << std::endl;
}