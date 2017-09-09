/*
 * myOptimization.h
 *
 *  Created on: Aug 22, 2017
 *      Author: avishai
 */

#ifndef VALIDITY_CHECKERS_MYOPTIMIZATION_H_
#define VALIDITY_CHECKERS_MYOPTIMIZATION_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../include/StateValidChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

namespace ob = ompl::base;
using namespace ob;

/** Defines an optimization objective which attempts to minimize the length of the path in the R^2 X S configuration space
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class LengthObjective : public ob::StateCostIntegralObjective, public StateValidChecker
{
public:
    LengthObjective(const ob::SpaceInformationPtr& si, map_data MD, ppMatrix FloorMap) : ob::StateCostIntegralObjective(si, true), StateValidChecker(si, MD, FloorMap) {cout << "Init custom objective function.\n";}
    ob::Cost stateCost(const ob::State* s) const
    {
      	return identityCost();
    }

    ob::Cost motionCost(const State *s1, const State *s2) const
    {
    	return ob::Cost(MotionCost(s1, s2));
    }

    ob::Cost motionCostHeuristic(const State *s1, const State *s2) const
    {
    	return motionCost(s1, s2);
    }
};


/** Defines an optimization objective which attempts to maximize the features seen by the camera
 *
    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class CameraObjective : public ob::StateCostIntegralObjective, public StateValidChecker
{
public:
    CameraObjective(const ob::SpaceInformationPtr& si, map_data MD, ppMatrix FloorMap) : ob::StateCostIntegralObjective(si, true), StateValidChecker(si, MD, FloorMap) {cout << "Init custom objective function.\n";}
    
    ob::Cost stateCost(const ob::State* s) const
    {
    	return identityCost();
    }

    ob::Cost motionCost(const State *s1, const State *s2) const
    {
    	return ob::Cost(MotionCost(s1, s2, 2));
    }

    ob::Cost motionCostHeuristic(const State *s1, const State *s2) const
    {
    	return motionCost(s1, s2);
    }
};


#endif /* VALIDITY_CHECKERS_MYOPTIMIZATION_H_ */
