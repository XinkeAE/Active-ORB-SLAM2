/*
 * OptimizationObj.h
 *
 *  Created on: Aug 22, 2017
 *      Author: avishai
 */

#ifndef VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_
#define VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_

#include <vector>
#include "ompl/base/Cost.h"
#include "camera.h"

#define MAP_FILE "./data/Corner_Map_S.txt"
#define UB_FILE "./data/upper_bound.txt"
#define LB_FILE "./data/lower_bound.txt"

typedef std::vector<std::vector< double > > ppMatrix;
typedef std::vector< double > Vector;

using namespace ompl::base;
using namespace std;

class OptimizationObj : public camera
{
public:
	//OptimizationObj() : camera(MAP_FILE, UB_FILE, LB_FILE) {}
	OptimizationObj() : camera() {}
	Cost motionCost(ppMatrix);

	Cost pathLengthObjective(ppMatrix);

	/** \brief Check whether the the cost \e c1 is considered better than the cost \e c2. By default, this returns true if if c1 is less than c2. */
	bool isCostBetterThan(Cost c1, Cost c2);

	/** \brief Get a cost which is greater than all other costs in this OptimizationObjective; required for use in Dijkstra/Astar. Defaults to returning the double value inf.*/
	Cost infiniteCost();

	/** \brief Return the minimum cost given \e c1 and \e c2. Uses isCostBetterThan. */
	Cost betterCost(Cost c1, Cost c2);

	/** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2. Default implementation defines this combination as an addition. */
	Cost combineCosts(Cost c1, Cost c2);

	/** \brief Get the value of the cost */
	Cost get_mCost() {
		return mCost;
	}
private:
	/** \brief Local connection cost between two states */
	Cost mCost;

	double normD(Vector, Vector);

};




#endif /* VALIDITY_CHECKERS_OPTIMIZATIONOBJ_H_ */
