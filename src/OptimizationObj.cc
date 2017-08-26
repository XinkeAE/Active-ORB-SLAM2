/*
 * OptimizationObj.cpp
 *
 *  Created on: Aug 22, 2017
 *      Author: avishai
 */

#include "OptimizationObj.h"

ob::Cost OptimizationObj::motionCost(ppMatrix Q) {

	return pathLengthObjective(Q);

}

ob::Cost OptimizationObj::pathLengthObjective(ppMatrix Q) {

	ob::Cost C(0.0);
	for (int i = 1; i < Q.size(); i++)
		C.value() += normD(Q[i-1], Q[i]);

	return C;
}

bool OptimizationObj::isCostBetterThan(Cost c1, Cost c2)
{
	return c1.value() < c2.value();
}

Cost infiniteCost()
{
	return Cost(std::numeric_limits<double>::infinity());
}

Cost OptimizationObj::betterCost(Cost c1, Cost c2)
{
	return isCostBetterThan(c1, c2) ? c1 : c2;
}

Cost OptimizationObj::combineCosts(Cost c1, Cost c2)
{
    return Cost(c1.value() + c2.value());
}

double OptimizationObj::normD(Vector q1, Vector q2) {

	double sum = 0;
	for (int i=0; i < q1.size(); i++)
		sum += (q1[i]-q2[i])*(q1[i]-q2[i]);
	return sqrt(sum);
}






