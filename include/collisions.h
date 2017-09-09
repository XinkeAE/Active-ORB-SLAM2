/*
 * collisions.h
 *
 *  Created on: Aug 18, 2017
 *      Author: avishai
 */

#ifndef VALIDITYCHECK_COLLISIONS_H_
#define VALIDITYCHECK_COLLISIONS_H_


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// For nn-search
#include "./nanoflann.hpp"
#include "./KDTreeVectorOfVectorsAdaptor.h"

typedef std::vector<std::vector< double > > ppMatrix;
typedef std::vector< double > Vector;
typedef std::vector< int > VectorInt;
typedef KDTreeVectorOfVectorsAdaptor< ppMatrix, double > my_kd_tree_t;

#define MIN_X -1.15
#define MAX_X 6.5
#define MIN_Y -5
#define MAX_Y 5

using namespace std;
using namespace nanoflann;

struct kNeighborSoln {
	VectorInt neighbors;
	Vector dist;
};

class collisionDetection
{
public:
	collisionDetection(ppMatrix);

	collisionDetection();

	void load_obstacles();

	bool check_collisions(Vector, double);

	void print_obs(Vector o);

	void kNeighbors(my_kd_tree_t&, Vector, kNeighborSoln&, size_t, bool = false);


private:
	ppMatrix Obs;
	double obs_r = 0.1; // Assumed point obstacle radius

	string path_file = "./data/obs.txt";

	int num_of_obs;

	my_kd_tree_t KDtree;
};

#endif /* VALIDITYCHECK_COLLISIONS_H_ */
