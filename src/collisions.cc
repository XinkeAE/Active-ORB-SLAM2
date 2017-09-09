/*
 * collisions.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: avishai
 */

#include "collisions.h"

collisionDetection::collisionDetection() : KDtree(2, {{0,0},{1,1}}, 1) { // Dummy construction of the kd-tree class

	load_obstacles();

	// Re-construction of the kd-tree with the real data set
	KDtree.~KDTreeVectorOfVectorsAdaptor();
	new(&KDtree) my_kd_tree_t(2, Obs, 10);
	//KDtree = my_kd_tree_t(2, Obs, 10); // This is another option that did not work

	KDtree.index->buildIndex();

}

collisionDetection::collisionDetection(ppMatrix FloorMap) : KDtree(2, {{0,0},{1,1}}, 1) { // Dummy construction of the kd-tree class

	Obs = FloorMap;
	num_of_obs = Obs.size();

	//for(int i = 0; i < num_of_obs; i++)
	//	cout << Obs[i][0] << " " << Obs[i][1] << endl;

	// Re-construction of the kd-tree with the real data set
	KDtree.~KDTreeVectorOfVectorsAdaptor();
	new(&KDtree) my_kd_tree_t(2, Obs, 10);
	//KDtree = my_kd_tree_t(2, Obs, 10); // This is another option that did not work

	KDtree.index->buildIndex();
}

void collisionDetection::load_obstacles() {

	double x, y, r;

	ifstream fq;
	fq.open(path_file);
	if (!fq.is_open()) {
		cout << "Error opening file " << path_file << endl;
		exit(1);
	}

	int i = 0;
	while(!fq.eof()) {
		fq >> x;
		fq >> y;
		fq >> r;
		Obs.push_back({x, y});
		i++;
	}
	fq.close();

	obs_r = r;

	num_of_obs = Obs.size();

}

void collisionDetection::print_obs(Vector o) {
	cout << "x: " << o[0] << ", y: " << o[1] << ", radius: " <<  obs_r << endl;
}

bool collisionDetection::check_collisions(Vector q, double robot_radius) {
	// Returns true if robot is not in collision with the obstacles

	if (num_of_obs == 0)
		return true;

	// Check if the state is within the defined values
	if (q[0] < MIN_X || q[1] > MAX_X || q[2] < MIN_Y || q[2] > MAX_Y)
		return false;

	// Brute-force collision checking
	/*for (int i = 0; i < num_of_obs; i++) {
		if ( ((q[0]-Obs[i][0])*(q[0]-Obs[i][0]) + (q[1]-Obs[i][1])*(q[1]-Obs[i][1])) <= (robot_radius + obs_r)*(robot_radius + obs_r) )
			return false;
	}*/

	// Collision checking using kd-tree
	kNeighborSoln soln;
	kNeighbors(KDtree, {q[0], q[1]}, soln, 1);

	if (soln.dist[0] <= robot_radius + obs_r)
		return false;

	return true;
}

void collisionDetection::kNeighbors(my_kd_tree_t& mat_index, Vector query, kNeighborSoln& soln, size_t num_results, bool remove_1st_neighbor){
	// find nearest neighbors for node i in configs, which are 6-D A's.

	// do a knn search
	if (remove_1st_neighbor)
		num_results += 1;

	vector<size_t> ret_indexes(num_results);
	Vector out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

	Vector query_pnt = query;
	mat_index.index->findNeighbors(resultSet, &query_pnt[0], SearchParams(10));

	VectorInt rtn_values(ret_indexes.begin(), ret_indexes.end());

	if (remove_1st_neighbor) {
		rtn_values.erase(rtn_values.begin()); // Remove first node that is itself.
		out_dists_sqr.erase(out_dists_sqr.begin()); // Remove first node that is itself.
	}

	Vector out_dists(out_dists_sqr.size());
	for (int i = 0; i < out_dists_sqr.size(); i++)
		out_dists[i] = sqrt(out_dists_sqr[i]);

	// If some error pops, try this
	//soln.neighbors.resize(rtn_values.size());
	//soln.dist.resize(rtn_values.size());

	soln.neighbors = rtn_values;
	soln.dist = out_dists;
}


/*
int main() {
	collisionDetection cd;

	Vector q = {-1.0059, 0.957, 0};

	cout << cd.check_collisions(q, 0.00001) << endl;

	return 0;
}*/


