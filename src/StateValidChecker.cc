/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */


#include "StateValidChecker.h"
#include <queue>

void StateValidChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

// ==========================================================================================================

void StateValidChecker::retrieveStateVector(const ob::State *state, Vector &q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	// Set state of rod
	for (unsigned i = 0; i < n; i++) {
		q[i] = Q->values[i];
	}
}

void StateValidChecker::updateStateVector(const ob::State *state, Vector q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < n; i++) {
		Q->values[i] = q[i];
	}
}

void StateValidChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	Vector q(n);

	for (unsigned i = 0; i < n; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}
	cout << "q: "; printVector(q);
}

bool StateValidChecker::checkMotionStraightLine(Vector q1, Vector q2) {
    
    Vector q(2);

    double d = normDistance(q1, q2, 2);
    double dd = 0.02;
    int nd = floor( d / dd );


    for (int i = 0; i <= nd; i++) {
		// Interpolate
		double s = (double)i / nd;
        q[0] = q1[0] * (1-s) +  s * q2[0];
        q[1] = q1[1] * (1-s) +  s * q2[1];

        if (!check_collisions(q, 0)) 
			return false;
    }

    return true;
}

// ==========================================================================================================

void StateValidChecker::setStartState(ob::State *st) {

	retrieveStateVector(st, StartState);

}

Vector StateValidChecker::getStartState() {

	return StartState;

}

// ==========================================================================================================

bool StateValidChecker::isValid(const ob::State *state) {
	Vector q(n);
	retrieveStateVector(state, q);

	return isValid(q);
}

bool StateValidChecker::isValid(Vector q) {

	if (!check_collisions(q, robot_r))
	{ 
		return false;
	}

	/** Check all constraints (collisions and number of features) */
	if (!heuristicValidityCheck) {
		
		return IsStateVisiblilty(q[0], q[1], q[2]);
	}
	/** Check number of features above a threshold only for states with distance maxDistHeuristicValidity
	 * from the start (only [x y] distance	 */
	else {
		
		if ( (q[0]-StartState[0])*(q[0]-StartState[0]) + (q[1]-StartState[1])*(q[1]-StartState[1]) < maxDistHeuristicValidity*maxDistHeuristicValidity ){
						return IsStateVisiblilty(q[0], q[1], q[2]);
		}
		else
			return true;
	}
}

bool StateValidChecker::checkMotion(const ob::State *s1, const ob::State *s2)
{
	bool result = true;
	int nd = stateSpace_->validSegmentCount(s1, s2);

	/* initialize the queue of test positions */
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		/* temporary storage for the checked state */
		ob::State *test = mysi_->allocState();

		/* repeatedly subdivide the path segment in the middle (and check the middle) */
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!isValid(test))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		mysi_->freeState(test);
	}

	return result;
}

// ========================== Reconstruct ===========================================================

bool StateValidChecker::reconstructMotion(const ob::State *s1, const ob::State *s2, ppMatrix &M) {

	int nd = stateSpace_->validSegmentCount(s1, s2);

	/* temporary storage for the checked state */
	ob::State *test = mysi_->allocState();

	Vector q(n);

	for (int i = 0; i <= nd; i++) {
		stateSpace_->interpolate(s1, s2, (double)i / (double)nd, test);
		retrieveStateVector(test, q);

		M.push_back(q);
	}

	return true;
}



// ========================== Misc ==================================================================

template <class T>
void StateValidChecker::printVector(T q) {

	cout << "[";
	for (int i = 0; i < q.size(); i++)
		cout << q[i] << " ";
	cout << "]" << endl;

}
void StateValidChecker::printMatrix(ppMatrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

double StateValidChecker::normDistance(Vector a1, Vector a2, int d) const {
	if (d==-1)
		d = a1.size();

	double sum = 0;
	for (int i=0; i < d; i++)
		sum += (a1[i]-a2[i])*(a1[i]-a2[i]);
	return sqrt(sum);
}

void StateValidChecker::log_path_file(ppMatrix M) {

	std::ofstream mf;

	mf.open("./paths/path.txt");

	//mf << M.size() << endl;

	for (int i = 0; i < M.size(); i++) {
		for (int j = 0; j < M[0].size(); j++)
			mf << M[i][j] << " ";
		mf << endl;
	}

	mf.close();
}

double StateValidChecker::fix_rot_angle(double q) {

	if (q < -PI)
		q += 2*PI;
	if (q > PI)
		q -= 2*PI;

	return q;
}

// ========================== Two-wheels motion ===========================================================

bool StateValidChecker::checkMotionTW(const ob::State *s1, const ob::State *s2) {
	// We assume that s1 is in the tree and therefore valid

	Vector q1(n), q2(n), q(n), q_temp(n);
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);

	TW_path_data vwt = GetShortestPath(q1, q2);
	if (!vwt.valid) // Add move straight to goal if the goal is sampled?
		return false;

	ppMatrix Q;
	Q.push_back(q1);

	for (int i = 0; i < vwt.v.size(); i++) {
		int m = 1+ceil(vwt.t[i]/dt); // dt + j*dt ?
		double dd = vwt.t[i] / (m-1);
		q = Q.back();
		for (int j = 1; j < m; j++) { // starts from 1 because the first point was already inserted to Q
			q_temp = myprop(q, vwt.v[i], vwt.w[i], j*dd);
			if (!isValid(q_temp))
				return false;
			Q.push_back(q_temp);
		}
	}

	return true;
}

bool StateValidChecker::reconstructMotionTW(const ob::State *s1, const ob::State *s2, ppMatrix &Q) {

	Vector q1(n), q2(n), q(n);
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);

	TW_path_data vwt = GetShortestPath(q1, q2);
	if (!vwt.valid)
		return false;

	Q.push_back(q1);

	for (int i = 0; i < vwt.v.size(); i++) {
		int m = 1+ceil(vwt.t[i]/dt); // dt + j*dt ?
		double dd = vwt.t[i] / (m-1);
		q = Q.back();
		for (int j = 1; j < m; j++) // starts from 1 because the first point was already inserted to Q
			Q.push_back(myprop(q, vwt.v[i], vwt.w[i], j*dd));
	}

	return true;
}

Vector StateValidChecker::myprop(Vector q, double vi, double wi, double ti) const {
	double h = q[2] + wi * ti;
	double x, y;

	if (wi) {
		x = q[0] + ((vi/wi)*(sin(h)-sin(q[2])));
		y = q[1] - ((vi/wi)*(cos(h)-cos(q[2])));
	}
	else {
		x = q[0] + ti * vi * cos(h);
		y = q[1] + ti * vi * sin(h);
	}

	return {x, y, h};

}

TW_path_data StateValidChecker::GetShortestPath(Vector q1, Vector q2) const {

	//   start and goal configurations:
	//       q1 = [x;y;theta]
	//       q2 = [x;y;theta]
	//
	//   minimum turning radius:
	//       turn_radius
	//
	//   shortest path (all 1x3 matrices):
	//       v is forward speed (normalized - always 1 or -1)
	//       w is turning rate
	//       t is time interval (i.e., apply v(i) and w(i) for time t(i))
	//
	//   if no path is found, [v,w,t] will all be empty sets
	//
	//   note that the total length of the shortest path is sum(t)

	TW_path_data vwt;

	double d1 = 1e9, dmid = 1e9, d2 = 1e9;
	short s1, s2, dir;
	bool sol = false;

	for (int s1cur = -1; s1cur < 2; s1cur+=2)
		for (int s2cur = -1; s2cur < 2; s2cur+=2) {
			for (int dircur = -1; dircur < 2; dircur+=2) {
				TW_tangent twt = twb_shortpath(q1, s1cur, q2, s2cur, dircur);
				if (twt.valid) {
					if (twt.d1mid2[0]+twt.d1mid2[1]+twt.d1mid2[2] < d1+dmid+d2) {
						sol = true;

						vwt.t = twt.d1mid2;
						d1 = vwt.t[0]; dmid = vwt.t[1]; d2 = vwt.t[2];
						s1 = s1cur;
						s2 = s2cur;
						dir = dircur;
					}
				}
			}
		}

	if (!sol) {
		vwt.valid = false;
		return vwt;
	}

	//v = {dir, dir, dir};
	//w = {s1*v[0]/turn_radius, 0, s2*v[2]/turn_radius};

	vwt.valid = true;
	vwt.v = {dir, dir, dir};
	vwt.w = {s1*vwt.v[0]/turn_radius, 0, s2*vwt.v[2]/turn_radius};

	return vwt;
}

TW_tangent StateValidChecker::twb_shortpath(Vector q1, int s1, Vector q2, int s2, int dir) const {

	//   q1,q2 are initial and final points
	//   s1,s2 are -1 for wheels to left, +1 for wheels to right
	//   dir is 1 for forward and -1 for backward
	//   r is turning radius
	//
	//   d1,dmid,d2 are lengths of each part of the curve in R^2
	//   qt1,qt2 are intermediate points
	//
	//   NOTE: particular combinations of dir, s1, s2 are sometimes infeasible.
	//   If this is the case, false is returned.

	Vector qc1(2), qc2(2);
	TW_tangent twt;

	double h1 = q1[2];
	double h2 = q2[2];
	qc1[0] = q1[0] - turn_radius * s1 * sin(h1);
	qc1[1] = q1[1] + turn_radius * s1 * cos(h1);
	qc2[0] = q2[0] - turn_radius * s2 * sin(h2);
	qc2[1] = q2[1] + turn_radius * s2 * cos(h2);

	if (s1*s2<0 && normDistance(qc1, qc2) < 2*turn_radius) {
		twt.valid = false;
		return twt;
	}

	twt = twb_gettangent(qc1, dir*s1, qc2, dir*s2);

	double delta1 = twb_getangle(qc1,twt.qt1) - twb_getangle(qc1,q1);
	if ( dir*s1 > 0 && delta1 < 0 )
		delta1 += 2*PI;
	else if ( dir*s1 < 0 && delta1 > 0 )
		delta1 -= 2*PI;

	double delta2 = twb_getangle(qc2,q2)-twb_getangle(qc2,twt.qt2);
	if ( dir*s2 > 0 && delta2 < 0 )
		delta2 += 2*PI;
	else if ( dir*s2 < 0 && delta2 > 0 )
		delta2 -= 2*PI;

	twt.d1mid2 = {fabs(turn_radius*delta1), normDistance(twt.qt1, twt.qt2, 2), fabs(turn_radius*delta2)};
	twt.qt1[2] = h1 + delta1;
	twt.qt2[2] = h2 - delta2;

	twt.valid = true;
	return twt;
}

TW_tangent StateValidChecker::twb_gettangent(Vector q1, int s1, Vector q2, int s2) const {
	// updates qt1 and qt2

	// q1,q2 are the centers of two circles, both with radius r
	// s1,s2 are the orientations (1 or -1) of the two circles
	//		(1=cc-wise, -1=c-wise)
	//
	// t1,t2 are the endpoints of a segment that is tangent to
	//		both circles and that matches the orientations s1,s2

	TW_tangent twt;
	twt.qt1.resize(2);
	twt.qt2.resize(2);

	if ( s1==1 && s2==1 ) {
		// Vector u from center 1 to center 2
		double ux = q2[0] - q1[0];
		double uy = q2[1] - q1[1];
		double norm_u = sqrt(ux*ux + uy*uy);

		// Vector perpendicular to u (turning clockwise) of length r
		//double uperp_x = turn_radius * uy / norm_u;
		//double uperp_y = -turn_radius * ux / norm_u;

		// Tangent points
		twt.qt1[0] = q1[0] + turn_radius * uy / norm_u;
		twt.qt1[1] = q1[1] - turn_radius * ux / norm_u;
		twt.qt2[0] = q2[0] + turn_radius * uy / norm_u;
		twt.qt2[1] = q2[1] - turn_radius * ux / norm_u;
	}
	else if ( s1==-1 && s2==-1 ) {
		// Vector u from center 1 to center 2
		double ux = q2[0] - q1[0];
		double uy = q2[1] - q1[1];
		double norm_u = sqrt(ux*ux + uy*uy);

		// Vector perpendicular to u (turning counter-clockwise) of length r
		//double uperp_x = -turn_radius * uy / norm_u;
		//double uperp_y = turn_radius * ux / norm_u;

		// Tangent points
		twt.qt1[0] = q1[0] - turn_radius * uy / norm_u;
		twt.qt1[1] = q1[1] + turn_radius * ux / norm_u;
		twt.qt2[0] = q2[0] - turn_radius * uy / norm_u;
		twt.qt2[1] = q2[1] + turn_radius * ux / norm_u;
	}
	else if ( s1==1 && s2==-1 ) {
		// Vector to midpoint of segment from center 1 to center 2
		double qmid_x = (q2[0]-q1[0])/2; // !!! Should it be (+) instead of (-) ???
		double qmid_y = (q2[1]-q1[1])/2;

		// Vector to tangent points in local coordinates
		double L = sqrt(qmid_x*qmid_x + qmid_y*qmid_y);
		double x = (turn_radius*turn_radius)/L;
		double y = (turn_radius/L)*sqrt((L*L)-(turn_radius*turn_radius));

		// Unit vector in the direction of qmid, and its perpendicular
		qmid_x /= L;
		qmid_y /= L;

		// Tangent points
		twt.qt1[0] = q1[0] + x * qmid_x + y * qmid_y;
		twt.qt1[1] = q1[1] + x * qmid_y - y * qmid_x;
		twt.qt2[0] = q2[0] - x * qmid_x - y * qmid_y;
		twt.qt2[1] = q2[1] - x * qmid_y + y * qmid_x;
	}
	else if ( s1==-1 && s2==1 ) {
		// Vector to midpoint of segment from center 1 to center 2
		double qmid_x = (q2[0]-q1[0])/2; // !!! Should it be (+) instead of (-) ???
		double qmid_y = (q2[1]-q1[1])/2;

		// Vector to tangent points in local coordinates
		double L = sqrt(qmid_x*qmid_x + qmid_y*qmid_y);
		double x = (turn_radius*turn_radius)/L;
		double y = (turn_radius/L)*sqrt((L*L)-(turn_radius*turn_radius));

		// Unit vector in the direction of qmid, and its perpendicular
		qmid_x /= L;
		qmid_y /= L;

		// Tangent points
		twt.qt1[0] = q1[0] + x * qmid_x - y * qmid_y;
		twt.qt1[1] = q1[1] + x * qmid_y + y * qmid_x;
		twt.qt2[0] = q2[0] - x * qmid_x + y * qmid_y;
		twt.qt2[1] = q2[1] - x * qmid_y - y * qmid_x;
	}
	else {
		printf("Error: twb_gettangent was passed s1=%d, s2=%d",s1,s2);
		exit(1);
	}

	return twt;
}

double StateValidChecker::twb_getangle(Vector q1, Vector q2) const {

	return atan2(q2[1]-q1[1],q2[0]-q1[0]);

}

// ============================== Optimization functions ===================================

double StateValidChecker::MotionCostLength(ppMatrix Q) const {

	double C = 0;
	for (int i = 1; i < Q.size(); i++)
		C += normDistance(Q[i-1], Q[i]);

	return C;
}

double StateValidChecker::MotionCostCamera(ppMatrix Q) const {

	double C = 0;
	for (int i = 1; i < Q.size()-1; i++) {
		//cout << Q[i][0] << " " << Q[i][1] << " " << Q[i][2] << endl;

		int c = countVisible(Q[i][0], Q[i][1], Q[i][2]);
		C += c < feature_threshold ? 1./1e-4 : 1./(double)c;
	}
	return C;
}

double StateValidChecker::MotionCost(const ob::State *s1, const ob::State *s2, const int cost_type) const {
	// cost_type: 1 - motion length, 2 - number of visible features

	Vector q1(n), q2(n), q(n);

	const ob::RealVectorStateSpace::StateType *Q1 = s1->as<ob::RealVectorStateSpace::StateType>();
	// Set state of rod
	for (unsigned i = 0; i < n; i++) {
		q1[i] = Q1->values[i];
	}
	const ob::RealVectorStateSpace::StateType *Q2 = s2->as<ob::RealVectorStateSpace::StateType>();
	// Set state of rod
	for (unsigned i = 0; i < n; i++) {
		q2[i] = Q2->values[i];
	}

	TW_path_data vwt = GetShortestPath(q1, q2);

	ppMatrix Q;
	Q.push_back(q1);

	for (int i = 0; i < vwt.v.size(); i++) {
		int m = 1+ceil(vwt.t[i]/dt); // dt + j*dt ?
		double dd = vwt.t[i] / (m-1);
		q = Q.back();
		for (int j = 1; j < m; j++) // starts from 1 because the first point was already inserted to Q
			Q.push_back(myprop(q, vwt.v[i], vwt.w[i], j*dd));
	}

	switch (cost_type) {
	case 1:
		return MotionCostLength(Q);
	case 2:
		return MotionCostCamera(Q);
	}
}






