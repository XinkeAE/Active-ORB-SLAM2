#ifndef PLANNING_H
#define PLANNING_H

#include <deque>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Map.h"

#include <opencv2/opencv.hpp>

#include "../include/plan.h"
#include "../include/StateValidChecker.h"

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;
class Map;

class Planning {
public:
    Planning(cv::Mat goal_pose, Map* pMap);

    // Run the planner.
    void Run();
	// Send request from Tracking thread to planning thread.
    void SendPlanningRequest(cv::Mat pose, KeyFrame* kf);
	// Request planning thread to stop.
	void RequestFinish();
	// Insert KeyFrame into the queue.
	void InsertKeyFrame(KeyFrame* pKF);
	// Get all possibly visible MapPoints for a given pose.
	std::set<MapPoint*> GetVisiblePoints(cv::Mat pose);

private:
	// Check if the thread should stop.
	bool CheckFinish();
	void SetFinish();
	bool isFinished();
    bool mbFinishRequested;
    bool mbFinished;
	bool mbStopped;
	std::mutex mMutexFinish;
	std::mutex mMutexStop;

    // Map
	Map* mpMap;
	
	// Start and goal
	std::vector<double> q_start;
	std::vector<double> q_goal;

	// Check if tracking thread sends a request, which contains a KeyFrame and a
	// current pose.
	bool CheckHasRequest();
	bool hasRequest;
	KeyFrame* currKF;
	cv::Mat currPose;
	std::mutex mMutexRequest;

	// Store the planned trajectory.
	std::vector<std::vector<double>> planned_trajectory;

	// Keyframe queue for planning (Not used for now).
	std::deque<KeyFrame*> mKeyFrameQueue;
	std::mutex mMutexKeyFrameQueue;
	int GetNumNewKeyFrames();
	KeyFrame* PopKeyFrameQueue(int num_pop);
};

}  // namespace ORB_SLAM

#endif // PLANNING_H
