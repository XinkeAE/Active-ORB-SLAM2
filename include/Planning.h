#ifndef PLANNING_H
#define PLANNING_H

#include <mutex>
#include <thread>
#include <vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;

class Planning {
public:
    Planning(cv::Mat goal_pose);

    // Run the planner.
    void Run();

    // Get all visible map points associated with the given pose.
    std::vector<MapPoint*> GetVisibleMapPoints(cv::Mat pose);
	// Request planning thread to stop.
	void RequestFinish();

private:
    // Get closest KeyFrame to the given pose.
    int GetClosestKeyFrameId(cv::Mat pose);

	// Check if the thread should stop.
	bool CheckFinish();
	void SetFinish();
	bool isFinished();
    bool mbFinishRequested;
    bool mbFinished;
	bool mbStopped;
	std::mutex mMutexFinish;
	std::mutex mMutexStop;
};

}  // namespace ORB_SLAM

#endif // PLANNING_H
