#ifndef PLANNING_H
#define PLANNING_H

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

private:
    // Get closest KeyFrame to the given pose.
    int GetClosestKeyFrameId(cv::Mat pose);

};

}  // namespace ORB_SLAM

#endif // PLANNING_H
