#ifndef OCTOMAPBUILDER_H
#define OCTOMAPBUILDER_H

#include <deque>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Map.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Converter.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;
class Map;

class OctomapBuilder {
public:
    OctomapBuilder();

    // Run the planner.
    void Run();
    // Send request from Planning thread to Octomap thread.
    void SendGetOctomapRequest();
    // Request planning thread to stop.
    void RequestFinish();
    // TODO: Update Octomap.
    void UpdateOctomap(KeyFrame* pKF);

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

    // Octomap
    octomap::OcTree* globalOctoMap;

    // Check if tracking thread sends a request, which contains a KeyFrame and a
    // current pose.
    bool CheckHasRequest();
    bool hasRequest;
    void AckRequest();
    cv::Mat currPose;
    cv::Mat depth;
    bool hasUpdate;
    std::mutex mMutexRequest;
    std::mutex mMutexUpdate;
};

}  // namespace ORB_SLAM

#endif // OCTOMAPBUILDER_H
