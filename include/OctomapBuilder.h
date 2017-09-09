#ifndef OCTOMAPBUILDER_H
#define OCTOMAPBUILDER_H

#include <condition_variable>
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

#include <Converter.h>

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
    void UpdateOctomap(const cv::Mat &depth, cv::Mat currPose);

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

    // Check if tracking thread sends a update, which contains a depth and a
    // current pose.
    bool CheckHasRequest();
    bool CheckHasUpdate();
    bool hasRequest;
    void AckRequest();
    cv::Mat currPose;
    cv::Mat depth;
    float depthFactor;
    bool hasUpdate;
    std::mutex mMutexRequest;
    std::mutex mMutexUpdate;
    std::condition_variable cvUpdate;

    // camera parameters
    float camera_fx;
    float camera_fy;
    float camera_cx;
    float camera_cy;
    cv::Mat T_bc;
    cv::Mat T_wc_mat;    
};

}  // namespace ORB_SLAM

#endif // OCTOMAPBUILDER_H
