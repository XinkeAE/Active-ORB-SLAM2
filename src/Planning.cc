#include "Planning.h"

namespace ORB_SLAM2 {
    Planning::Planning(cv::Mat goal_pose) {}
    void Planning::Run() {}
    std::vector<MapPoint*> Planning::GetVisibleMapPoints(cv::Mat pose) {
        return {};
    }
    int Planning::GetClosestKeyFrameId(cv::Mat pose) {
        return 0;
    }
}
