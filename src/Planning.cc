#include "Planning.h"

namespace ORB_SLAM2 {
    Planning::Planning(cv::Mat goal_pose) {}
    Planning::Planning(cv::Mat goal_pose, Map* pMap_){
        mpMap = pMap;
    }
    void Planning::Run() {}
    std::vector<MapPoint*> Planning::GetVisibleMapPoints(cv::Mat pose) {
        return {};
    }
    int Planning::GetClosestKeyFrameId(cv::Mat pose) {
        return 0;
    }
}
