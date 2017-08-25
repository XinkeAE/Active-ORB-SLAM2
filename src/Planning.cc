#include "Planning.h"

namespace ORB_SLAM2 {
Planning::Planning(cv::Mat goal_pose) {}

void Planning::Run() {
    while (1) {
        // Process request.

    }
}

std::vector<MapPoint*> Planning::GetVisibleMapPoints(cv::Mat pose) {
    return {};
}

void Planning::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

int Planning::GetClosestKeyFrameId(cv::Mat pose) {
    return 0;
}

bool Planning::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Planning::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool Planning::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}  // namespace ORB_SLAM
