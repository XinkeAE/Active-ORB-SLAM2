#include "Planning.h"

#include <cmath>

namespace ORB_SLAM2 {

Planning::Planning(cv::Mat goal_pose, Map* pMap){
    mpMap = pMap;
    hasRequest = false;
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    p_type = PLANNER_RRTSTAR;
    o_type = OBJECTIVE_PATHLENGTH;

    q_start = {0, 0, 0};
    q_goal = {5.584, -2.0431, -1.5707};
    
    pl = new plan_slam();
  
}

void Planning::Run() {
    while (1) {
        // Run planner when Tracking thread send request.
        if (CheckHasRequest()) {
            // Call Planner with currKF and currPose.
            cout << "in planning loop" << endl;
            planningMap.clear();
            UB.clear();
            LB.clear();

            // update map here
            // 1. access to the map
            // 2. get the map points 
            // 3. compute the Upper bound, the lower bound
            {
                unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
                vector<MapPoint*> vpPts = mpMap->GetAllMapPoints();
                cout << "totally " << vpPts.size() << " points." << endl;
                for(size_t i=0; i<vpPts.size(); i++){
                    if(vpPts[i]->isBad())
                        continue;
                    
                    planningMap.push_back(std::vector<double>{vpPts[i]->GetWorldPos().at<float>(0),
                                                             vpPts[i]->GetWorldPos().at<float>(1),
                                                             vpPts[i]->GetWorldPos().at<float>(2)});

                    if(vpPts[i]->theta_std * 3 < 15.0/57.3){
                        theta_interval = 15.0/57.3;
                    }else{
                        theta_interval = vpPts[i]->theta_std * 3;
                    }

                    UB.push_back(double(vpPts[i]->theta_mean + theta_interval));
                    LB.push_back(double(vpPts[i]->theta_mean - theta_interval));
                }
            }

            std::cout << planningMap.size() << std::endl;

            pl->UpdateMap(planningMap, UB, LB);

            // do actual planning
            pl->plan(q_start, q_goal, 2, p_type, o_type);

            // save trajectory
            current_trajectory.clear();
            current_trajectory = pl->get_path_matrix();

            // check the point when the visibility constrain is not satisfied
            int nxt_start = pl->AdvanceStepCamera(current_trajectory);

            if(nxt_start>-1){
                q_start = current_trajectory[nxt_start];
                
                // Set planned trajectory.
                unique_lock<mutex> trajectory_lock(mMutexTrajectory);
                planned_trajectory.insert(planned_trajectory.end(), current_trajectory.begin(), current_trajectory.begin()+nxt_start);
                trajectory_lock.unlock();
            }

            // Ack the request.
            AckRequest();
        }
    }
}

// This function is called from Tracking thread (System).
void Planning::SendPlanningRequest(cv::Mat pose, KeyFrame* kf) {
    unique_lock<mutex> lock(mMutexRequest);
    currPose = pose;
    currKF = kf;
    hasRequest = true;
}

std::vector<std::vector<double>> Planning::GetPlanningTrajectory() {
    unique_lock<mutex> lock(mMutexTrajectory);
    std::vector<std::vector<double>> trajectory_copy = planned_trajectory;
    //std::cout << " way points number " << planned_trajectory.size() << std::endl;
    return trajectory_copy;
}

void Planning::AckRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    //planned_trajectory.clear(); 
    hasRequest = false;
}

void Planning::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

float GetTranslationMatrixDistance(const cv::Mat& pose1,
                                   const cv::Mat& pose2) {
    return pow(pose1.at<float>(0, 3) - pose2.at<float>(0, 3), 2) +
           pow(pose1.at<float>(1, 3) - pose2.at<float>(1, 3), 2) +
           pow(pose1.at<float>(2, 3) - pose2.at<float>(2, 3), 2);
}

// Given a Tsc, find the set of possibly visible points from the closest key
// frame.
std::set<MapPoint*> Planning::GetVisiblePoints(cv::Mat pose) {
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    std::vector<KeyFrame*> key_frames = mpMap->GetAllKeyFrames();
    // Find the closest key frame.
    double min_dist =
            GetTranslationMatrixDistance(pose, key_frames.front()->GetPose());
    KeyFrame* min_kf = key_frames.front();
    for (auto* kf : key_frames) {
        float curr_dist = GetTranslationMatrixDistance(pose, kf->GetPose());
        if (min_dist > curr_dist) {
            min_dist = curr_dist;
            min_kf = kf;
        }
    }
    // Find all visible points from the key frame.
    std::set<MapPoint*> visible_mps = min_kf->GetMapPoints();
    for (auto* kf : min_kf->GetConnectedKeyFrames()) {
        std::set<MapPoint*> connected_visible_mps = kf->GetMapPoints();
        visible_mps.insert(connected_visible_mps.begin(),
                           connected_visible_mps.end());
    }
    cout << "#visible points=" << visible_mps.size() << endl;
    return visible_mps;
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

bool Planning::CheckHasRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    return hasRequest;
}

int Planning::GetNumNewKeyFrames() {
    unique_lock<mutex> lock(mMutexKeyFrameQueue);
    return mKeyFrameQueue.size();
}

// This function is called from Tracking thread.
void Planning::InsertKeyFrame(KeyFrame* pKF) {
    unique_lock<mutex> lock(mMutexKeyFrameQueue);
    if (pKF->mnId > 0) {
        mKeyFrameQueue.push_back(pKF);
    }
}

KeyFrame* Planning::PopKeyFrameQueue(int num_pop) {
    // Pops the first num_pop items in the queue and
    // returns the last element popped.
    unique_lock<mutex> lock(mMutexKeyFrameQueue);
    KeyFrame* poppedKF;
    for (int i = 0; i < num_pop; i++) {
        poppedKF = mKeyFrameQueue.front();
        mKeyFrameQueue.pop_front();
    }
    return poppedKF;
}

}  // namespace ORB_SLAM
