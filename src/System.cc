/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

#include <time.h>

#include <numeric>
#include <math.h>

void compute_std(std::vector<float> v, float & mean, float & stdev)
{
    float sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();

    std::vector<float> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                std::bind2nd(std::minus<float>(), mean));
    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
}

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, Map* map, ORBVocabulary* voc, const string& strMapFile):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }
    

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
	  bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	else
	  bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    if (map == NULL && strMapFile.empty()) {
        cout << "Create new map." << endl;
        mpMap = new Map();
    } else {
        mpMap = new Map();
        if (!LoadMapCameraPara(strSettingsFile)) {
            cerr << "Failed to load map camera setting from " << strMapFile << endl;
            exit(-1);
        }        
        if (!LoadMap(strMapFile)) {
            cerr << "Failed to load map from " << strMapFile << endl;
            exit(-1);
        }
        cerr << "Map loaded with " << mpMap->MapPointsInMap() << " points."
             << endl;
        for(auto kf: mpMap->GetAllKeyFrames()) {
            mpKeyFrameDatabase->add(kf);
        }
    }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
                             
    // Initialize the Planning thread
    mpPlanner = new Planning(cv::Mat(), mpMap);
    mptPlanning = new thread(&ORB_SLAM2::Planning::Run, mpPlanner);

    // Initialize the Octomap thread
    mpOctomapBuilder = new OctomapBuilder();
    mptOctomapBuilding = new thread(&ORB_SLAM2::OctomapBuilder::Run,
                                    mpOctomapBuilder);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }
    

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    if(!goal_set){
        x_goal = 4.0;
        
        y_goal = 0.0;
        theta_goal = 0.0;
        
        mpTracker->set_goal(x_goal, y_goal, theta_goal);
        mpPlanner->set_goal(x_goal, y_goal, theta_goal);

        goal_set = true;
        
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    // Request planning
    cv::Mat currPose = mpTracker->currPose.clone();

    if (!currPose.empty()){
        x_curr = currPose.at<float>(0,3);
        y_curr = currPose.at<float>(1,3);
        theta_curr = atan2(currPose.at<float>(1,0), currPose.at<float>(0,0));
        
        // update octomap
        if (mpTracker->mbKeyframe || !octomapInitialize) {

            mpOctomapBuilder->UpdateOctomap(depthmap, currPose);
            if(mpOctomapBuilder->calcOccupiedPoints()){
                vector<vector<float>> occupiedPoints = mpOctomapBuilder->getOccupiedPoints();
                vector<vector<float>> lowptrs=mpOctomapBuilder->getLowProbPoints();
                mpTracker->UpdateCollision(occupiedPoints);
                mpTracker->UpdateLow(lowptrs);

                // frontier computation
                mpOctomapBuilder->findFrontier();
                vector<vector<float>> frontier = mpOctomapBuilder->getFrontier();
                mpOctomapBuilder->clusterFrontier();
                vector<vector<float>> frontier_center = mpOctomapBuilder->getFrontierCenter();
                mpTracker->UpdateFrontier(frontier);
                mpTracker->UpdateFrontierCenter(frontier_center);

                if(occupiedPoints.size()!=0 && frontier.size()!=0)
                    octomapInitialize = true;
            }
           
        }

        
        // octomap variables
        std::vector<std::vector<float>> floorMap;
        std::vector<std::vector<float>> frontierMap;
        std::vector<std::vector<double>> planned_trajectory;

        // call planning
        if (octomapInitialize && !mpTracker->goalReached){

            // call the planning
            if(mpTracker->needPlanning()){
                
                // get the floor map and frontier map 
                // TodoXinke: add the empty space  
                frontierMap.clear();
                floorMap.clear();
                frontierMap = mpOctomapBuilder->getFrontier();
                floorMap = mpOctomapBuilder->getOccupiedPoints();

                mpPlanner->setFloorMap( floorMap );
                
                if(mpPlanner->approxSolution){
                    mpPlanner->SendPlanningRequest(cv::Mat(), nullptr);
                    planRequestSent = true;
                }

            }


            // update the planned trajectory            
            if(mpTracker->planned_trajectory.size() != mpPlanner->GetPlanningTrajectory().size()){
                // copy the planned trajectory out
                planned_trajectory = mpPlanner->GetPlanningTrajectory();
                for (size_t planCopyIter = mpTracker->planned_trajectory.size(); planCopyIter < (planned_trajectory.size()); planCopyIter++ ){
                    std::vector<double> wayPoint = planned_trajectory[planCopyIter];
                    mpTracker->planned_trajectory.push_back(wayPoint);
                }
                planRequestSent = false;
                mpTracker->explorationFinish = false; // reset the exploration flag
            }

        }

    }

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

Map* System::GetMap() {return this->mpMap;}

bool System::GetKeyframeStatus()
{
    return mpTracker->mbKeyframe;
}

cv::Mat System::GetCamIntrinsic(){
    return mpTracker->GetCamIntrinsic();
}

float System::GetDepthScaleFactor(){
    return mpTracker->GetDepthScaleFactor();
}

bool System::SaveMap(const string &filename) {
    cerr << "System Saving to " << filename << endl;
    return mpMap->Save(filename);
}

bool System::LoadMap(const string& filename) {
    cerr << "Loading map from " << filename << endl;
    return mpMap->Load(filename, *mpVocabulary);
}

bool System::LoadMapCameraPara(const string& filename) {
    cerr << "Loading camera setting from " << filename << endl;
    return mpMap->LoadCofficient(filename);
}


void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }*/

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    list<bool>::iterator lbKf = mpTracker->mlbKeyFrame.begin();
	list<int>::iterator lnM = mpTracker->mlMatchesInliers.begin(); //xinke 
    list<int>::iterator lnMBad = mpTracker->mlMatchesOutliers.begin(); //xinke
    list<int>::iterator lnNMP = mpTracker->mlMapPoints.begin(); //xinke 
    list<int>::iterator lnM_mdl = mpTracker->mlModelMatchesInliers.begin(); //xinke 
    list<int>::iterator lnM_kf = mpTracker->mlKfMatchesInliers.begin(); //xinke 
    list<int>::iterator lnObsv = mpTracker->mlTotalObservations.begin(); //xinke 
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++, lnM++, lnNMP++, lbKf++, lnM_mdl++, lnM_kf++, lnObsv++, lnMBad++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        double kfTimeStamp = pKF->mTimeStamp;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
            kfTimeStamp = pKF->mTimeStamp;
        }

        Trw = Trw*pKF->GetPose()*Two;
        cv::Mat Trw_kf = pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        cv::Mat Rwr = Trw_kf.rowRange(0,3).colRange(0,3).t();
        cv::Mat twr = -Rwr*Trw_kf.rowRange(0,3).col(3);

        vector<float> q_kf = Converter::toQuaternion(Rwr);         

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " 
          << setprecision(6) << kfTimeStamp << " " << setprecision(9) << twr.at<float>(0) << " " << twr.at<float>(1) << " " << twr.at<float>(2) << " " << q_kf[0] << " " << q_kf[1] << " " << q_kf[2] << " " << q_kf[3]
                             << " "  <<  *lnM << " " << *lnNMP << " " << *lbKf << " " << *lnM_mdl << " " << *lnM_kf << " " << *lnObsv << " " << *lnMBad << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        KeyFrame* pKF_p = vpKFs[i]->GetParent();

        if(pKF_p==NULL)
            continue;
       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad()||pKF_p->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();

        cv::Mat R_p = pKF_p->GetRotation().t();
        vector<float> q_p = Converter::toQuaternion(R_p);
        cv::Mat t_p = pKF_p->GetCameraCenter();

        int nConnectedKfs = (pKF->GetConnectedKeyFrames()).size();
        int nMatchedMps = (pKF->GetMapPointMatches()).size();
        int nMps = (pKF->GetMapPoints()).size();

        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " "
          << setprecision(6) << pKF_p->mTimeStamp << setprecision(7) << " " << t_p.at<float>(0) << " " << t_p.at<float>(1) << " " << t_p.at<float>(2)
          << " " << q_p[0] << " " << q_p[1] << " " << q_p[2] << " " << q_p[3] << " "          
          << " " << nConnectedKfs << " " << nMatchedMps << " " << nMps << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM_fe(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cout << "pass0!" << endl;
        cv::Mat R = pKF->GetFirstEstRotation().t();
        cout << "pass1!" << endl;
        vector<float> q = Converter::toQuaternion(R);
        cout << "pass2!" << endl;
        cv::Mat t = pKF->GetFirstEstCameraCenter();
        cout << "pass!" << endl;

        cv::Mat R_p = pKF->GetFirstEstRotation_parent().t();
        vector<float> q_p = Converter::toQuaternion(R_p);
        cv::Mat t_p = pKF->GetFirstEstCameraCenter_parent();

        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " 
          << setprecision(6) << pKF->mTimeStamp_parent << setprecision(7) << " " << t_p.at<float>(0) << " " << t_p.at<float>(1) << " " << t_p.at<float>(2)
          << " " << q_p[0] << " " << q_p[1] << " " << q_p[2] << " " << q_p[3] << " "
          << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveMapPoints(const string &filename)
{
    cout << endl << "Saving map points to " << filename << " ..." << endl;

    vector<MapPoint*> vpPts = mpMap->GetAllMapPoints();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    cout << "totally " << vpPts.size() << " points." << endl;
    for(size_t i=0; i<vpPts.size(); i++){
        MapPoint* pPt = vpPts[i];

        if(pPt->isBad())
            continue;
        
        cv::Mat Pos = pPt->GetWorldPos();
        cv::Mat Normal = pPt->GetNormal();
        int number_observed = pPt->Observations();

        //KeyFrame* pKF = pPt->GetReferenceKeyFrame();

        std::vector<float> view_cos_vector;
        float view_mean;
        float view_std;

        std::vector<float> theta_s;
        float theta_mean;
        float theta_std;

        for(size_t j=0; j<pPt->mNormalVectors.size(); j++)
        {
            cv::Mat normali = pPt->mNormalVectors[j];
            view_cos_vector.push_back(normali.dot(Normal));   
            //std::cout << j << " view cosine = " <<  normali.dot(Normal) << std::endl; 

            float theta = atan2(normali.at<float>(0,0),normali.at<float>(2,0));
            theta_s.push_back(theta);

        }

        compute_std(view_cos_vector, view_mean, view_std);
        compute_std(theta_s, theta_mean, theta_std);

        cv::Mat Pos_f;
        cv::Mat Normal_f;

        Pos.convertTo(Pos_f,CV_32FC1);
        Normal.convertTo(Normal_f,CV_32FC1);

        f  << Pos_f.at<float>(0) << " " << Pos_f.at<float>(1) << " " << Pos_f.at<float>(2) << " "
           << Normal_f.at<float>(0) << " " << Normal_f.at<float>(1) << " " << Normal_f.at<float>(2) << " "
           << number_observed << " " << theta_mean << " " << theta_std << endl;   
 
    }

    f.close();
    cout << endl << "map saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

std::vector<double> System::getCurrWaypoint()
{
    return mpTracker->curr_des;
}

bool System::getRecoverMode(){
    return mpTracker->recoverMode;
}

int System::getExplorationStatus(){
    return mpTracker->explorationStatus;
}

} //namespace ORB_SLAM
