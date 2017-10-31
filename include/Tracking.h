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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<math.h>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "Planning.h"
#include "OctomapBuilder.h"
#include "StateValidChecker.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    // Get Camera Intrinsic Matrix
    cv::Mat GetCamIntrinsic();

    // Get Scale Factor
    float GetDepthScaleFactor();

    void UpdateCollision(const std::vector<std::vector<float>> &bCollision);
    void UpdateLow(const std::vector<std::vector<float>> &bCollision);
    void UpdateFrontier(const std::vector<std::vector<float>> &frontier);
    void UpdateFrontierCenter(const std::vector<std::vector<float>> &frontierCenter);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;
    
    

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;
    list<int> mlModelMatchesInliers; //Xinke
    list<int> mlKfMatchesInliers; //Xinke
	list<int> mlMatchesInliers; // Xinke
    list<int> mlMatchesOutliers; // Xinke
    list<int> mlMapPoints; // Xinke
    list<int> mlTotalObservations; //Xinke
    list<bool> mlbKeyFrame;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    // Check if the current frame is keyframe
    bool mbKeyframe;

    void Reset();

    // For planning compute the pose in the world frame
    cv::Mat T_wb_initial_mat;
    cv::Mat currPose;
    // 2d coordinates
    float x_curr = 0;
    float y_curr = 0;
    float theta_curr = 0;

    float x_goal = 0.0;
    float y_goal = 0;
    float theta_goal = 0;
    void set_goal(float x, float y, float theta);

    cv::Mat T_ws_mat = (cv::Mat_<float>(4,4) <<    0, 0, 1, 0.22, //0.22,//0.25,
                                                -1, 0, 0, -0.1, // -0.1,//-0.1,
                                                0,-1, 0, 0,
                                                0, 0, 0, 1);

    cv::Mat T_cb_mat = (cv::Mat_<float>(4,4) << 0, -1, 0, -0.1, //-0.1,
                                                0, 0, -1, 0,
                                                1,0, 0, -0.22, //-0.22,
                                                0, 0, 0, 1);
    
    bool TwbInitialized = false;
    size_t TwbCounter = 0;

    // Planning/Control related
    std::vector<std::vector<double>> planned_trajectory;
    std::vector<double> curr_des;
    int path_it_counter=0;
    bool needPlanning();
    bool checkWayPoint();

    // Exploration related
    // exploration mode:
    // 0: not in exploration mode
    // -1: explore left
    // -2: return to right
    // 1: explore right
    // 2: return to left
    int explorationStatus = 0 ;
    bool explorationFinish = true;
    bool needExploration();
    bool computeExplorationMode();
    float explore_star_angle;
    float explore_stop_diff;
    std::vector<std::vector<float>> frontierCenters;
    std::vector<std::vector<float>> frontier;
    std::vector<std::vector<double>> occupied;
    std::vector<float> frontierCentersDir;
    float Bound1 = 0;
    float Bound2 = 0;
    size_t goalDetectedCounter = 0;


    // recover
    bool recoverMode = false;
    bool recover_success=false;
    size_t succ_counter=0;
    size_t recoverCounter = 0;
    size_t waypointCounter = 0;
    bool checkWayPointRecover();

    bool goalDetected = false;
    bool goalReached = false;
    bool isGoalReached(float x_curr, float y_curr, float theta_curr);
    
    
    
protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
