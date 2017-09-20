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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::Publisher& posePub, ros::Publisher octoPub, ros::Publisher destPub, ros::Publisher lostPub, ros::Publisher explorePub, ros::Publisher dirPub)
                :mpSLAM(pSLAM),posePublisher(posePub),octomapPublisher(octoPub),currDestPublisher(destPub), trackingLostPublisher(lostPub), explorationPublisher(explorePub), directionPublisher(dirPub)
    {
        T_ws_mat = (cv::Mat_<float>(4,4) <<    0, 0, 1, 0.22, //0.22,//0.25,
                                               -1, 0, 0, -0.1, // -0.1,//-0.1,
                                                0,-1, 0, 0,
                                                0, 0, 0, 1);
        T_cb_mat = (cv::Mat_<float>(4,4) <<     0, -1, 0, -0.1, //-0.1,
                                                0, 0, -1, 0,
                                                1,0, 0, -0.22, //-0.22,
                                                0, 0, 0, 1);

        T_ws = cvMatToTF(T_ws_mat);

        globalOctoMap = new octomap::OcTree(0.1f);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    tf::Transform cvMatToTF ( cv::Mat Tcw );

    tf::Transform T_ws; // transformation from slam frame (z forward, x right) to world frame (z up, x forward)
    cv::Mat T_ws_mat;
    cv::Mat T_cb_mat;
    cv::Mat T_wc_mat;

    cv::Mat T_wb_mat;
    cv::Mat T_wb_initial_mat;
    bool initialized = false;
    int counter = 0;

    // declare the octomap here
    octomap::OcTree* globalOctoMap;
    cv::Mat CamIntrinsic;
    float depthFactor;
    float camera_fx;
    float camera_fy;
    float camera_cx;
    float camera_cy;

    ORB_SLAM2::System* mpSLAM;
    geometry_msgs::PoseStamped pose_out_;
    octomap_msgs::Octomap octomap_out_;
    geometry_msgs::Pose2D dest_out_;
    ros::Publisher posePublisher; 
    ros::Publisher octomapPublisher;
    ros::Publisher currDestPublisher;
    ros::Publisher trackingLostPublisher;
    ros::Publisher explorationPublisher;
    ros::Publisher directionPublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3 && argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings (path_to_map)" << endl;        
        ros::shutdown();
        return 1;
    }   

    string path_to_map = argc == 4 ? string(argv[3]) : "";
    bool is_localization_mode = argc == 4;  

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,NULL,NULL,path_to_map);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_mono", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped> ( "/orb/pose_est", 2 );
    ros::Publisher octo_pub = nh.advertise<octomap_msgs::Octomap>( "octomap_3d", 1 );
    ros::Publisher dest_pub = nh.advertise<geometry_msgs::Pose2D> ("planning/data", 1);
    ros::Publisher lost_pub = nh.advertise<std_msgs::Bool> ("/orb/lost", 1);
    ros::Publisher expr_pub = nh.advertise<std_msgs::Bool> ("/orb/expr", 1);
    ros::Publisher dir_pub = nh.advertise<std_msgs::Bool> ("/orb/dir", 1);

    ImageGrabber igb(&SLAM, pose_pub, octo_pub, dest_pub, lost_pub, expr_pub, dir_pub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    if (is_localization_mode) {
		cerr << endl << "Activate localization." << endl;
        SLAM.ActivateLocalizationMode();
	}   

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("Trajectory.txt");
    SLAM.SaveMapPoints("MapPoints.txt");
    
    if(!is_localization_mode){
        SLAM.SaveMap("ORBSLAM_Full_Map.txt");
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    CamIntrinsic = mpSLAM->GetCamIntrinsic();
    camera_fx = CamIntrinsic.at<float>(0,0);
    camera_fy = CamIntrinsic.at<float>(1,1);
    camera_cx = CamIntrinsic.at<float>(0,2);
    camera_cy = CamIntrinsic.at<float>(1,2);

    depthFactor = mpSLAM->GetDepthScaleFactor();

    trackingLostPublisher.publish(mpSLAM->getRecoverMode());
    
    if (pose.empty())
        return;
    else{
        pose = pose.inv();
        if(!initialized){
            T_wb_initial_mat = cv::Mat(T_ws_mat*pose*T_cb_mat);
            counter ++;
            if (counter > 5)
                initialized = true;
        }
        T_wb_mat = cv::Mat(T_ws_mat*pose*T_cb_mat);

        tf::Transform pose_tf = cvMatToTF(T_wb_initial_mat.inv()*T_wb_mat);

        tf::Quaternion pose_orientation = (pose_tf).getRotation();
        tf::Vector3 pose_origin = (pose_tf).getOrigin();

        pose_out_.header.stamp = cv_ptrRGB->header.stamp;
        pose_out_.pose.position.x = pose_origin.getX();
        pose_out_.pose.position.y = pose_origin.getY();
        pose_out_.pose.position.z = pose_origin.getZ();
        pose_out_.pose.orientation.x = pose_orientation.getX();
        pose_out_.pose.orientation.y = pose_orientation.getY();
        pose_out_.pose.orientation.z = pose_orientation.getZ();
        pose_out_.pose.orientation.w = pose_orientation.getW();

        if(initialized){
            posePublisher.publish(pose_out_);

            std::vector<double> curr_dest = mpSLAM->getCurrWaypoint();

            int exprStatus = mpSLAM->getExplorationStatus();

            if(exprStatus == 0){
                explorationPublisher.publish(false);
            }else{
                explorationPublisher.publish(true);
                if(exprStatus == 1){
                    directionPublisher.publish(false);
                }else if(exprStatus == -1){
                    directionPublisher.publish(true);
                }
            }

            if(curr_dest.size() > 0){
                dest_out_.x = float(curr_dest[0]);
                dest_out_.y = float(curr_dest[1]);
                dest_out_.theta = float(curr_dest[2]);
                currDestPublisher.publish(dest_out_);
            }

        }
            

    }

    

}

tf::Transform ImageGrabber::cvMatToTF ( cv::Mat Tcw ) {
    tf::Transform cam_to_first_keyframe_transform;
    // invert since Tcw (transform from world to camera)
    cv::Mat pose = Tcw;

    //Extract transformation from the raw orb SLAM pose
    tf::Vector3 origin;
    tf::Matrix3x3 transform_matrix;

    origin.setValue ( pose.at<float> ( 0, 3 ), pose.at<float> ( 1, 3 ), pose.at<float> ( 2, 3 ) );

    transform_matrix.setValue ( pose.at<float> ( 0, 0 ), pose.at<float> ( 0, 1 ), pose.at<float> ( 0, 2 ),
                                pose.at<float> ( 1, 0 ), pose.at<float> ( 1, 1 ), pose.at<float> ( 1, 2 ),
                                pose.at<float> ( 2, 0 ), pose.at<float> ( 2, 1 ), pose.at<float> ( 2, 2 ) );

    //transform_matrix.getRotation(transform_quat);
    cam_to_first_keyframe_transform.setOrigin ( origin );
    cam_to_first_keyframe_transform.setBasis ( transform_matrix );

    return cam_to_first_keyframe_transform;
}


