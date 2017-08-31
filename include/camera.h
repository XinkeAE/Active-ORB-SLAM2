#ifndef CAM_MOD
#define CAM_MOD
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <iterator>
#include <stdio.h>
#include <numeric>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "MapPoint.h"
//#include <system.h>
#include <opencv2/core/eigen.hpp>

#define THRES 100

typedef struct {
    	std::vector<std::vector<double>> Map;
    	std::vector<double> UB;
        std::vector<double> LB;
        std::vector<double> maxDist;
        std::vector<double> minDist;
} map_data;

using namespace std;

class camera
{
public:
    camera(map_data MD);

    camera(std::vector<ORB_SLAM2::MapPoint*> &vpPts);
    
    std::vector<std::vector<float> > read_text(std::string points);

    std::vector<float> read_text_single_line(std::string points);

    bool isInFrustum(std::vector<float> MapPoint_s, float upper_limit, float lower_limit, Eigen::Matrix4f, Eigen::Matrix4f, float max_range, float min_range) const;

    bool setRobotPose(Eigen::Matrix4f Twb);

    bool setRobotPose(float x_w, float y_w, float theta_rad_w);

    int countVisible(float x_w, float y_w, float theta_rad_w) const;
    int countVisible(cv::Mat Twb) const;

    bool IsStateVisiblilty(double x_w, double y_w, double theta_rad_w, int thres = -1);

    void update_map(std::vector<ORB_SLAM2::MapPoint*> &vpPts);

private:

    //cv::Mat_<float> vec2cvMat_2D(std::vector< std::vector<float> > &inVec);
    void compute_std(std::vector<float> v, float & mean, float & stdev);

    // camera parameters:
    float fx, fy;
    float cx, cy;
    float MinX, MaxX;
    float MinY, MaxY;
    float max_dist, min_dist;

    // map points:
    std::vector<std::vector<float> > map_vec;
    //cv::Mat map;
    std::vector<float> upper_bound;
    std::vector<float> lower_bound;
    std::vector<float> max_range;
    std::vector<float> min_range;


    // camera frame
    /*
    cv::Mat T_cs;
    cv::Mat T_sc;
    cv::Mat T_sw;
    cv::Mat T_bc;
    cv::Mat T_sb;
    */
    //Eigen::Matrix4f T_cs;
    //Eigen::Matrix4f T_sc;
    Eigen::Matrix4f T_sw;
    Eigen::Matrix4f T_bc;
    Eigen::Matrix4f T_sb; 

    int feature_threshold;

};


#endif
