#ifndef CAM_MOD
#define CAM_MOD
//#include <opencv2/opencv.hpp>
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
//#include <system.h>

#define THRES 20

using namespace std;

class camera
{
public:
    camera();

    camera(std::string map_file, std::string upper_bound_file, std::string lower_bound_file);

    camera(std::vector<MapPoint*> &vpPts);
    
    std::vector<std::vector<float> > read_text(std::string points);

    std::vector<float> read_text_single_line(std::string points);

    bool isInFrustum(std::vector<float> MapPoint_s, float upper_limit, float lower_limit, Eigen::Matrix4f, Eigen::Matrix4f) const;

    bool setRobotPose(Eigen::Matrix4f Twb);

    bool setRobotPose(float x_w, float y_w, float theta_rad_w);

    int countVisible(float x_w, float y_w, float theta_rad_w) const;

    bool IsStateVisiblilty(float x_w, float y_w, float theta_rad_w);

    void update_map(std::vector<MapPoint*> &vpPts)

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
