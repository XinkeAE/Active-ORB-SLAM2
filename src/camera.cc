#include "camera.h"


camera::camera(){

    fx = 520.49;
    fy = 522.47;
    cx = 474.95;
    cy = 264.27;
    MaxX = 940;
    MinX = 20;
    MaxY = 530;
    MinY = 10;
    max_dist = 3;
    min_dist = 0.1;

    feature_threshold = THRES;

}

camera::camera(std::string map_file, std::string upper_bound_file, std::string lower_bound_file)
{
	std::cout << "Initializing camera data..." << std::endl;

    fx = 520.49;
    fy = 522.47;
    cx = 474.95;
    cy = 264.27;
    MaxX = 940;
    MinX = 20;
    MaxY = 530;
    MinY = 10;
    max_dist = 3;
    min_dist = 0.1;

    feature_threshold = THRES;

    //load files Map
    map_vec=read_text(map_file);

    upper_bound=read_text_single_line(upper_bound_file);

    lower_bound=read_text_single_line(lower_bound_file);

    T_sw<<0,   -1.0000,         0,    -0.1000,
          0,         0,   -1.0000,         0,
          1.0000,         0,         0,   -0.2500,
          0,         0,        0,    1.0000;

    T_bc <<0 ,        0 ,   1.0000 ,   0.2500 ,
            -1.0000 ,        0  ,       0 ,  -0.1000,
            0 ,  -1.0000 ,        0  ,       0,
            0 ,        0  ,       0   , 1.0000;

}

/*
camera::camera(std::vector<MapPoint*> &vpPts)
{
	std::cout << "Initializing camera data..." << std::endl;

    fx = 520.49;
    fy = 522.47;
    cx = 474.95;
    cy = 264.27;
    MaxX = 940;
    MinX = 20;
    MaxY = 530;
    MinY = 10;
    max_dist = 3;
    min_dist = 0.1;

    feature_threshold = THRES;

    //load files Map
    update_map(vpPts);

    T_sw<<0,   -1.0000,         0,    -0.1000,
          0,         0,   -1.0000,         0,
          1.0000,         0,         0,   -0.2500,
          0,         0,        0,    1.0000;

    T_bc <<0 ,        0 ,   1.0000 ,   0.2500 ,
            -1.0000 ,        0  ,       0 ,  -0.1000,
            0 ,  -1.0000 ,        0  ,       0,
            0 ,        0  ,       0   , 1.0000;

}
*/

int camera::countVisible(float x_w, float y_w, float theta_rad_w) const {
    int num_pt=map_vec.size();
    int num_visible=0;

    //cout << x_w << " " << y_w << " " << theta_rad_w << endl;

    // I modified this section to enable the function to be const. (requirement of OMPL)
    Eigen::Matrix4f T_wb;
    T_wb<<(float)(cos(theta_rad_w)),  (float)(-sin(theta_rad_w)), 0, x_w,
            (float)(sin(theta_rad_w)), (float)cos(theta_rad_w), 0, y_w,
            0,         0,         1,   0,
            0,         0,        0,    1.0000;

    Eigen::Matrix4f T_sc = T_sw * T_wb * T_bc;
    Eigen::Matrix4f T_cs = T_sc.inverse();

    if (1) {//setRobotPose(x_w, y_w, theta_rad_w)) {
        
        for (int i = 0; i < num_pt; ++i)
        {
            if (isInFrustum(map_vec[i],  upper_bound[i],  lower_bound[i], T_sc, T_cs))
                num_visible+=1;
        }
    }   
    else
    {
        std::cout<<"Cannot get current robot pose"<<std::endl;
    } 
    return num_visible;
}

bool camera::IsStateVisiblilty(float x_w, float y_w, float theta_rad_w) {

	return countVisible(x_w, y_w, theta_rad_w) > feature_threshold;
}


bool camera::isInFrustum(std::vector<float> MapPoint_s, float upper_limit, float lower_limit, Eigen::Matrix4f T_sc, Eigen::Matrix4f T_cs) const {
    
        //convert map points into carmera frame
        Eigen::Matrix<float, 4, 1> map_point_s(MapPoint_s.data());
        Eigen::Matrix<float, 4, 1> map_point_c=T_cs*map_point_s; 

        float PcX = map_point_c(0,0);
        float PcY= map_point_c(1,0);
        float PcZ = map_point_c(2,0);

        // step 1: rule out the points with depth smaller than 0.05
        if(PcZ<0.05)
            return false;
        // step 2: rule out the points which are out of current view
        float inv_z = 1.0f/PcZ;
        float u=fx*PcX*inv_z+cx;
        float v=fy*PcY*inv_z+cy;
        

        if(u<MinX || u>MaxX)    
            return false;

        if(v<MinY || v>MaxY)
            return false;

        // step 3: rule out the points which are too close or too far away
        float dist=sqrt(PcZ*PcZ+PcY*PcY+PcX*PcX);
        if(dist<min_dist || dist>max_dist)
            return false;

        //% step 4: rule out the points whose viewing direction is out of 95% t-distribution
    
        float delta_x_s=map_point_s(0,0)-T_sc(0,3);
        float delta_z_s=map_point_s(2,0)-T_sc(2,3);
        float theta_robot_s=atan2(delta_x_s,delta_z_s);
        
        if(theta_robot_s<lower_limit || theta_robot_s>upper_limit)
            return false;

        return true;
    
}

std::vector<std::vector<float> > camera::read_text(std::string filename)
{
    std::ifstream          file(filename.c_str());
    if (!file.is_open()) {
    	cout << "Error opening file " << filename.c_str() << endl;
    	exit(1);
    }
    
    std::string  line;
    std::cout<<"open file: "<<filename<<std::endl;
    std::vector<std::vector<float>> out;

    // Read one line at a time into the variable line:
    while(std::getline(file, line))
    {
        std::vector<float>   lineData;
        
        std::stringstream  lineStream(line);

        float value;
        // Read an integer at a time from the line
        while(lineStream >> value)
        {
            // Add the integers from a line to a 1D array (vector)
            lineData.push_back(value);
            //std::cout<<value<<" ";
        }
        // When all the integers have been read, add the 1D array
        // into a 2D array (as one line in the 2D array)
        out.push_back(lineData);
    }

    return out;

}


std::vector<float>  camera::read_text_single_line(std::string filename)
{
    std::ifstream          file(filename.c_str());
    
    std::string  line;
    std::cout<<"open file: "<<filename<<std::endl;
    std::vector<float> out;

    // Read one line at a time into the variable line:
    while(std::getline(file, line))
    {
        
        std::stringstream  lineStream(line);

        float value;
        // Read an integer at a time from the line
        while(lineStream >> value)
        {
            // Add the integers from a line to a 1D array (vector)
            out.push_back(value);
        }
        // When all the integers have been read, add the 1D array
        // into a 2D array (as one line in the 2D array)
        
    }
    std::cout<<" Load Finished!"<<std::endl;
    return out;

}

/*
void camera::update_map(std::vector<MapPoint*> &vpPts)
{
    if(!map_vec.empty())
    {
        map_vec.clear();
        upper_bound.clear();
        lower_bound.clear();
    }


    if(!vpPts.empty())
    {
        for(size_t i=0; i<vpPts.size(); i++){
            MapPoint* pPt = vpPts[i];
    
            if(pPt->isBad())
                continue;
            
            cv::Mat Pos = pPt->GetWorldPos();
            cv::Mat Normal = pPt->GetNormal();
            int number_observed = pPt->Observations();

            std::vector<float> theta_s;
            float theta_mean;
            float theta_std;
    
            for(size_t j=0; j<pPt->mNormalVectors.size(); j++)
            {
                cv::Mat normali = pPt->mNormalVectors[j];
                float theta = atan2(normali.at<float>(0,0),normali.at<float>(2,0));
                theta_s.push_back(theta);
            }
    
            compute_std(theta_s, theta_mean, theta_std);

            std::vector<float> one_pt;
            one_pt.push_back(Pos.at<float>(0));
            one_pt.push_back(Pos.at<float>(1));
            one_pt.push_back(Pos.at<float>(2));
            one_pt.push_back(1f);

            map_vec.push_back(one_pt);
            lower_bound.push_back(theta_mean-2.5*theta_std);
            upper_bound.push_back(theta_mean+2.5*theta_std);
        }
    }
}
*/

void camera::compute_std(std::vector<float> v, float & mean, float & stdev)
{
    float sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();

    std::vector<float> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                std::bind2nd(std::minus<float>(), mean));
    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
}




/*
cv::Mat_<float> camera::vec2cvMat_2D(std::vector< std::vector<float> > &inVec){
  int rows = static_cast<int>(inVec.size());
    int cols = static_cast<int>(inVec[0].size());

    //std::cout<<rows<<" "<<cols<<std::endl;

    cv::Mat_<float> resmat(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        resmat.row(i) = cv::Mat(inVec[i]).t();
    }
    std::cout<<" Load Finished!"<<std::endl;
    return resmat;
}*/









