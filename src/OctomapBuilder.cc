#include "OctomapBuilder.h"

namespace ORB_SLAM2 {

OctomapBuilder::OctomapBuilder(){
    globalOctoMap = new octomap::OcTree(0.1f);
    hasUpdate = false;
    camera_cx = 474.95;
    camera_cy = 264.26;
    camera_fx = 520.48;
    camera_fy = 522.47;

    cv::Mat T_cb_mat = (cv::Mat_<float>(4,4) <<     0, -1, 0, -0.1, //-0.1,
    0, 0, -1, 0,
    1,0, 0, -0.22, //-0.22,
    0, 0, 0, 1);

    depthFactor = 1/1027.6;

    T_bc = T_cb_mat.inv();

}

void OctomapBuilder::Run() {
    while (1) {
        if (CheckHasUpdate()) {

                //cout  << __LINE__ << endl;
                Eigen::Matrix4f T_wc_eig = Converter::toMatrix4f(T_wc_mat);
                Eigen::Quaternionf q_wc(T_wc_eig.topLeftCorner<3,3>());

                //cout  << __LINE__ << endl;

                octomap::pose6d T_wc_octo(octomap::point3d(T_wc_eig(0,3),T_wc_eig(1,3),T_wc_eig(2,3)),
                                        octomath::Quaternion(q_wc.w(), q_wc.x(), q_wc.y(), q_wc.z() ) );

                //cout  << __LINE__ << endl;

                cout << depth.empty() << endl;

                octomap::Pointcloud local_cloud;  
                for(int m=0; m<depth.rows;m++){
                        for (int n=0; n<depth.cols; n++){
                        //cout  << __LINE__ << endl;    
                        //cout << depth << endl;

                        //cout << "depth row = " << depth.rows << ", depth col = " << depth.cols << endl;
                        //cout << "m = " << m << ", n= " << n << endl;

                        if(depth.ptr(m)==NULL){
                            cout << "pointer problem!!!!" << endl;
                        }
                        
                        float d = depth.ptr<float>(m)[n];

                        //cout  << __LINE__ << endl;
                        


                        if(d < 0.05) continue;
                        //cout  << __LINE__ << endl;
                        

                        float z = d;
                        float x = (float(n) - camera_cx) * z / camera_fx;
                        float y = (float(m) - camera_cy) * z / camera_fy;

                        //cout  << __LINE__ << endl;
                        
                        //cout << "x = " << x << ", y = " << y << ", z = " << z << endl;

                        if(z > 5) continue;
                        if( y > 0.25 || y < -0.2) continue;
                        //cout  << __LINE__ << endl;                        
                        local_cloud.push_back(x,y,z);
                        //cout  << __LINE__ << endl;
                    }
                }
                //cout  << __LINE__ << endl;            
                local_cloud.transform(T_wc_octo);
                //cout  << __LINE__ << endl;
                octomath::Vector3 vec3 = T_wc_octo.trans();
                //cout  << __LINE__ << endl;
                globalOctoMap->insertPointCloud(
                        local_cloud, octomap::point3d(vec3.x(), vec3.y(), vec3.z()));
                //cout  << __LINE__ << endl;
                AckRequest();
        }
    }
}

// This function is called from Tracking thread (System).
void OctomapBuilder::SendGetOctomapRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    hasRequest = true;
}

void OctomapBuilder::UpdateOctomap(const cv::Mat &depth_, cv::Mat currPose_) {
    unique_lock<mutex> lock(mMutexUpdate);
    hasUpdate = true;
    depth = depth_.clone();
    depth.convertTo(depth,CV_32F,depthFactor);
    currPose = currPose_.clone();
    T_wc_mat = currPose*T_bc;
}

bool OctomapBuilder::CheckHasUpdate() {
    unique_lock<mutex> lock(mMutexUpdate);
    return hasUpdate;
}

void OctomapBuilder::AckRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    hasRequest = false;
}

}  // namespace ORB_SLAM
