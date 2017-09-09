#include "OctomapBuilder.h"

namespace ORB_SLAM2 {

OctomapBuilder::OctomapBuilder(){
    globalOctoMap = new octomap::OcTree(0.1f);
    hasUpdate = false;
}

void OctomapBuilder::Run() {
    while (1) {
        if (CheckHasUpdate()) {
            // TODO: convert currPose to octomap::pose6d T_wc_octo
            octomap::pose6d T_wc_octo;
            octomap::Pointcloud local_cloud;  
            for(int m=0; m<depth.rows;m++){
                for (int n=0; n<depth.cols; n++){
                float d = depth.ptr<float>(m)[n];
                if(d == 0) continue;
                float z = d;
                float x = (float(n) - camera_cx) * z / camera_fx;
                float y = (float(m) - camera_cy) * z / camera_fy;
                if(z > 5) continue;
                if( y > 0.25 || y < -0.2) continue;
                local_cloud.push_back(x,y,z);
        }
    }
    local_cloud.transform(T_wc_octo);
    octomath::Vector3 vec3 = T_wc_octo.trans();
    globalOctoMap->insertPointCLoud(
            local_cloud, octomap::point3d(vec3.x(), vec3.y(), vec3.z()));
        }
    }
}

// This function is called from Tracking thread (System).
void OctomapBuilder::SendGetOctomapRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    hasRequest = true;
}

void OctomapBuilder::UpdateOctomap(const cv::Mat &depth, cv::Mat currPose) {
    unique_lock<mutex> lock(mMutexUpdate);
    hasUpdate = true;
    depth = depth;
    currPose = currPose;
}

bool Planning::CheckHasUpdate() {
    unique_lock<mutex> lock(mMutexUpdate);
    return hasUpdate;
}

}  // namespace ORB_SLAM
