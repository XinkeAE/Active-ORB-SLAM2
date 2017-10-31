#include "OctomapBuilder.h"
#include <math.h> 

namespace ORB_SLAM2 {

OctomapBuilder::OctomapBuilder(){
    globalOctoMap = new octomap::OcTree(0.15f);
    globalOctoMap->setOccupancyThres(0.7);
    //globalOctoMap->setProbMiss(0.51);
    hasUpdate = false;
    /*
    camera_cx = 474.95;
    camera_cy = 264.26;
    camera_fx = 520.48;
    camera_fy = 522.47;*/

    camera_fx = 554.2547;
    camera_fy = 554.2547;
    camera_cx = 320.5;
    camera_cy = 240.5;   

    cv::Mat T_cb_mat = (cv::Mat_<float>(4,4) <<     0, -1, 0, -0.1,
    0, 0, -1, 0,
    1,0, 0, -0.22,
    0, 0, 0, 1);

    //depthFactor = 1/1027.6; // real camera
    depthFactor = 1;

    T_bc = T_cb_mat.inv();

    lut = new octomap::OcTreeLUT(16);

}

void OctomapBuilder::Run() {
    while (1) {
        unique_lock<mutex> lock(mMutexUpdate);
        cvUpdate.wait(lock, [&] {
                if (this->hasUpdate) {
                    this->hasUpdate = false;
                    return true;
                } else {
                    return false;
                }
            });
        // Update the octomap.
        Eigen::Matrix4f T_wc_eig = Converter::toMatrix4f(T_wc_mat);
        Eigen::Quaternionf q_wc(T_wc_eig.topLeftCorner<3,3>());

        octomap::pose6d T_wc_octo(
                octomap::point3d(
                        T_wc_eig(0,3), T_wc_eig(1,3), 0), // for 3d case we need z coordinates
                octomath::Quaternion(
                        q_wc.w(), q_wc.x(), q_wc.y(), q_wc.z()));

        octomap::Pointcloud local_cloud;  
        for(int m=0; m<depth.rows; m++){
            for (int n=0; n<depth.cols; n++){
                float d = depth.at<float>(m,n);
                if(isnan(depth.at<float>(m,n))) d = 7;
                if(d < 0.05) continue;
                float z = d;
                
                float x = (float(n) - camera_cx) * z / camera_fx;
                float y = (float(m) - camera_cy) * z / camera_fy;
                //if(z > 8) z = 8;
                if( y > 0.1 || y < -1) continue;
                //y=0;
                Eigen::Vector4f hPt;
                hPt << x,y,z,1;
                Eigen::Vector4f hPt_w = T_wc_eig * hPt;
                local_cloud.push_back(hPt_w(0),hPt_w(1),0);
             }
        }
        //local_cloud.transform(T_wc_octo);
        octomath::Vector3 vec3 = T_wc_octo.trans();
        unique_lock<mutex> lock2(mMutexRequest);
        globalOctoMap->insertPointCloud(
                local_cloud, octomap::point3d(vec3.x(), vec3.y(), 0)); // for 3d case we need z coordinates
        globalOctoMap->updateInnerOccupancy();

        unsigned int maxDepth = globalOctoMap->getTreeDepth();

        // expand collapsed occupied nodes until all occupied leaves are at maximum depth
        /*
        vector<octomap::OcTreeNode*> collapsed_occ_nodes;
        do {
            collapsed_occ_nodes.clear();
            for (octomap::OcTree::iterator it = globalOctoMap->begin(); it != globalOctoMap->end(); ++it)
            {
            if(globalOctoMap->isNodeOccupied(*it) && it.getDepth() < maxDepth)
            {
                collapsed_occ_nodes.push_back(&(*it));
            }
            }
            for (vector<octomap::OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
            {
                globalOctoMap->expandNode(*it);
            }
            cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
        } while(collapsed_occ_nodes.size() > 0);*/

        lock2.unlock();
    }
}

// This function is called from Planning thread.
void OctomapBuilder::SendGetOctomapRequest() {
    unique_lock<mutex> lock(mMutexRequest);
    hasRequest = true;
}

// This function is called from Tracking thread.
void OctomapBuilder::UpdateOctomap(const cv::Mat &depth_, cv::Mat currPose_) {
    unique_lock<mutex> lock(mMutexUpdate);
    hasUpdate = true;
    depth_.convertTo(depth, CV_32FC1, depthFactor);
    currPose = currPose_.clone();
    T_wc_mat = currPose*T_bc;
    lock.unlock();
    cvUpdate.notify_one();
}

bool OctomapBuilder::calcOccupiedPoints() {
    //vector<vector<float>> occupiedPoints;
    unique_lock<mutex> lock(mMutexRequest);
    OccupiedPoints.clear();
    FreePoints.clear();
    for (auto it = globalOctoMap->begin(); it != globalOctoMap->end(); it++) {
        if (globalOctoMap->isNodeOccupied(*it)) {
            OccupiedPoints.push_back({it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()});
        }else
        {
            FreePoints.push_back({it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()});
        }
    }
    lock.unlock();
    return true;
}

vector<vector<float>> OctomapBuilder::getLowProbPoints() {

    return FreePoints;
}
vector<vector<float>> OctomapBuilder::getOccupiedPoints() {
    
        return OccupiedPoints;
}


void OctomapBuilder::genNeighborCoord(octomap::OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor) 
{
    occupiedNeighbor.clear();
    octomap::OcTreeKey neighbor_key;
    std::vector<int> dir = {0,1,2,3,6,7,8,9};
    for (int i = 0; i < 8; i++) 
    {
        lut->genNeighborKey(start_key, dir[i], neighbor_key);
        octomap::point3d query = globalOctoMap->keyToCoord(neighbor_key);
        occupiedNeighbor.push_back(query);

    }
}

void OctomapBuilder::findFrontier(){

    frontierCells.clear();
    frontierCells_store.clear();
    bool neighbFree = false; 
    bool neighbUnknown = false;

    std::vector<octomap::point3d> neighbor;

    for (auto it = globalOctoMap->begin(); it != globalOctoMap->end(); it++) {

        float x = it.getCoordinate().x();
        float y = it.getCoordinate().y();
        float z = it.getCoordinate().z();

        octomap::point3d cellPoint(x,y,z);
        octomap::OcTreeKey key;

        if(!globalOctoMap->coordToKeyChecked(cellPoint, key)){

            cout << "Error in search: [" << cellPoint << "] is out of OcTree bounds!" << endl;
            return;
            
        }

		//check point state: free/occupied
        octomap::OcTreeNode* node = globalOctoMap->search(key);      
        
        bool occupied = globalOctoMap->isNodeOccupied(node);

        if(!occupied){
            neighbFree = false;
            neighbUnknown = false;

            genNeighborCoord(key, neighbor);

            for (std::vector<octomap::point3d>::iterator iter = neighbor.begin();iter != neighbor.end(); iter++)
            {
                    octomap::point3d neipoint=*iter;

                    //check point state: free/unknown
                    octomap::OcTreeNode* node = globalOctoMap->search(neipoint);
                    if(node == NULL)
                        neighbFree=1;
                    else
                    {
                        if(!globalOctoMap->isNodeOccupied(node))
                            neighbUnknown=1;
                    }
            }
            if(neighbFree==1 && neighbUnknown==1)
            {
                frontierCells.insert(key);
            }            

        }

    }

    frontierCells_store = frontierCells;

}

vector<vector<float>> OctomapBuilder::getFrontier(){
    vector <vector<float>> frontier_vector;
    for(octomap::KeySet::iterator iter = frontierCells_store.begin(), end=frontierCells_store.end(); iter!= end; ++iter)
    {
           octomap::point3d cell = globalOctoMap->keyToCoord(*iter);
           frontier_vector.push_back({cell.x(),cell.y(),cell.z()});
    }
    return frontier_vector;    
}

vector<vector<float>> OctomapBuilder::getFrontierCenter(){
    vector <vector<float>> frontier_centers;
    for(octomap::KeySet::iterator iter = candidateCells.begin(), end=candidateCells.end(); iter!= end; ++iter)
    {
           octomap::point3d cell = globalOctoMap->keyToCoord(*iter);
           frontier_centers.push_back({cell.x(),cell.y(),cell.z()});
    }
    return frontier_centers;    
}



bool OctomapBuilder::findCenter(std::vector<octomap::OcTreeKey>& cluster, octomap::OcTreeKey& centerCell)
{
    //centerCell = cluster[0];
    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;
    for(size_t i = 0; i <  cluster.size(); i++){
        octomap::point3d cell = globalOctoMap->keyToCoord(cluster[i]);
        x_sum += cell.x();
        y_sum += cell.y();
        z_sum += cell.z();
    }

    if (cluster.size() == 0){
        std::cout << "cluster has zero size, something is wrong!" << std::endl;
        return false;
    }

    float x_mean = x_sum/cluster.size();
    float y_mean = y_sum/cluster.size();
    float z_mean = z_sum/cluster.size();

    octomap::point3d center(x_mean, y_mean, z_mean);

    bool success = globalOctoMap->coordToKeyChecked(center, centerCell);

    return success;

}

void OctomapBuilder::clusterFrontier()
{

    std::vector <octomap::OcTreeKey> frontier_vector;
    //preprocess put the frontier cells in frontiercells into a queue
    for(octomap::KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
    {
           frontier_vector.push_back(*iter);
    }

    std::queue<octomap::OcTreeKey> temp_queue;
    std::vector<std::vector<octomap::OcTreeKey> > cluster_gather;
    while(!frontier_vector.empty())
    {
        octomap::OcTreeKey f_cell;
        octomap::OcTreeKey temp_cell;
        f_cell = frontier_vector.front();
        frontier_vector.erase(frontier_vector.begin());
        temp_queue.push(f_cell);
        octomap::point3d fcell_point;
        fcell_point = globalOctoMap->keyToCoord(f_cell);
        std::vector<octomap::point3d> neighbor;
        std::vector<octomap::OcTreeKey> cluster;
        
        while(!temp_queue.empty())
        {
            temp_cell = temp_queue.front();
            temp_queue.pop();
            genNeighborCoord(temp_cell, neighbor) ;
            for (std::vector<octomap::point3d>::iterator iter = neighbor.begin();iter != neighbor.end(); iter++)
            {
                octomap::point3d neipoint=*iter;
                octomap::OcTreeKey nei_key;
                globalOctoMap->coordToKeyChecked(neipoint, nei_key);
                octomap::KeySet::iterator got = frontierCells.find(nei_key);
                if (got == frontierCells.end())
                    continue;
                temp_queue.push(nei_key);
                frontierCells.erase(nei_key);

                frontier_vector.clear();
                for(octomap::KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
                {
                       frontier_vector.push_back(*iter);
                }               
                
            }
            cluster.push_back(temp_cell);
        }
        // don't consider the cluster which is too small
        if(cluster.size() < 10)
            continue;
        cluster_gather.push_back(cluster);
    }

    candidateCells.clear();
    int cluster_size=0;
    for(std::vector<std::vector<octomap::OcTreeKey> >::iterator iter = cluster_gather.begin(); iter!= cluster_gather.end(); iter++)
    {
        cluster_size++;
        octomap::OcTreeKey center_cell;
        if(findCenter(*iter,center_cell)){
            //octomap::point3d cpoint;
            //cpoint = globalOctoMap->keyToCoord(center_cell);
            candidateCells.insert(center_cell);
        }

    }
}

}  // namespace ORB_SLAM
