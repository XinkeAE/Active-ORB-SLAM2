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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <math.h>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}
void MapDrawer::SetCurrentCollision(const std::vector<std::vector<float>> &bCollision)
{
    unique_lock<mutex> lock(mMutexCollision);
    mCollisionPts=bCollision;
    //std::cout<<"Octomap Updated in Drawer"<<std::endl;
}

void MapDrawer::DrawMapCollision()
{
    //const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    //set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    unique_lock<mutex> lock(mMutexCollision);

    if(mCollisionPts.empty())
        return;

    int rows = static_cast<int>(mCollisionPts.size());
    glPointSize(3*mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,1.0);

    for(int i=0; i<rows; i++)
    {   
        cv::Mat P_w=(cv::Mat_<float>(4,1) << mCollisionPts[i][0],
                    mCollisionPts[i][1],
                    mCollisionPts[i][2],
                    1);
        cv::Mat P_s=T_sw_mat*P_w;
        glVertex3f(P_s.at<float>(0,0),P_s.at<float>(1,0),P_s.at<float>(2,0));
    }
    glEnd();
}




void MapDrawer::SetCurrentLowProb(const std::vector<std::vector<float>> &bCollision)
{
    unique_lock<mutex> lock(mMutexlow);
    mLowPts=bCollision;
    //std::cout<<"Octomap Updated in Drawer"<<std::endl;
}

void MapDrawer::DrawMapLowProb()
{
    //const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    //set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    unique_lock<mutex> lock(mMutexlow);

    if(mLowPts.empty())
    {
        return;
    }

    int rows = static_cast<int>(mLowPts.size());
    glPointSize(3*mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(int i=0; i<rows; i++)
    {   
        cv::Mat P_w=(cv::Mat_<float>(4,1) << mLowPts[i][0],
                    mLowPts[i][1],
                    mLowPts[i][2],
                    1);
        cv::Mat P_s=T_sw_mat*P_w;
        glVertex3f(P_s.at<float>(0,0),P_s.at<float>(1,0),P_s.at<float>(2,0));
    }
    glEnd();
}

// visualize the frontier
void MapDrawer::SetCurrentFrontier(const std::vector<std::vector<float>> &frontier)
{
    unique_lock<mutex> lock(mMutexFrontier);
    mFrontiers=frontier;
}

void MapDrawer::DrawMapFrontier()
{
    unique_lock<mutex> lock(mMutexFrontier);

    if(mFrontiers.empty())
    {
        return;
    }

    int rows = static_cast<int>(mFrontiers.size());
    glPointSize(5*mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.5,0.1);

    for(int i=0; i<rows; i++)
    {   
        cv::Mat P_w=(cv::Mat_<float>(4,1) << mFrontiers[i][0],
                    mFrontiers[i][1],
                    mFrontiers[i][2],
                    1);
        cv::Mat P_s=T_sw_mat*P_w;
        glVertex3f(P_s.at<float>(0,0),P_s.at<float>(1,0),P_s.at<float>(2,0));
    }
    glEnd();
}

// visualize the frontier centers   
void MapDrawer::SetCurrentFrontierCenter(const std::vector<std::vector<float>> &frontierCenter)
{
    unique_lock<mutex> lock(mMutexFrontierCenter);
    mFrontierCenters=frontierCenter;
}


void MapDrawer::DrawMapFrontierCenter()
{
    unique_lock<mutex> lock(mMutexFrontierCenter);

    if(mFrontierCenters.empty())
    {
        return;
    }

    int rows = static_cast<int>(mFrontierCenters.size());
    glPointSize(9*mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.4,0.1,0.5);

    for(int i=0; i<rows; i++)
    {   
        cv::Mat P_w=(cv::Mat_<float>(4,1) << mFrontierCenters[i][0],
                    mFrontierCenters[i][1],
                    mFrontierCenters[i][2],
                    1);
        cv::Mat P_s=T_sw_mat*P_w;
        glVertex3f(P_s.at<float>(0,0),P_s.at<float>(1,0),P_s.at<float>(2,0));
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::SetCurrentPath(const std::vector<std::vector<double>> &bPath)
{
    unique_lock<mutex> lock(mMutexPath);
    mPath=bPath;
}

void MapDrawer::DrawPath()
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;
    int counter=0;
    {
        unique_lock<mutex> lock(mMutexCounter);
        counter=currPathCounter;
        //std::cout<<counter<<std::endl;
    }
    //std::cout<<counter<<std::endl;
    unique_lock<mutex> lock(mMutexPath);
    cv::Mat T_sc_prev;
    //std::cout<<__LINE__<<std::endl;
    //std::cout<<(0<counter)<<std::endl;
    if(!mPath.empty()){
        for(int i=0; i<counter; i++)
        {
            std::vector<float> node= {float(mPath[i][0]),float(mPath[i][1]),float(mPath[i][2])};

            cv::Mat T_wb= (cv::Mat_<float>(4,4)<< cos(node[2]), -sin(node[2]),0,node[0],
                                                sin(node[2]),cos(node[2]), 0, node[1],
                                                0 , 0, 1 ,0,
                                                0,  0,  0,  1);

            cv::Mat T_sc=((T_ws_mat.inv())*T_wb*(T_cb_mat.inv())).t();

            glPushMatrix();

            glMultMatrixf(T_sc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,0.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        
            if(i!=0)
            {
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.5f,0.0f,0.5f);
                glBegin(GL_LINES);
                    glVertex3f(T_sc_prev.at<float>(3,0),T_sc_prev.at<float>(3,1),T_sc_prev.at<float>(3,2));
                    glVertex3f(T_sc.at<float>(3,0),T_sc.at<float>(3,1),T_sc.at<float>(3,2));
                glEnd();
            }
            T_sc_prev=T_sc;
        }

        // plot current goal

        std::vector<float> node= {float(mPath[counter][0]),float(mPath[counter][1]),float(mPath[counter][2])};
        
        cv::Mat T_wb= (cv::Mat_<float>(4,4)<< cos(node[2]), -sin(node[2]),0,node[0],
                                            sin(node[2]),cos(node[2]), 0, node[1],
                                            0 , 0, 1 ,0,
                                            0,  0,  0,  1);

        cv::Mat T_sc=((T_ws_mat.inv())*T_wb*(T_cb_mat.inv())).t();

        glPushMatrix();

        glMultMatrixf(T_sc.ptr<GLfloat>(0));

        glLineWidth(mKeyFrameLineWidth*2);
        glColor3f(1.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    
        if(counter!=0)
        {
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,1.0f,0.0f);
            glBegin(GL_LINES);
                glVertex3f(T_sc_prev.at<float>(3,0),T_sc_prev.at<float>(3,1),T_sc_prev.at<float>(3,2));
                glVertex3f(T_sc.at<float>(3,0),T_sc.at<float>(3,1),T_sc.at<float>(3,2));
            glEnd();

            glBegin(GL_LINE_LOOP);
            for(int ii = 0; ii < 300; ii++) 
            { 
                float theta = 2.0f * 3.1415926f * float(ii) / float(300);//get the current angle 
                glVertex3f( 0.12 * sinf(theta)+T_sc.at<float>(3,0), T_sc.at<float>(3,1), 0.12 * cosf(theta)+T_sc.at<float>(3,2));//output vertex 
        
            } 
            glEnd();
        }
        T_sc_prev=T_sc;

        for(int i=counter+1; i<mPath.size(); i++)
        {
            std::vector<float> node= {float(mPath[i][0]),float(mPath[i][1]),float(mPath[i][2])};

            cv::Mat T_wb= (cv::Mat_<float>(4,4)<< cos(node[2]), -sin(node[2]),0,node[0],
                                                sin(node[2]),cos(node[2]), 0, node[1],
                                                0 , 0, 1 ,0,
                                                0,  0,  0,  1);

            cv::Mat T_sc=((T_ws_mat.inv())*T_wb*(T_cb_mat.inv())).t();

            glPushMatrix();

            glMultMatrixf(T_sc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(1.0f,0.8f,0.35f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        
            if(i!=0)
            {
                glLineWidth(mKeyFrameLineWidth);
                glPushAttrib(GL_ENABLE_BIT); 
                glLineStipple(1, 0xAAAA); 
                glEnable(GL_LINE_STIPPLE);
                glColor3f(0.5f,0.5f,0.5f);
                glBegin(GL_LINES);
                    glVertex3f(T_sc_prev.at<float>(3,0),T_sc_prev.at<float>(3,1),T_sc_prev.at<float>(3,2));
                    glVertex3f(T_sc.at<float>(3,0),T_sc.at<float>(3,1),T_sc.at<float>(3,2));
                glEnd();
                glPopAttrib();

                 
                 
            }
            T_sc_prev=T_sc;
        }
    }
}



void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::SetCurrentCounter(int counter)
{
    unique_lock<mutex> lock(mMutexCounter);
    currPathCounter=counter;
    //std::cout<<currPathCounter<<std::endl;
}

} //namespace ORB_SLAM
