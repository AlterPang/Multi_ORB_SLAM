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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    // 存储用于画图的Frame信息
    // 包括：图像 特征点连线形成的轨迹（初始化时） 框（跟踪时的MapPoint） 圈（跟踪时的特征点）
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0)); //L177: mImGray1.copyTo(mIm)
    mIm_cam2 = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0)); //L177: mImGray1.copyTo(mIm)
}

// 准备需要显示的信息，包括图像、状态、其它的提示
cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    cv::Mat im_cam2;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    // 将成员变量赋值给局部变量，加互斥锁
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im); // 这里使用深拷贝是因为后面会把单通道灰度图像转为3通道图像;深拷贝后的拷贝对象和源对象互相独立
        mIm_cam2.copyTo(im_cam2);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap; //大小为N_total的bool
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
    {
        cvtColor(im, im, CV_GRAY2BGR); //im转为BGR
        cvtColor(im_cam2,im_cam2,CV_GRAY2BGR);
    }
    //Draw
    // 当前帧的特征坐标与初始帧的特征点坐标连成线，形成轨迹
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size(); //Note 大小为N_total
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //画矩形框, 绿色，表示在MapPoint对应的特征点
                    if(i<N_cam1){
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1); //画圈?
                        mnTracked++;
                    }
                    else{ //NOTE 相机2的视频
                        cv::rectangle(im_cam2,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im_cam2,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1); //画圈?
                        mnTracked++;
                    }

                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    // 蓝色，表示上一帧visual odometry产生的点对应的特征点
//                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
//                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
//                    mnTrackedVO++;
                    if(i<N_cam1){
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                        mnTrackedVO++;
                    }
                    else{ //NOTE 相机2的视频
                        cv::rectangle(im_cam2,pt1,pt2,cv::Scalar(255,0,0));
                        cv::circle(im_cam2,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                        mnTrackedVO++;
                    }
                }
            }
        }
    }

    //拼接两个视频
    cv::Mat im_total= cv::Mat(480,1280,CV_8UC3, cv::Scalar(0,0,0)); //Mat类是(行数,列数)
    cv::Mat ROI_im1=im_total(cv::Rect(0,0,640,480)); //Rect是(x坐标,y坐标,宽,高)
    cv::Mat ROI_im2=im_total(cv::Rect(640,0,640,480));
    im.copyTo (ROI_im1);
    im_cam2.copyTo (ROI_im2); //cam2数据来源于track,所以还没有???


    cv::Mat imWithInfo;
//    DrawTextInfo(im,state, imWithInfo);
    DrawTextInfo(im_total,state, imWithInfo);

    return imWithInfo; //返回带文字信息和特征框的imgray1
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type()); //imText的行列数和类型
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type()); //imText下方添加文字
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker) //更新framedrawer
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    pTracker->mImGray2.copyTo(mIm_cam2); //track 里还没有cam2数据??
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys_total;//plc
    N_cam1 = pTracker->mCurrentFrame.N;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])//i点是内点?
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
