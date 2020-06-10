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

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){} //构造函数对成员mpSLAM初始化

    //将Ros消息转化为Opencv 格式(RGB和深度) //&表示msgRGB, msgD都是ImageConstPtr类型的常引用
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1,const sensor_msgs::ImageConstPtr& msgD1,
                  const sensor_msgs::ImageConstPtr& msgRGB2,const sensor_msgs::ImageConstPtr& msgD2);

    ORB_SLAM2::System* mpSLAM;
};
//plc
string int2string(int value);

int main(int argc, char **argv)//argc是变量个数(包括程序本身)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ORB_SLAM2::System SLAM(argv[1],argv[2],argv[3],ORB_SLAM2::System::RGBD,true);//加上标定文件

    //定义捕获图像的对象
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;  //定义节点句柄

    //Subscriber滤波器作为ROS消息的最顶层，不能够将其他滤波器的输出作为其输入
    //从/camera/rgb/image_raw 和 depth/image_raw 订阅
    message_filters::Subscriber<sensor_msgs::Image> rgb1_sub(nh, "/camera_01/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth1_sub(nh, "/camera_01/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb2_sub(nh, "/camera_02/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth2_sub(nh, "/camera_02/depth/image_raw", 1);
    //  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    //下边两条指令是为了将rgb_sub和depth_sub进行数据对齐，滤掉不对齐的帧数
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb1_sub,depth1_sub, rgb2_sub,depth2_sub);
    //采用registerCallback()方法，注册回调函数ImageGrabber::GrabRGBD
    cout<<"ros_rgbd.cc"<<endl;
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3,_4));  //_1,_2表示占位符,分别用第1,2个参数代替

    //此函数用于ROS不断回调节点imageGrabber::GrabRGBD函数
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    time_t t=std::time(0);
    struct tm * now = std::localtime( & t );
    string time=int2string(now->tm_year + 1900)+
                '-'+int2string(now->tm_mon + 1)+
                '-'+int2string(now->tm_mday)+
                '-'+int2string(now->tm_hour)+
                '-'+int2string(now->tm_min)+
                '-'+int2string(now->tm_sec);

    SLAM.SaveTrajectoryTUM(time+"_CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(time+"_KeyFrameTrajectory.txt");

    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1,const sensor_msgs::ImageConstPtr& msgD1,
                            const sensor_msgs::ImageConstPtr& msgRGB2,const sensor_msgs::ImageConstPtr& msgD2)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB1;
    try
    {
        cv_ptrRGB1 = cv_bridge::toCvShare(msgRGB1);  //将ROS RGB消息转化为Opencv支持的RGB格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD1;
    try
    {
        cv_ptrD1 = cv_bridge::toCvShare(msgD1);  //将ROS深度消息转化为Opencv支持的深度格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRGB2;
    try
    {
        cv_ptrRGB2 = cv_bridge::toCvShare(msgRGB2);  //将ROS RGB消息转化为Opencv支持的RGB格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD2;
    try
    {
        cv_ptrD2 = cv_bridge::toCvShare(msgD2);  //将ROS深度消息转化为Opencv支持的深度格式
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


//    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    mpSLAM->TrackRGBD(cv_ptrRGB1->image,cv_ptrD1->image,
                      cv_ptrRGB2->image,cv_ptrD2->image,
                      cv_ptrRGB1->header.stamp.toSec());
}

string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}


