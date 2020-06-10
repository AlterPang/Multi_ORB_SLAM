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

#include<opencv2/core/core.hpp>

#include<System.h>

//plc
#include "ctime"
#include "time.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)//用法: 0程序位置,1字典位置,2yaml文件位置,3序列集位置,4联系文件一位置,5联系文件二,6标定文件(新增)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association1 path_to_association2 path_to_calibration" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<string> vstrImageFilenamesRGB2;
    vector<string> vstrImageFilenamesD2;
    vector<double> vTimestamps;
    vector<double> vTimestamps2;//其实没用到,因为用的一个时间戳
    string strAssociationFilename = string(argv[4]);
    string strAssociationFilename2 = string(argv[5]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    LoadImages(strAssociationFilename2, vstrImageFilenamesRGB2, vstrImageFilenamesD2, vTimestamps2);//联系文件二里的图片名

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();

    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    if(vstrImageFilenamesRGB2.empty())
    {
        cerr << endl << "No images found in provided path2." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD2.size()!=vstrImageFilenamesRGB2.size())
    {
        cerr << endl << "Different number of images for rgb2 and depth2." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],argv[6],ORB_SLAM2::System::RGBD,true);//增加的argv[6]是标定文件

    // Vector for tracking time statistics
    // 跟踪时间统计
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imRGB2, imD2;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file (argv[3]是序列集位置)
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRGB2 = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB2[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD2 = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD2[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];//时间戳,两相机同一个

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }
        if(imRGB2.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB2[ni] << endl;
            return 1;
        }
        //下面是没有相机2序列集时用的
//        cv::Mat imRGB2 =imRGB.clone();
//        cv::Mat imD2(imD.rows,imD.cols,imD.type(),-1);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
//        SLAM.TrackRGBD(imRGB,imD,tframe);
        SLAM.TrackRGBD(imRGB,imD,imRGB2,imD2,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    usleep(5000);
//    while (1){}

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory

    //plc
    time_t t=std::time(0);
    struct tm * now = std::localtime( & t );
    string savetime=to_string(now->tm_year + 1900)+
                     '-'+to_string(now->tm_mon + 1)+
                     '-'+to_string(now->tm_mday)+
                     '-'+to_string(now->tm_hour)+
                     '-'+to_string(now->tm_min)+
                     '-'+to_string(now->tm_sec);

//    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("Multi_"+savetime+"_CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("Multi_"+savetime+"_KeyFrameTrajectory.txt");

    return 0;
}

//读取图片函数
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());//打开联系文件
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);//读取一行
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
