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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include <cv.h>


using namespace std;
// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyPoint数据类型

namespace ORB_SLAM2
{
// strSettingPath 是yaml文件
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB,
                   const string &strSettingPath, const int sensor,const cv::Mat CalibMatrix):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
    mCaliMatrix(CalibMatrix.clone())//两相机标定的变换矩阵.todo 用地址*吗???

{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //K:摄像机内参矩阵
    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    cv::Mat K = cv::Mat::eye(3,3,CV_32F); //eye返回一个恒等指定大小和类型矩阵。
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);//从外部文件得到的mK等变量通过Frame(...)传给其他类

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];//金字塔层数: TUM1.yaml里是8
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mpORBextractorLeft_cam2 = new ORBextractor(nFeatures/2,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];  //TUM1.yaml 里是5000.0,表示5000单位为真实世界里的1米
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor; //0.0002
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


//cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
//{
//    mImGray1 = imRectLeft; //双目,忽略
//    cv::Mat imGrayRight = imRectRight;
//
//    if(mImGray1.channels()==3)  //3通道rgb 或者4通道rgba(alpha通道)
//    {
//        if(mbRGB)  //rgb或gbr格式
//        {
//            cvtColor(mImGray1,mImGray1,CV_RGB2GRAY);
//            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
//        }
//        else
//        {
//            cvtColor(mImGray1,mImGray1,CV_BGR2GRAY);
//            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
//        }
//    }
//    else if(mImGray1.channels()==4)
//    {
//        if(mbRGB)
//        {
//            cvtColor(mImGray1,mImGray1,CV_RGBA2GRAY);
//            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
//        }
//        else
//        {
//            cvtColor(mImGray1,mImGray1,CV_BGRA2GRAY);
//            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
//        }
//    }
//
//    mCurrentFrame = Frame(mImGray1,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth); //双目,忽略
//
//    Track();
//
//    return mCurrentFrame.mTcw.clone();
//}

// 输入左目RGB或RGBA图像和深度图
// 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB1,const cv::Mat &imD1,
                                const cv::Mat &imRGB2,const cv::Mat &imD2,
                                const double &timestamp)
{
    mImGray= imRGB1;
    mImGray1 = imRGB1;
    mImGray2 = imRGB2;
//    cout<<"Tracking.cc::GrabImageRgbd"<<endl;
//    cv::Mat imDepth = imD1;
    cv::Mat imDepth1 = imD1;
    cv::Mat imDepth2 = imD2;
    // 步骤1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)  //3通道rgb 或者4通道rgba(alpha通道)
    {
        if(mbRGB)   //rgb或gbr格式
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(mImGray2,mImGray2,CV_RGB2GRAY);
        }
        else
            {
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
                cvtColor(mImGray2,mImGray2,CV_BGR2GRAY);
            }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(mImGray2,mImGray2,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(mImGray2,mImGray2,CV_BGRA2GRAY);
        }
    }

    // 步骤2：将深度相机的disparity转为Depth
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth1.type()!=CV_32F)  //TUM1.yaml里是5000.0,作倒数为0.0002
    {
        //若原尺度因子不为0,且图片类型非32f,则做转换
        imDepth1.convertTo(imDepth1,CV_32F,mDepthMapFactor);
        imDepth2.convertTo(imDepth2,CV_32F,mDepthMapFactor);
    }
//    cout<<"Tracking.cc::即将构建Frame"<<endl;
    // 步骤3：构造Frame //灰度图,深度图,时间戳,orb提取器,orb词典?,摄像机内参矩阵mk,
    //mpORBextractorLeft 在tracking.h 和frame.h 中是同名类
    //所有帧都在这里构造(关键帧也是基于这里构造) //@mThDepth 深度阈值,小于它的直接生成点,大于的要用两帧匹配来生成
    mCurrentFrame = Frame(mImGray,imDepth1,mImGray2,imDepth2,timestamp,
                          mpORBextractorLeft,mpORBextractorLeft_cam2,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mCaliMatrix);

    //生成Frame后开始跟踪
    // 步骤4：跟踪
//    cout << "Tracking.cc::L290: 构建Frame后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
//    cout<<"Tracking.cc::已构建Frame,开始Track()"<<endl;
    // note 计算跟踪时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Track();
    // note 计算建图时间
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tlba= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"局部建图成功，花费时间为： "<<tlba<<endl;

    return mCurrentFrame.mTcw.clone();
}


//cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
//{
//    mImGray = im;
//
//    if(mImGray.channels()==3)
//    {
//        if(mbRGB)
//            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
//        else
//            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
//    }
//    else if(mImGray.channels()==4)
//    {
//        if(mbRGB)
//            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
//        else
//            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
//    }
//
//    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
//        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,m,mDistCoef,mbf,mThDepth);
//    else
//        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
//
//    Track();
//
//    return mCurrentFrame.mTcw.clone();
//}

/**
 * @brief Main tracking function. It is independent of the input sensor.
 *
 * Tracking 线程
 */
void Tracking::Track()
{
    // track包含两部分：估计运动、跟踪局部地图

    // mState为tracking的状态机
    // SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // 步骤1：初始化
    if(mState==NOT_INITIALIZED)
    {
//        cout<<"Tracking.cc::L345 Track(): 开始初始化"<<endl;
        if(mSensor==System::STEREO || mSensor==System::RGBD)
        {
//            cout<<"Tracking.cc:: L348 Track(): RGBD初始化"<<endl;
            StereoInitialization();
        }
        else
        {
//            cout<<"Tracking.cc::L355 Track(): 单目初始化"<<endl;
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);  //更新framedrawer

        if(mState!=OK)
        {
//            cout<<"Tracking.cc::L359 Track(): RGBD初始化失败"<<endl;
            return;
        }
    }
    else // 步骤2：已初始化,跟踪
    {
//        cout<<"Tracking.cc::L363 Track(): 开始跟踪..."<<endl;
        // System is initialized. Track Frame.
        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // 在viewer中有个开关menuLocalizationMode，有它控制是否ActivateLocalizationMode，并最终管控mbOnlyTracking
        // mbOnlyTracking等于false表示正常VO模式（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式
        if(!mbOnlyTracking) //mbOnlyTracking=false,(默认)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            // 正常初始化成功
            if(mState==OK)
            {
//                cout<<"Tracking.cc::L387 正常初始化成功"<<endl;
                // Local Mapping might have changed some MaUpdateLastFramepPoints tracked in last frame
                // 检查并更新上一帧被替换的MapPoints
                // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                CheckReplacedInLastFrame();
//                cout << "Tracking.cc::L394: 替换后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

//                cout<<"Tracking.cc::L393 跟踪参考帧或者运动模型"<<endl;
                // 步骤2.1：跟踪上一帧或者参考帧或者重定位

                // 运动模型是空的或刚完成重定位
                // mCurrentFrame.mnId<mnLastRelocFrameId+2这个判断不应该有
                // 应该只要mVelocity不为空，就优先选择TrackWithMotionModel
                // mnLastRelocFrameId上一次重定位的那一帧
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2) // mVelocity = mCurrentFrame.mTcw*LastTwc;
                {
//                    cout<<"Tracking.cc::L402 跟踪参考帧..."<<endl;
                    // 将上一帧的位姿作为当前帧的初始位姿
                    // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame(); //plc cam2 未优化
                }
                else //运动模型不为空
                {
//                    cout<<"Tracking.cc::L413 通过恒速模型跟踪..."<<endl;
                    // 根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                    {
//                        cout<<"Tracking.cc::L420 恒速模型跟踪失败,跟踪参考帧..."<<endl;
                        // TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动速度模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                        // 最后通过优化得到优化后的位姿
                        bOK = TrackReferenceKeyFrame();
                    }
                }
            }
            else//mState不ok
            {
//                cout<<"Tracking.cc::L412 重定位"<<endl;
                // BOW搜索，PnP求解位姿
                bOK = Relocalization();///////todo 这里还没看****************
            }
        }
        else //mbOnlyTracking=true,只跟踪 todo 还没看
        {
            // Localization Mode: Local Mapping is deactivated
            // 只进行跟踪tracking，不插入关键帧,局部地图不工作

            // 步骤2.1：跟踪上一帧或者参考帧或者重定位
            if(mState==LOST)// tracking跟丢了
            {
//                cout<<"Tracking.cc::L428 (仅跟踪) 重定位..."<<endl;
                // BOW搜索，PnP求解位姿
                bOK = Relocalization(); //仅跟踪
            }
            else//未跟丢
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常，
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
//                        cout<<"Tracking.cc::L443 (仅跟踪) 运动模型..."<<endl;
                        bOK = TrackWithMotionModel(); //plc cam2的点未优化
                    }
                    else
                    {
//                        cout<<"Tracking.cc::L448 (仅跟踪) 跟踪参考帧..."<<endl;
                        bOK = TrackReferenceKeyFrame();//(仅跟踪)
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为1，则表明此帧匹配了很少的3D map点，少于10个，要跪的节奏，既做跟踪又做定位
                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();  //plc cam2的点未优化
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization(); //仅跟踪

                    // 重定位没有成功，但是跟踪成功
                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++) //plc
//                            for(int i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

//        cout << "Tracking.cc::L515: 现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
        // 将最新的关键帧作为reference frame
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // 步骤2.2：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        // 在步骤2.1中主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
        // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)//跟踪+建图
        {
//            cout<<"Tracking.cc::L527 (跟踪+建图) 跟踪局部地图..."<<endl;

            if(bOK)//上面跟踪成功
                bOK = TrackLocalMap();  //cam2呢
//            else
//                cout<<"Tracking.cc::L527 前面跟踪失败,不进行跟踪局部地图..."<<endl;
        }
        else//仅跟踪
        {
//            cout<<"Tracking.cc::L534 (仅跟踪) 跟踪局部地图..."<<endl;
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.

            // 重定位成功
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)//跟踪局部地图成功
            mState = OK;
        else
            mState=LOST;//初始跟踪失败或者初始跟踪成功但是跟踪局部地图失败

//        cout<<"Tracking.cc::L549 更新drawer..."<<endl;
        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)//跟踪局部地图成功
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                // 步骤2.3：更新恒速运动模型TrackWithMotionModel中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

//            cout<<"Tracking.cc::L570 清除临时添加的MapPoints..."<<endl;
            // Clean VO matches
            // 步骤2.4：清除UpdateLastFrame中为当前帧临时添加的MapPoints
            for(int i=0; i<mCurrentFrame.N_total; i++) //plc
//            for(int i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // 步骤2.5：清除临时的MapPoints，这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
            // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
            mlpTemporalPoints.clear();

//            cout << "Tracking.cc::L597: 清除临时地图点后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

            // Check if we need to insert a new keyframe
            // 步骤2.6：检测并插入关键帧，对于双目会产生新的MapPoints
            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
//                cout << "Tracking.cc::L603: 插入关键帧后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 删除那些在bundle adjustment中检测为outlier的3D map点
//            for(int i=0; i<mCurrentFrame.N;i++)
            for(int i=0; i<mCurrentFrame.N_total;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
//            cout << "Tracking.cc::L618: 剔除outliers后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)//跟踪失败 (初始跟踪失败 或者初始跟踪成功但跟踪局部地图失败)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
        {
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
        // 保存上一帧的数据
        mLastFrame = Frame(mCurrentFrame); //当前帧作为下一循环的上一帧
//        cout<<"Tracking.cc::L641 保存为上一帧,mLastFrame = Frame(mCurrentFrame)..."<<endl;
    }//跟踪结束

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // 步骤3：记录位姿信息，用于轨迹复现  (若是第一帧,则初始化后直接到这步)
//    cout<<"Tracking.cc::L623 记录位姿信息..."<<endl;
    if(!mCurrentFrame.mTcw.empty())
    {
        // 计算相对姿态T_currentFrame_referenceKeyFrame
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        // 如果跟踪失败，则相对位姿使用上一次值
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


 // 双目和rgbd的地图初始化
 // 由于具有深度信息，直接生成MapPoints
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
//        mCurrentFrame.mvKeysRays.reserve(mCurrentFrame.N);
        // Set Frame pose to the origin
        // 步骤1：设定初始位姿
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F)); //保存在mTcw,并调用UpdatePoseMatrices()

        // Create KeyFrame
        // 步骤2：将当前帧构造为初始关键帧 pKFini:KeyFrameinitial
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW. (最后一个是KeyFrameDatabase)
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);  //KeyFrame.cc

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        // 步骤3：在地图中添加该初始关键帧
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        // 步骤4：为每个特征点构造MapPoint;(初始帧)
        vector<vector<cv::Vec3f>> keyRaysTemp(2);
        keyRaysTemp[0].resize(mCurrentFrame.N);
        keyRaysTemp[1].resize(mCurrentFrame.N_cam2);
//        cout<<"Tracking.cc L659 :: 每个特征点构造MapPoint"<<endl;
        for(int i=0; i<mCurrentFrame.N;i++)  //N:关键点数量
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
//                int cam = mCurrentFrame.keypoint_to_cam.find(i)->second;
                // 步骤4.1：通过反投影得到该特征点的3D坐标
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);//初始化得到的地图点//纵向(3x1)
                //获得特征点坐标
//                float x3Dx, x3Dy, x3Dz;
//                if (x3D.at<float>(0,0) || x3D.at<float>(1,0)|| x3D.at<float>(2,0))
//                {
//                    x3Dx=x3D.at<float>(0,0);
//                    x3Dy=x3D.at<float>(1,0);
//                    x3Dz=x3D.at<float>(2,0);
//                }
//                keyRaysTemp[0][i]=cv::Vec3f(x3Dx, x3Dy, x3Dz);
//                mCurrentFrame.mvKeysRays[i]=cv::Vec3f(x3Dx, x3Dy, x3Dz);//特征点3d(世界)坐标?
//                mCurrentFrame.mvKeysRays.push_back(keyRaysTemp[0][i]); //特征点3d(世界)坐标?
                // 步骤4.2：将3D点构造为MapPoin
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap); //pKFini是初始关键帧
                // 步骤4.3：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围
//                cout<<"Tracking.cc L680 "<<endl;
                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                pNewMP->AddObservation(pKFini,i);
                // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                pKFini->AddMapPoint(pNewMP,i);
                // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
//                cout<<"Tracking.cc L686 "<<endl;
                pNewMP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范围
//                cout<<"Tracking.cc L736 :: UpdateNormalAndDepth() i="<<i<<endl;
                pNewMP->UpdateNormalAndDepth();
                // 步骤4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);//插入到mspMapPoints变量

//                cout<<"Tracking.cc L693 "<<endl;
                // 步骤4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }
//        cout<<"Tracking.cc L709 :: 为相机2的每个特征点构造MapPoint"<<endl;
        //cam_2
        for(int i=0; i<mCurrentFrame.N_cam2;i++)  //N:关键点数量
        {
            float z = mCurrentFrame.mvDepth_cam2[i];
            if(z>0)
            {
                // 步骤4.1：通过反投影得到该特征点的3D坐标
                cv::Mat x3D = mCurrentFrame.UnprojectStereo_cam2(i);
                //获得特征点坐标
//                float x3Dx, x3Dy, x3Dz;
//                x3Dx=x3D.at<float>(0,0);
//                x3Dy=x3D.at<float>(1,0);
//                x3Dz=x3D.at<float>(2,0);
//                std::vector<cv::Vec3d>> keyRaysTemp;
//                keyRaysTemp[1].resize(mCurrentFrame.N_cam2);
//                keyRaysTemp[1][i]=cv::Vec3f(x3Dx, x3Dy, x3Dz);
//                mCurrentFrame.mvKeysRays[(mCurrentFrame.N)+i]=cv::Vec3f(x3Dx, x3Dy, x3Dz);//特征点3d(世界)坐标?
//                mCurrentFrame.mvKeysRays.push_back(keyRaysTemp[1][i]);;//特征点3d(世界)坐标?
                // 步骤4.2：将3D点构造为MapPoin
//                int cam_2=1;
//                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap,cam_2); //todo 摄像机2的mp 这里??
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap); //摄像机2的mp
//                // 步骤4.3：为该MapPoint添加属性：
//                // a.观测到该MapPoint的关键帧
//                // b.该MapPoint的描述子
//                // c.该MapPoint的平均观测方向和深度范围
//
//                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                pNewMP->AddObservation(pKFini,i + mCurrentFrame.N);
//                // 步骤4.5：表示该初始KeyFrame的哪个特征点可以观测到哪个3D点
                pKFini->AddMapPoint(pNewMP,i + mCurrentFrame.N);
//                // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
                pNewMP->ComputeDistinctiveDescriptors();
//                // c.更新该MapPoint平均观测方向以及观测距离的范围
                pNewMP->UpdateNormalAndDepth();
//                // 步骤4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);//插入到mspMapPoints变量

                // 步骤4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mCurrentFrame.mvpMapPoints[(mCurrentFrame.N)+i]=pNewMP;
            }
        }

//        for(int i=0; i<mCurrentFrame.N_cam2;i++)
//        {
//            int temp=(mCurrentFrame.N)+i;
//            mCurrentFrame.mvpMapPoints[temp]=mCurrentFrame.mvpMapPoints_cam2[i];
//        }

        cout << "Tracking.cc::L759: New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);//双目初始化后插入的帧(仅一次)

        mLastFrame = Frame(mCurrentFrame);  //当前帧作为下一循环的上一帧
        mnLastKeyFrameId=mCurrentFrame.mnId;//用来判断距该关键帧间隔多少帧
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;//rgbd初始化成功
//        cout<<"Tracking.cc L781 ::StereoInitialization(): RGBD初始化完毕"<<endl;
    }
}
/**
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 */
void Tracking::MonocularInitialization()
  {
      cout<<"Tracking.cc L798 ::单目初始化..."<<endl;

      if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);  //当前帧作为下一循环的上一帧
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

/**
 * @brief CreateInitialMapMonocular
 *
 * 为单目摄像头三角化生成MapPoints
 */
void Tracking::CreateInitialMapMonocular()//仅单目
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20); //单目初始化用到的全局BA

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);//单目
    mpLocalMapper->InsertKeyFrame(pKFcur);//单目

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;//单目
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

/**
 * @brief 替换上一帧中的某些MapPoints
 *
 * Local Mapping 线程可能会修改上一帧的某些MapPoints
 */
void Tracking::CheckReplacedInLastFrame()
{
//    cout<<"Tracking.cc L994 ::CheckReplacedInLastFrame() 替换上一帧的一些点"<<endl;
//    cout<<"Tracking.cc L994 ::上一帧的地图点个数为"<<mLastFrame.mvpMapPoints.size()<<endl;
//    cout<<"Tracking.cc L994 :: mLastFrame.N_total数为"<<mLastFrame.N_total<<endl;

    for(int i =0; i<mLastFrame.N_total; i++) //mlastFrame即上一次循环中的mCurrentFrame
    {
//        if( i>(mLastFrame.N_total-20) )
//        {
//            cout<<"L1018遍历第"<<i<<"个点"<<endl;
//        }
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
//            if( i>(mLastFrame.N_total-20) )
//            {
//            cout<<"L1018遍历第"<<i<<"个点,坐标是:"<<pMP->GetWorldPos()<<endl;
//            }
            //函数返回 mpReplaced, 而mpReplaced 为LocalMapping线程中函数Replace(pMP)中的参数pMP.
            //即 替换成点pRep
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/**
 * @brief 对参考关键帧的MapPoints进行跟踪
 *
 * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
 * 2. 对属于同一node的描述子进行匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 */
bool Tracking::TrackReferenceKeyFrame()//参考帧即最近的关键帧
{
//    cout<<"Tracking.cc::L997 TrackReferenceKeyFrame()..."<<endl;
    // Compute Bag of Words vector
    // 步骤1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW(); //得到mBowVec,mFeatVec (变量包括两相机)
    mCurrentFrame.ComputeBoW_cam1(); //得到mBowVec_cam1,mFeatVec_cam1 (变量包括两相机)

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

//    cout<<"Tracking.cc::L1045 bow搜索当前帧与参考帧的匹配..."<<endl;
    // 步骤2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    // 特征点的匹配关系由MapPoints进行维护 //todo
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches); //当前帧与参考帧之间的特征点匹配

//    cout<<"Tracking.cc::L1050: 通过BOW搜索到当前帧与参考关键帧有 <"<<nmatches<<"> 个匹配点"<<endl;

    if(nmatches<15)
        return false;
    // 步骤3:将上一帧的位姿态作为当前帧位姿的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;//大小为当前帧特征点数, 非匹配点为Null
    mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

//    cout<<"Tracking.cc::L1042 位姿优化..."<<endl;
    // 步骤4:通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame,true);
//    cout << "Tracking.cc::L1061: 位姿优化后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
//    cout<<"Tracking.cc::L1062: 跟踪参考关键帧得到 <"<<nmatches<<"> 个匹配点"<<endl;
//    cout<<"Tracking.cc::L1063: 位姿优化后当前帧位姿为 "<<endl<<mCurrentFrame.mTcw<<endl;

    // Discard outliers
    // 步骤5：剔除优化后的outlier匹配点（MapPoints） //cam2???
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N_total; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])//是outlier
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);//剔除
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)//该点不是outlier且有被观测到
                nmatchesMap++;
        }
    }

//    cout << "Tracking.cc::L1085: 剔除outliers后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
//    cout<<"Tracking.cc::L1086: 剔除outliers后得到 <"<<nmatches<<"> 个匹配点"<<endl;

    return nmatchesMap>=10;
}
/**
 * @brief 双目或rgbd摄像头根据深度值为上一帧(普通帧?)产生新的MapPoints
 *
 * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） \n
 * 可以通过深度值产生一些新的MapPoints
 */
void Tracking::UpdateLastFrame()
{
//    cout<<"Tracking.cc::L1066 UpdateLastFrame()..."<<endl;
    // Update pose according to reference keyframe
    // 步骤1：更新最近一帧的位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back(); // Tlr*Trw = Tlw 1:last r:reference w:world

    mLastFrame.SetPose(Tlr*pRef->GetPose());//得到Tcw, Rcw, Rwc, tcw, twc
//    cout <<"Tracking.cc::L1106: 上一帧位姿为"<< endl << mLastFrame.mTcw << endl;


    // 如果上一帧为关键帧，或者单目的情况，则退出
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // 步骤2：对于双目或rgbd摄像头，为上一帧临时生成新的MapPoints
    // 注意这些MapPoints不加入到Map中，在tracking的最后会删除
    // 跟踪过程中需要将将上一帧的MapPoints投影到当前帧可以缩小匹配范围，加快当前帧与上一帧进行特征点匹配

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // 步骤2.1：得到上一帧有深度值的特征点
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N_total);//plc
    for(int i=0; i<mLastFrame.N_total;i++)
    {
        float z = mLastFrame.mvDepth_total[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }
    //CAM2 暂时不需要
//    vector<pair<float,int> > vDepthIdx_cam2;
//    vDepthIdx_cam2.reserve(mLastFrame.N_cam2);
//    for(int i=0; i<mLastFrame.N_cam2;i++)
//    {
//        float z = mLastFrame.mvDepth_cam2[i];
//        if(z>0)
//        {
//            vDepthIdx_cam2.push_back(make_pair(z,i));
//        }
//    }

    if(vDepthIdx.empty())
        return;

    // 步骤2.2：按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());
//    sort(vDepthIdx_cam2.begin(),vDepthIdx_cam2.end()); //后续待完成

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // 步骤2.3：将距离比较近的点包装成MapPoints(临时)
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;//特征点编号

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            int cam = mLastFrame.keypoint_to_cam.find(i)->second;//特征点所在的相机编号
//            int descIdx = mLastFrame.cont_idx_to_local_cam_idx.find(i)->second;//特征点的局部编号
            // 这些生成MapPoints后并没有通过：
            // a.AddMapPoint、
            // b.AddObservation、
            // c.ComputeDistinctiveDescriptors、
            // d.UpdateNormalAndDepth添加属性，
            // 这些MapPoint仅仅为了提高双目和RGBD的跟踪成功率
//            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            cv::Mat x3D = mLastFrame.UnprojectStereo_camid(cam,i);//所在相机编号,特征点的总编号
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP; // 添加新的MapPoint

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)//深度大于阈值则break(因为之后的深度也必定大于阈值)
            break;
    }
}
/**
 * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
 *
 * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时）(用于跟踪)
 * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配                //这一步要对cam2做吧????
 * @return 如果匹配数大于10，返回true
 * @see V-B Initial Pose Estimation From Previous Frame
 */
bool Tracking::TrackWithMotionModel()
{
//    cout<<"Tracking.cc::L1159: TrackWithMotionModel()..."<<endl;
    ORBmatcher matcher(0.9,true);//原来是0.9

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // 步骤1：对于双目或rgbd摄像头，根据深度值为上一关键帧生成新的MapPoints(临时,在CreateNewKeyFrame前会全部删除)
    // （跟踪过程中需要将当前帧与上一帧进行特征点匹配，将上一帧的MapPoints投影到当前帧可以缩小匹配范围）
    // 在跟踪过程中，去除outlier的MapPoint，如果不及时增加MapPoint会逐渐减少
    // 这个函数的功能就是补充增加RGBD和双目相机上一帧的MapPoints数
    UpdateLastFrame(); //非单目情况下 todo 暂时只有相机1生成了地图点

    // 根据Const Velocity Model(认为这两帧之间的相对运动和之前两帧间相对运动相同)估计当前帧的位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
//    cout <<"Tracking.cc::L1219: 设定当前帧位姿为"<<endl << mCurrentFrame.mTcw << endl;

    //mvpMapPoints 大小为N_total(在frame中初始化的)
    //fill(vec.begin(), vec.end(), val) 原来容器中每个元素被重置为val
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;

//    cout<<"Tracking.cc::L1214 TrackWithMotionModel(): SearchByProjection..."<<endl;
    // 步骤2：根据匀速度模型进行对上一帧的MapPoints进行跟踪
    // 根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
    // todo mutli
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,
            mSensor==System::MONOCULAR, mCaliMatrix);

//    cout<<"Tracking.cc::L1220: 通过投影匹配到 <"<<nmatches<<"> 个点"<<endl;

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,
                mSensor==System::MONOCULAR, mCaliMatrix);
//        cout<<"Tracking.cc::L1222: 第一次匹配点数量过少, 重新通过投影匹配到 <"<<nmatches<<"> 个点"<<endl;
    }

    if(nmatches<20)
    {
//        cout<<"Tracking.cc::L1227: 第二次匹配点数量仍然过少, 跟踪失败"<<endl;
        return false;
    }

//    cout<<"Tracking.cc::L1237 TrackWithMotionModel(): 位姿优化"<<endl;
    // Optimize frame pose with all matches
    // 步骤3：优化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);//todo 单相机
//    cout << "Tracking.cc::L1243: 位姿优化后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

//    cout<<"Tracking.cc::L1242 :TrackWithMotionModel(): 剔除outliers"<<endl;
    // Discard outliers
    // 步骤4：优化位姿后剔除outlier的mvpMapPoints
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)//N_total?
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
//                cout<<"Tracking.cc::L1227 :第"<<i<<"个点是outliers,删去"<<endl;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

//    cout<<"Tracking.cc::L1317: 去除outliers后剩下 <"<<nmatches<<"> 个匹配点"<<endl;
//    cout<<"Tracking.cc::L1318: 当前帧上能被其他帧观测到的点有 <"<<nmatchesMap<<"> 个"<<endl;
//    cout << "Tracking.cc::L1276: 去除outliers后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
    return nmatchesMap>=10;
}

/**
 * @brief 对Local Map的MapPoints进行跟踪
 *
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 * @see V-D track Local Map
 */
bool Tracking::TrackLocalMap()
{
//    cout<<"Tracking.cc:: L1247 TrackLocalMap()..."<<endl;
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    // 步骤1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
    UpdateLocalMap();

//    cout<<"Tracking.cc:: L1255 TrackLocalMap() 搜索局部点..."<<endl;
    // 步骤2：在局部地图中查找与当前帧匹配的MapPoints
    SearchLocalPoints();

    // Optimize Pose
    // 在这个函数之前，在Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel中都有位姿优化，
    // 步骤3：更新局部所有MapPoints后对位姿再次优化
    Optimizer::PoseOptimization(&mCurrentFrame);
//    cout << "Tracking.cc::L1307: 位姿优化后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
    mnMatchesInliers = 0;

//    cout<<"Tracking.cc:: L1304 TrackLocalMap() 更新地图点的观测..."<<endl;
    // Update MapPoints Statistics
    // 步骤3：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
//    for(int i=0; i<mCurrentFrame.N; i++)//怎么改cam2 ?
    for(int i=0; i<mCurrentFrame.N_total; i++)//怎么改cam2 ?
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1
            if(!mCurrentFrame.mvbOutlier[i])//若该点不是outlier
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)//正常建图模式
                {
                    // 该MapPoint被其它关键帧观测到过
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else// 记录当前帧跟踪到的MapPoints，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)//该点是outlier
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // 步骤4：决定是否跟踪成功
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    {
//        cout<<"Tracking.cc:: L1336 TrackLocalMap失败..."<<endl;
        return false;
    }
    if(mnMatchesInliers<30)
    {
//        cout<<"Tracking.cc:: L1341 TrackLocalMap失败..."<<endl;
        return false;
    }
    else
    {
//        cout<<"Tracking.cc:: L1346 TrackLocalMap成功..."<<endl;
        return true;
    }
}

/**
 * @brief 判断当前帧是否为关键帧
 * @return true if needed
 */
bool Tracking::NeedNewKeyFrame()
{
    // 步骤1：如果用户在界面上选择重定位，那么将不插入关键帧
    // 由于插入关键帧过程中会生成MapPoint，因此用户选择重定位后地图上的点云和关键帧都不会再增加
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // 如果局部地图被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // 步骤2：判断是否距离上一次插入关键帧的时间太短
    // mCurrentFrame.mnId是当前帧的ID
    // mnLastRelocFrameId是最近一次重定位帧的ID
    // mMaxFrames等于图像输入的帧率
    // 如果关键帧比较少，则考虑插入关键帧
    // 或距离上一次重定位超过1s，则考虑插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    // 步骤3：得到参考关键帧跟踪到的MapPoints数量
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // 步骤4：查询局部地图管理器是否繁忙
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    // 步骤5：对于双目或RGBD摄像头，统计总的可以添加的MapPoints数量和跟踪到地图中的MapPoints数量
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    // 步骤6：决策是否需要插入关键帧
    // 设定inlier阈值，和之前帧特征点匹配的inlier比例
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f; // 关键帧只有一帧，那么插入关键帧的阈值设置很低

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // 很长时间没有插入关键帧
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // localMapper处于空闲状态
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    // 跟踪要跪的节奏，0.25和0.3是一个比较低的阈值
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 阈值比c1c要高，与之前参考帧（最近的一个关键帧）重复度不是太高
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

/**
 * @brief 创建新的关键帧
 *
 * 对于非单目的情况，同时创建新的MapPoints
 */
void Tracking::CreateNewKeyFrame()
{
//    cout<<"Tracking.cc::L1446 CreateNewKeyFrame()..."<<endl;
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // 步骤2：将当前关键帧设置为当前帧的参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 这段代码和UpdateLastFrame中的那一部分代码功能相同
    // 步骤3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // 步骤3.1：得到当前帧深度小于阈值的特征点
        // 创建新的MapPoint, depth < mThDepth
        //cam1
        vector<pair<float,int> > vDepthIdx;//<特征点深度z,其编号i>
        //  .reserve()是容器预留空间，但在空间内不真正创建元素对象，所以在没有添加新的对象之前，不能引用容器内的元素。
        // 加入新的元素时，要调用push_back()/insert()函数。
        vDepthIdx.reserve(mCurrentFrame.N);//cam1
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];  //只用到mvDepth和mvDepth_cam2要怎么改??
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i)); //make_pair 组成数据对(first, second)
            }
        }

//        mCurrentFrame.mvKeysRays.resize(mCurrentFrame.N_total);
//        mCurrentFrame.mvKeysRays = vector<cv::Vec3f>(mCurrentFrame.N_total,static_cast<cv::Vec3f>(0,0,0));//这句有必要吗
        // 步骤3.2：按照深度从小到大排序
        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            // 步骤3.3：将距离比较近的点包装成MapPoints
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second; //second 是数据对的第二个数据(特征点编号)

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);//关键帧的地图点
//                    float x3Dx, x3Dy, x3Dz;
//                    x3Dx=x3D.at<float>(0,0);
//                    x3Dy=x3D.at<float>(1,0);
//                    x3Dz=x3D.at<float>(2,0);
//                    std::vector<cv::Vec3d>> keyRaysTemp;
//                    mCurrentFrame.mvKeysRays[i]=cv::Vec3f(x3Dx, x3Dy, x3Dz);//特征点3d(世界)坐标?
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    // 这些添加属性的操作是每次创建MapPoint后都要做的
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }

        //cam2
        vector<pair<float,int> > vDepthIdx_cam2;//深度,编号(局部相机)
        vDepthIdx_cam2.reserve(mCurrentFrame.N_cam2);
        for(int i=0; i<mCurrentFrame.N_cam2; i++)
        {
            float z = mCurrentFrame.mvDepth_cam2[i];  //只用到mvDepth和mvDepth_cam2要怎么改??
            if(z>0)
            {
                vDepthIdx_cam2.push_back(make_pair(z,i)); //make_pair 组成数据对(first, second)深度,特征点编号
            }
        }

        // 步骤3.2：按照深度从小到大排序
        if(!vDepthIdx_cam2.empty())
        {
            sort(vDepthIdx_cam2.begin(),vDepthIdx_cam2.end());

            // 步骤3.3：将距离比较近的点包装成MapPoints
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx_cam2.size();j++)
            {
                int i = vDepthIdx_cam2[j].second; //second 是数据对的第二个数据(i)

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i+mCurrentFrame.N];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i+mCurrentFrame.N] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo_cam2(i);//世界坐标
//                    float x3Dx, x3Dy, x3Dz;
//                    x3Dx=x3D.at<float>(0,0);
//                    x3Dy=x3D.at<float>(1,0);
//                    x3Dz=x3D.at<float>(2,0);
//                    std::vector<cv::Vec3d>> keyRaysTemp;
//                    mCurrentFrame.mvKeysRays[i+(mCurrentFrame.N)]=cv::Vec3f(x3Dx, x3Dy, x3Dz);//特征点3d(世界)坐标?
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    // 这些添加属性的操作是每次创建MapPoint后都要做的 ?????
                    pNewMP->AddObservation(pKF,i+(mCurrentFrame.N));
                    pKF->AddMapPoint(pNewMP,i+(mCurrentFrame.N));
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i+mCurrentFrame.N]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx_cam2[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
//
//        for(int i=0; i<mCurrentFrame.N_cam2;i++)
//        {
//            int temp=(mCurrentFrame.N)+i;
//            mCurrentFrame.mvpMapPoints[temp]=mCurrentFrame.mvpMapPoints_cam2[i];
//        }

    }

    mpLocalMapper->InsertKeyFrame(pKF);

//    cout<<"Tracking.cc:: L1635 SetNotStop(false) ..."<<endl;
    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

// 将Local MapPoints重投影（isInFrustum()）到当前帧
// 并标记了这些点是否在当前帧的视野中，即mbTrackInView
// 在SearchByProjection()中对这些MapPoints，在其投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
// 用于TrackLocalMap()
void Tracking::SearchLocalPoints()
{
//    cout<<"Tracking.cc:: L1588 SearchLocalPoints() ..."<<endl;
    // Do not search map points already matched
    // 步骤1：遍历在当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    // 因为当前的mvpMapPoints一定在当前帧的视野中
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
//        cout<<"Tracking.cc:: L1698 SearchLocalPoints() ..."<<endl;
        MapPoint* pMP = *vit;
        if(pMP)
        {
//            cout<<"Tracking.cc:: L1702 SearchLocalPoints() ..."<<endl;
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点将来不被投影，因为已经匹配过
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    // 步骤2：将所有局部MapPoints投影到当前帧，判断是否在视野范围内，然后进行投影匹配
    //局部地图点在UpdateLocalMap()中生成
//    cout<<"Tracking.cc:: L1723 SearchLocalPoints() ..."<<endl;
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        // 步骤2.1：判断LocalMapPoints中的点是否在在视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
//        cout<<"Tracking.cc:: L1743 局部地图点共有 <"<<nToMatch<<"> 个点在视野内"<<endl;
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        // 步骤2.2：对视野范围内的MapPoints通过投影进行特征点匹配
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
    else if(nToMatch<=0)
    {
//        cout<<"Tracking.cc:: L1756 局部地图点共有 <0> 个点在视野内"<<endl;
    }
}
/**
 * @brief 更新LocalMap
 *
 * 局部地图包括：
 * - K1个关键帧、K2个临近关键帧和参考关键帧
 * - 由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
//    cout<<"Tracking.cc:: L1646 UpdateLocalMap() ..."<<endl;
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 更新局部关键帧和局部MapPoints
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}
/**
 * @brief 更新局部关键点，called by UpdateLocalMap()
 *
 * 局部关键帧mvpLocalKeyFrames的MapPoints更新mvpLocalMapPoints
 */
void Tracking::UpdateLocalPoints()
{
    // 步骤1：清空局部MapPoints
    mvpLocalMapPoints.clear();

    // 步骤2：遍历局部关键帧mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // 步骤2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // mnTrackReferenceForFrame防止重复添加局部MapPoint
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
    //plc
    const int nLocalMapPoint = mvpLocalMapPoints.size();
//    cout<<"Tracking.cc:: L1762 UpdateLocalMap() 更新后共有 <"<<nLocalMapPoint<<"> 个局部点"<<endl;
}


/**
 * @brief 更新局部关键帧，called by UpdateLocalMap()
 *
 * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新mvpLocalKeyFrames
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // 步骤1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧
    map<KeyFrame*,int> keyframeCounter;
//    for(int i=0; i<mCurrentFrame.mvpMapPoints.size();i++) //plc
    for(int i=0; i<mCurrentFrame.N_total; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                // 能观测到当前帧MapPoints的关键帧
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // 步骤2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // V-D K1: shares the map points with current frame
    // 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // V-D K2: neighbors to K1 in the covisibility graph
    // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent(); // 这个parent是多相机帧的父帧
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

// BOW搜索，PnP求解位姿
// 将当前帧转换成关键字的包，然后检索关键帧数据库用于重定位。
// 如果几个地方的图像对于当前帧相似，将会有几个候选帧用于重定位。
// 对于每个候选关键帧都要计算ORB特征匹配一致性，即当然帧和ORB特征点关联的地图点的关键帧。
// 这样我们就有了每个候选关键帧的一组2D到3D的一一对应关系。
// 现在对每个候选帧执行RANSAC迭代算法，计算出相机的位姿解决每次迭代的PnP问题。
// 如果我们获得了相机位姿，我们就可以继续执行追踪功能
/*
1. 计算当前帧的BoW映射；
2. 在关键帧数据库中找到相似的候选关键帧；
3. 通过BoW匹配当前帧和每一个候选关键帧，如果匹配数足够 >15，进行EPnP求解；
4. 对求解结果使用BA优化，如果内点较少，则反投影候选关键帧的地图点到当前帧 获取额外的匹配点
   根据特征所属格子和金字塔层级重新建立候选匹配，选取最优匹配；
   若这样依然不够，放弃该候选关键帧，若足够，则将通过反投影获取的额外地图点加入，再进行优化。
5. 如果内点满足要求(>50)则成功重定位，将最新重定位的id更新：mnLastRelocFrameId = mCurrentFrame.mnId; 否则返回false。
 */  //todo 暂只支持cam1的重定位
bool Tracking::Relocalization()
{
    cout<<"Tracking.cc:: L1822 Relocalozation()..."<<endl;
    // note 计算重定位时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // Compute Bag of Words Vector
    // 步骤1：计算当前帧特征点的Bow映射
    mCurrentFrame.ComputeBoW_cam1(); //计算词包mBowVec 和 mFeatVec

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // 步骤2：找到与当前帧相似的候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame); //得到相似的关键帧

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();//候选关键帧个数

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    //我们首先执行与每个候选匹配的ORB匹配
    //如果找到足够的匹配，我们设置一个PNP解算器
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches; //当前帧与各候选帧的匹配点
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 步骤3：通过BoW进行匹配 如果匹配数足够 >15，进行EPnP求解；
            int nmatches = matcher.SearchByBoW_cam1(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15) //匹配个数小于15的筛掉
            {
                vbDiscarded[i] = true;
                continue;
            }
            else//匹配足够, 加入求解器 用pnp求解
            {
                // 初始化PnPsolver
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991); // 随机采样
                vpPnPsolvers[i] = pSolver; // 添加求解器
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    // 执行迭代直到找到一个候选匹配关键帧和符合变换关系Rt的足够的内点数量
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i]) // 跳过匹配效果差的候选关键帧
                continue;

            // Perform 5 Ransac Iterations
            // 5次 随机采样序列 求解位姿  Tcw
            vector<bool> vbInliers; // 符合变换的内点个数
            int nInliers;
            bool bNoMore;

            // 步骤4：通过EPnP算法估计姿态
            // 求解器求解 进行EPnP求解
            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers); //迭代5次 得到变换矩阵

            // If Ransac reachs max. iterations discard keyframe
            // 迭代5次效果还不好
            if(bNoMore)
            {
                vbDiscarded[i]=true; // EPnP求解结果不好,匹配效果差,则放弃该候选关键帧
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            // 对求解结果使用BA优化，如果内点较少，则反投影当前帧的地图点到候选关键帧获取额外的匹配点；
            // 若这样依然不够，放弃该候选关键帧，若足够，则将通过反投影获取的额外地图点加入，再进行优化。
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw); //设置当前帧位姿

                set<MapPoint*> sFound; // 地图点

                const int np = vbInliers.size(); // 符合位姿Tcw的内点数量

                for(int j=0; j<np; j++) // 遍历每一个符合的内点
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];// 对应的地图点i帧 的地图点j
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                // 步骤5：通过PoseOptimization对姿态进行优化求解
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io]) // 优化后更新状态 外点
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);// 地图点为空指针

                // If few inliers, search by projection in a coarse window and optimize again
                // 步骤6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                // 如果内点较少，则反投影候选关键帧的地图点vpCandidateKFs[i]到当前帧像素坐标系下
                // 根据格子和金字塔层级信息 在当前帧下选择与地图点匹配的特征点 获取额外的匹配点
                if(nGood<50)
                {
                    //todo cam2
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        // 当前帧特征点对应的地图点数大于50 进行优化
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        //如果许多内点仍然不够，则在较窄的窗口中再次用投影搜索
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            // 缩小搜索窗口
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io]) //外点
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        cout<<"tracking.cc::L2129:重定位失败"<<endl;
        return false;
    }
    else
    {
//        cout<<"tracking.cc::L2129:重定位成功"<<endl;
        mnLastRelocFrameId = mCurrentFrame.mnId; // 重定位 帧ID
        // note 计算重定位时间
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double treloc= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout<<"重定位成功，花费时间为： "<<treloc<<endl;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
