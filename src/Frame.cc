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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
// 复制构造函数, mLastFrame = Frame(mCurrentFrame)
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     //mpORBextractorLeft_cam1(frame.mpORBextractorLeft_cam1), mpORBextractorRight_cam1(frame.mpORBextractorRight_cam1), //plc
     mpORBextractorLeft_cam2(frame.mpORBextractorLeft_cam2), mpORBextractorRight_cam2(frame.mpORBextractorRight_cam2),  //plc
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N),//N_cam1(frame.N_cam1),
     N_cam2(frame.N_cam2),
     N_total(frame.N_total),
     mRcam12(frame.mRcam12),mtcam12(frame.mtcam12),//标定的变换矩阵
     mvKeys(frame.mvKeys), //mvKeys_cam1(frame.mvKeys_cam1),
     mvKeys_cam2(frame.mvKeys_cam2),
     mvKeys_total(frame.mvKeys_total), //
     mvKeysRight(frame.mvKeysRight), //mvKeysRight_cam1(frame.mvKeysRight_cam1),
     mvKeysRight_cam2(frame.mvKeysRight_cam2), mvKeysUn(frame.mvKeysUn),  mvKeysUn_cam1(frame.mvKeysUn_cam1),
     mvKeysUn_cam2(frame.mvKeysUn_cam2),
     mvKeysUn_total(frame.mvKeysUn_total),
//     mvKeysRays(frame.mvKeysRays),//plc
     mvuRight(frame.mvuRight),mvuRight_cam2(frame.mvuRight_cam2),
     mvuRight_total(frame.mvuRight_total),//plc
     mvDepth(frame.mvDepth),mvDepth_cam1(frame.mvDepth_cam1),mvDepth_cam2(frame.mvDepth_cam2),//
     mvDepth_total(frame.mvDepth_total),
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mBowVec_cam1(frame.mBowVec_cam1), mFeatVec_cam1(frame.mFeatVec_cam1),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mDescriptors_cam2(frame.mDescriptors_cam2.clone()), mDescriptorsRight_cam2(frame.mDescriptorsRight_cam2.clone()), //plc
     mDescriptors_total(frame.mDescriptors_total),//plc
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mvbOutlier_cam2(frame.mvbOutlier_cam2 ),//plc
     mnId(frame.mnId),
     keypoint_to_cam(frame.keypoint_to_cam),cont_idx_to_local_cam_idx(frame.cont_idx_to_local_cam_idx),//plc
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
//    mGrids.resize(2);
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];
//    for (int c = 0; c < 2; ++c)//TODO 多相机合并成一个mGrids
//    {
//        mGrids[c] = frame.mGrids[c];
//    }

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


////双目相机初始化
//Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
//    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
//     mpReferenceKF(static_cast<KeyFrame*>(NULL))
//{
//    // Frame ID
//    mnId=nNextId++;
//
//    // Scale Level Info
//    mnScaleLevels = mpORBextractorLeft->GetLevels();
//    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
//    mfLogScaleFactor = log(mfScaleFactor);
//    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
//    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
//    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
//    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
//
//    // ORB extraction
//    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
//    thread threadRight(&Frame::ExtractORB,this,1,imRight);
//    threadLeft.join();
//    threadRight.join();
//
//    N = mvKeys.size();
//
//    if(mvKeys.empty())
//        return;
//
//    UndistortKeyPoints();
//
//    ComputeStereoMatches();
//
//    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
//    mvbOutlier = vector<bool>(N,false);
//    mvbOutlier_cam2 = vector<bool>(N_cam2,false); //plc
//
//
//    // This is done only for the first Frame (or after a change in the calibration)
//    if(mbInitialComputations)
//    {
//        ComputeImageBounds(imLeft);
//
//        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
//        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
//
//        fx = K.at<float>(0,0);
//        fy = K.at<float>(1,1);
//        cx = K.at<float>(0,2);
//        cy = K.at<float>(1,2);
//        invfx = 1.0f/fx;
//        invfy = 1.0f/fy;
//
//        mbInitialComputations=false;
//    }
//
//    mb = mbf/fx;
//
//    AssignFeaturesToGrid();
//}
//multi-RGBD相机初始化
Frame::Frame(const cv::Mat &imGray1, const cv::Mat &imDepth1, const cv::Mat &imGray2, const cv::Mat &imDepth2,
             const double &timeStamp, ORBextractor* extractor,ORBextractor* extractor_cam2,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef,
             const float &bf, const float &thDepth, const cv::Mat &CalibMatrix)
    :mpORBvocabulary(voc),
     mpORBextractorLeft(extractor),
     mpORBextractorLeft_cam2(extractor_cam2), //plc
     mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mpORBextractorRight_cam2(static_cast<ORBextractor*>(NULL)), //plc
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();//TUM1.yaml里是8
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    mDescriptors_total.resize(2);//总描述子

    //+++++++++++++++++++++++++++++
    mRcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);
    mtcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    mtcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    mtcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    //++++++++++++++++++++++++++++++

//    cout<<"Extract ORB..."<<endl;

    // ORB extraction   //关键点和描述子储存在全局变量mvKeys 和 mDescriptors中
    ExtractORB(0,imGray1);

//    ExtractORB_cam1(0,imGray1); //储存在mvKeys_cam1, mDescriptors_cam1中
    ExtractORB_cam2(0,imGray2); //提取特征点,储存在mvKeys_cam2

//    cout<<"Frame.cc::L177..."<<endl;

        //描述子 plc
//    mDescriptors_total[0]= mDescriptors.clone();
    mDescriptors_total[0]= mDescriptors;
//    cout<<"Frame.cc::L181..."<<endl;
//    mDescriptors_total[1]= mDescriptors_cam2.clone();
    mDescriptors_total[1]= mDescriptors_cam2;
//    for(int i=0;i<N_cam2;i++)
//    {
//        mDescriptors_total.push_back(mDescriptors_cam2.row(i));
//    }
//    cout<<"Frame.cc::L186..."<<endl;
    N = mvKeys.size();
//    N_cam1 = mvKeys_cam1.size();
    N_cam2 = mvKeys_cam2.size();

    N_total = N + N_cam2;

//    mvKeys_total.reserve(N_total);
    mvKeys_total.resize(N_total);

//    cout<<"N="<<N<<endl;
//    cout<<"N_cam2="<<N_cam2<<endl;
    cout<<"N_total="<<N_total<<endl;

//    mvKeysRays.reserve(N_total);//mvKeysRays表示第i个特征点点的坐标(存在一组数组里)
//    mvKeysRays.resize(N_total);//mvKeysRays表示第i个特征点点的坐标(存在一组数组里)
//    mvKeysRays.reserve(N_total);//mvKeysRays表示第i个特征点点的坐标(存在一组数组里)
//    mvKeysRays.reserve(N_cam2);//mvKeysRays表示相机c的第i个点的坐标(存在一组数组里)

    //保存特征点的局部编号
    int currPtIdx = 0;//←总编号

    for (int i = 0; i < N; ++i)//i是局部(某相机下)编号
    {
        keypoint_to_cam[currPtIdx] = 0;//<size_t, int>,该帧总编号为currPtIdx的点，该点所在相机编号为0
        cont_idx_to_local_cam_idx[currPtIdx] = i;//<size_t, int>为<点在多相机帧系统的总编号，各相机下局部编号>
//        cv::KeyPoint &kp = keyPtsTemp[c][i];
//         //fill grids
//        int nGridPosX, nGridPosY;
//        if (PosInGrid(c, kp, nGridPosX, nGridPosY))
//            mGrids[c][nGridPosX][nGridPosY].push_back(currPtIdx);
        mvKeys_total[i]=mvKeys[i];
        ++currPtIdx;
    }
    for (int i = 0; i < N_cam2; ++i)
    {
        keypoint_to_cam[currPtIdx] = 1;//<size_t, int>,该帧总编号为currPtIdx的点，该点所在的相机编号为1
        cont_idx_to_local_cam_idx[currPtIdx] = i;//<size_t, int>为<点在多相机帧系统的总编号，在各相机分别的编号>
        mvKeys_total[currPtIdx]=mvKeys_cam2[i];
        ++currPtIdx;
    }

    if(mvKeys.empty() && mvKeys_cam2.empty())
        return;
//    cout<<"Frame.cc::L230..."<<endl;
    // Undistort特征点 //矫正后关键点储存在mvKeysUn
    UndistortKeyPoints();
//    cout<<"Frame.cc L174"<<endl;
    if(!mvKeys_cam2.empty())
        UndistortKeyPoints_cam2(); //存在mvKeysUn_cam2中
    // 得到立体信息,关键点i的深度储存在mvDepth[i]上
    ComputeStereoFromRGBD(imDepth1);
//    ComputeStereoFromRGBD_cam1(imDepth1);
    // 得到关键点i的深度信息存在 mvuRight_cam2[i] , mvDepth_cam2[i]
    ComputeStereoFromRGBD_cam2(imDepth2);
//    cout<<"Frame.cc::L240..."<<endl;
    // 对应的mappoints
    mvpMapPoints = vector<MapPoint*>(N_total,static_cast<MapPoint*>(NULL));
    // plc
//    mvpMapPoints_cam1 = vector<MapPoint*>(N_cam1,static_cast<MapPoint*>(NULL));
//    mvpMapPoints_cam2 = vector<MapPoint*>(N_cam2,static_cast<MapPoint*>(NULL));
//    mvbOutlier = vector<bool>(N,false); //N ?
    mvbOutlier = vector<bool>(N_total,false); //N ?
    // 观测不到的点;plc
    mvbOutlier_cam2 = vector<bool>(N_cam2,false);//没用?
//        cout<<"Frame.cc L191"<<endl;
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray1); //因为图像失真，考虑了畸变，转换后图像的边界信息就发生了变化，对帧先计算出边界信息
        ComputeImageBounds(imGray2); //图2 ??

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    //将提取的特征分配到网格（在网格下看关键点是否正确），可以加速匹配过程
    AssignFeaturesToGrid();//cam1/2
}

////单目相机
//Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
//    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
//     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
//{
//    // Frame ID
//    mnId=nNextId++;
//
//    // Scale Level Info
//    mnScaleLevels = mpORBextractorLeft->GetLevels();
//    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
//    mfLogScaleFactor = log(mfScaleFactor);
//    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
//    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
//    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
//    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
//
//    // ORB extraction
//    ExtractORB(0,imGray);
//
//    N = mvKeys.size();
//
//    if(mvKeys.empty())
//        return;
//
//    UndistortKeyPoints();
//
//    // Set no stereo information
//    mvuRight = vector<float>(N,-1);
//    mvDepth = vector<float>(N,-1);
//
//    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
//    mvbOutlier = vector<bool>(N,false);
//
//    // This is done only for the first Frame (or after a change in the calibration)
//    if(mbInitialComputations)
//    {
//        ComputeImageBounds(imGray);
//
//        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
//        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
//
//        fx = K.at<float>(0,0);
//        fy = K.at<float>(1,1);
//        cx = K.at<float>(0,2);
//        cy = K.at<float>(1,2);
//        invfx = 1.0f/fx;
//        invfy = 1.0f/fy;
//
//        mbInitialComputations=false;
//    }
//
//    mb = mbf/fx;
//
//    AssignFeaturesToGrid();
//}

//将提取的特征分配到网格（在网格下看关键点是否正确），可以加速匹配过程
void Frame::AssignFeaturesToGrid() //每10x10像素为一格,共48x64格
{
    // 将特征分配到格子中
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    int nReserve_cam2 = 0.5f*N_cam2/(FRAME_GRID_COLS*FRAME_GRID_ROWS);

    mGrids.resize(2);

    for (int c=0;c<2;++c){
        mGrids[c]= vector<vector<vector<size_t> > >(FRAME_GRID_COLS);
        for (unsigned int j = 0; j < FRAME_GRID_COLS; ++j)
            mGrids[c][j] = vector<vector<size_t> >(FRAME_GRID_ROWS);
    }

    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
        {
            mGrid[i][j].reserve(nReserve);
            mGrids[0][i][j].reserve(nReserve);
            mGrid_cam2[i][j].reserve(nReserve_cam2);//TODO 各个相机合并成mGrids
            mGrids[1][i][j].reserve(nReserve_cam2);
        }


    // 在mGrid中记录了各特征点
    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
        {
            mGrid[nGridPosX][nGridPosY].push_back(i);
            mGrids[0][nGridPosX][nGridPosY].push_back(i);
        }
    }
    for(int i=0;i<N_cam2;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn_cam2[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
        {
            mGrid_cam2[nGridPosX][nGridPosY].push_back(i+N);//每个格子里相机2的特征点全局索引
            mGrids[1][nGridPosX][nGridPosY].push_back(i+N);//每个格子里相机2的特征点全局索引
        }
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)  //()是operator重载运算符,括号内分别是(输入图像,mask,关键点,描述子)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors); //mvKeys,mDescriptors两个参数储存关键点和描述子
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
// //plc
//void Frame::ExtractORB_cam1(int flag, const cv::Mat &im)
//{
//    if(flag==0)
//        (*mpORBextractorLeft_cam1)(im,cv::Mat(),mvKeys_cam1,mDescriptors_cam1); //mvKeys,mDescriptors两个参数储存关键点和描述子
//    else
//        (*mpORBextractorRight_cam1)(im,cv::Mat(),mvKeysRight_cam1,mDescriptorsRight_cam1);
//}

void Frame::ExtractORB_cam2(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft_cam2)(im,cv::Mat(),mvKeys_cam2,mDescriptors_cam2); //mvKeys,mDescriptors两个参数储存关键点和描述子
    else
        (*mpORBextractorRight_cam2)(im,cv::Mat(),mvKeysRight_cam2,mDescriptorsRight_cam2);
}

void Frame::SetPose(cv::Mat Tcw)//得到Tcw, Rcw, Rwc, tcw, twc
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();//由Tcw得到Rcw, Rwc, tcw, twc
}

void Frame::UpdatePoseMatrices()//由Tcw得到Rcw, Rwc, tcw, twc
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}
/**
 * @brief 判断一个点是否在视野内
 *
 * 计算了重投影坐标，观测方向夹角，预测在当前帧的尺度
 * @param  pMP             MapPoint
 * @param  viewingCosLimit 视角和平均视角的方向阈值
 * @return                 true if is in view
 * @see SearchLocalPoints()
 */
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

/**
 * @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点
 * @param x        图像坐标u
 * @param y        图像坐标v
 * @param r        边长
 * @param minLevel 最小尺度
 * @param maxLevel 最大尺度
 * @return         满足条件的特征点的序号
 */
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N_total);//plc

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

/**
 * @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点
 * @param x        图像坐标u
 * @param y        图像坐标v
 * @param r        边长
 * @param minLevel 最小尺度
 * @param maxLevel 最大尺度
 * @return         满足条件的特征点的序号
 */ //NOTE 增加相机编号
vector<size_t> Frame::GetFeaturesInArea(const int cam,
                                        const float &x, const float  &y, const float  &r,
                                        const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N_total);//plc

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrids[cam][ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn_total[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

/**
 * @brief Bag of Words Representation
 *
 * 计算词包mBowVec和mFeatVec，其中mFeatVec记录了属于第i个node（在第4层）的ni个描述子
 */
void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors_total); //一帧的描述子从矩阵形式转换成一条条单行的向量
        //当前描述子转换为词袋向量和节点与索引的特征向量，4代表字典树中的从叶子节点向上数的第4层
        //BowVec就是描述一张图像的一系列视觉词汇，视觉词汇的id和它的权重值: <wordid, wordvalue>
        //FeatVec就是节点的id和每个节点拥有的特征索引:map<NodeId, vector<int>>
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4); //mpORBvocabulary:Vocabulary used for relocalization.
    }
}

void Frame::ComputeBoW_cam1()
{
    if(mBowVec_cam1.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors); //一帧的描述子从矩阵形式转换成一条条单行的向量
        //当前描述子转换为词袋向量和节点与索引的特征向量，4代表字典树中的从叶子节点向上数的第4层
        //BowVec就是描述一张图像的一系列视觉词汇，视觉词汇的id和它的权重值
        //FeatVec就是节点的id和每个节点拥有的特征索引:map<NodeId, vector<int>>
        mpORBvocabulary->transform(vCurrentDesc,mBowVec_cam1,mFeatVec_cam1,4); //mpORBvocabulary:Vocabulary used for relocalization.
    }
}

void Frame::UndistortKeyPoints()  //矫正畸变后关键点储存在mvKeysUn
{
//    cout<<"frame.cc:: L589 UndistortKeyPoints() 矫正关键点..."<<endl;
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2); //reshape 改变行列数,但是行数*列数不变
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    mvKeysUn_total.resize(N_total);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
        mvKeysUn_total[i]=kp;//总关键点
    }
}

void Frame::UndistortKeyPoints_cam2()  //矫正后关键点储存在mvKeysUn_cam2
{
//    cout<<"frame.cc:: L624 UndistortKeyPoints_cam2() 矫正关键点cam2..."<<endl;
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn_cam2=mvKeys_cam2;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N_cam2,2,CV_32F);
    for(int i=0; i<N_cam2; i++)
    {
        mat.at<float>(i,0)=mvKeys_cam2[i].pt.x;
        mat.at<float>(i,1)=mvKeys_cam2[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn_cam2.resize(N_cam2);
    for(int i=0; i<N_cam2; i++)
    {
        cv::KeyPoint kp = mvKeys_cam2[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn_cam2[i]=kp;
        mvKeysUn_total[N+i]=kp;//总关键点
    }
}

//因为图像失真，考虑了畸变，转换后图像的边界信息就发生了变化，对帧先计算出边界信息
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    // 如果是畸变图像
    if(mDistCoef.at<float>(0)!=0.0)
    {
        // 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0;
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;
        mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols;
        mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        // 通过mat类型，转成4个点对即图像的4个边角点，进行畸变计算
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // 对矫正之后的点选出最大最小边界值，也就是左上与左下进行比较获取x的最小值
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

//匹配左右图的特征点(双目初始化)
//void Frame::ComputeStereoMatches()
//{
//    mvuRight = vector<float>(N,-1.0f);
//    mvDepth = vector<float>(N,-1.0f);
//
//    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
//
//    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;
//
//    //Assign keypoints to row table
//    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
//
//    for(int i=0; i<nRows; i++)
//        vRowIndices[i].reserve(200);
//
//    const int Nr = mvKeysRight.size();
//
//    for(int iR=0; iR<Nr; iR++)
//    {
//        const cv::KeyPoint &kp = mvKeysRight[iR];
//        const float &kpY = kp.pt.y;
//        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
//        const int maxr = ceil(kpY+r);
//        const int minr = floor(kpY-r);
//
//        for(int yi=minr;yi<=maxr;yi++)
//            vRowIndices[yi].push_back(iR);
//    }
//
//    // Set limits for search
//    const float minZ = mb;
//    const float minD = 0;
//    const float maxD = mbf/minZ;
//
//    // For each left keypoint search a match in the right image
//    vector<pair<int, int> > vDistIdx;
//    vDistIdx.reserve(N);
//
//    for(int iL=0; iL<N; iL++)
//    {
//        const cv::KeyPoint &kpL = mvKeys[iL];
//        const int &levelL = kpL.octave;
//        const float &vL = kpL.pt.y;
//        const float &uL = kpL.pt.x;
//
//        const vector<size_t> &vCandidates = vRowIndices[vL];
//
//        if(vCandidates.empty())
//            continue;
//
//        const float minU = uL-maxD;
//        const float maxU = uL-minD;
//
//        if(maxU<0)
//            continue;
//
//        int bestDist = ORBmatcher::TH_HIGH;
//        size_t bestIdxR = 0;
//
//        const cv::Mat &dL = mDescriptors.row(iL);//双目?
//
//        // Compare descriptor to right keypoints
//        for(size_t iC=0; iC<vCandidates.size(); iC++)
//        {
//            const size_t iR = vCandidates[iC];
//            const cv::KeyPoint &kpR = mvKeysRight[iR];
//
//            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
//                continue;
//
//            const float &uR = kpR.pt.x;
//
//            if(uR>=minU && uR<=maxU)
//            {
//                const cv::Mat &dR = mDescriptorsRight.row(iR);
//                const int dist = ORBmatcher::DescriptorDistance(dL,dR);
//
//                if(dist<bestDist)
//                {
//                    bestDist = dist;
//                    bestIdxR = iR;
//                }
//            }
//        }
//
//        // Subpixel match by correlation
//        if(bestDist<thOrbDist)
//        {
//            // coordinates in image pyramid at keypoint scale
//            const float uR0 = mvKeysRight[bestIdxR].pt.x;
//            const float scaleFactor = mvInvScaleFactors[kpL.octave];
//            const float scaleduL = round(kpL.pt.x*scaleFactor);
//            const float scaledvL = round(kpL.pt.y*scaleFactor);
//            const float scaleduR0 = round(uR0*scaleFactor);
//
//            // sliding window search
//            const int w = 5;
//            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
//            IL.convertTo(IL,CV_32F);
//            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
//
//            int bestDist = INT_MAX;
//            int bestincR = 0;
//            const int L = 5;
//            vector<float> vDists;
//            vDists.resize(2*L+1);
//
//            const float iniu = scaleduR0+L-w;
//            const float endu = scaleduR0+L+w+1;
//            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
//                continue;
//
//            for(int incR=-L; incR<=+L; incR++)
//            {
//                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
//                IR.convertTo(IR,CV_32F);
//                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);
//
//                float dist = cv::norm(IL,IR,cv::NORM_L1);
//                if(dist<bestDist)
//                {
//                    bestDist =  dist;
//                    bestincR = incR;
//                }
//
//                vDists[L+incR] = dist;
//            }
//
//            if(bestincR==-L || bestincR==L)
//                continue;
//
//            // Sub-pixel match (Parabola fitting)
//            const float dist1 = vDists[L+bestincR-1];
//            const float dist2 = vDists[L+bestincR];
//            const float dist3 = vDists[L+bestincR+1];
//
//            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
//
//            if(deltaR<-1 || deltaR>1)
//                continue;
//
//            // Re-scaled coordinate
//            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
//
//            float disparity = (uL-bestuR);
//
//            if(disparity>=minD && disparity<maxD)
//            {
//                if(disparity<=0)
//                {
//                    disparity=0.01;
//                    bestuR = uL-0.01;
//                }
//                mvDepth[iL]=mbf/disparity;
//                mvuRight[iL] = bestuR;
//                vDistIdx.push_back(pair<int,int>(bestDist,iL));
//            }
//        }
//    }
//
//    sort(vDistIdx.begin(),vDistIdx.end());
//    const float median = vDistIdx[vDistIdx.size()/2].first;
//    const float thDist = 1.5f*1.4f*median;
//
//    for(int i=vDistIdx.size()-1;i>=0;i--)
//    {
//        if(vDistIdx[i].first<thDist)
//            break;
//        else
//        {
//            mvuRight[vDistIdx[i].second]=-1;
//            mvDepth[vDistIdx[i].second]=-1;
//        }
//    }
//}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) //计算立体??
{
//    cout<<"frame.cc:: L875 ComputeStereoFromRGBD..."<<endl;
    mvuRight = vector<float>(N,-1);
    mvuRight_total = vector<float>(N_total,-1);
    mvDepth = vector<float>(N,-1);
    mvDepth_total = vector<float>(N_total,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i]; //关键点i
        const cv::KeyPoint &kpU = mvKeysUn[i]; //已校正关键点i

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u); //关键点i的深度

        if(d>0)
        {
            mvDepth[i] = d;  //关键点i的深度
            mvDepth_total[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;  //设置了虚拟右影像上的 u 坐标值 . 因为 ur=ul-bf/d
            mvuRight_total[i] = mvuRight[i];  //设置了虚拟右影像上的 u 坐标值 . 因为 ur=ul-bf/d
        }
    }
}

//void Frame::ComputeStereoFromRGBD_cam1(const cv::Mat &imDepth2)
//{
//    mvuRight_cam1 = vector<float>(N,-1);
//    mvDepth_cam1 = vector<float>(N,-1);
//
//    for(int i=0; i<N; i++)
//    {
//        const cv::KeyPoint &kp = mvKeys_cam1[i]; //关键点i
//        const cv::KeyPoint &kpU = mvKeysUn_cam1[i]; //已校正关键点i
//
//        const float &v = kp.pt.y;
//        const float &u = kp.pt.x;
//
//        const float d = imDepth2.at<float>(v,u); //关键点i的深度
//
//        if(d>0)
//        {
//            mvDepth_cam1[i] = d;  //关键点i的深度
//            mvuRight_cam1[i] = kpU.pt.x-mbf/d;  //设置了虚拟右影像上的 u 坐标值 . 因为 ur=ul-bf/d
//        }
//    }
//}
//
void Frame::ComputeStereoFromRGBD_cam2(const cv::Mat &imDepth2)
{
//    cout<<"frame.cc:: L926 ComputeStereoFromRGBD_cam2..."<<endl;
    mvuRight_cam2 = vector<float>(N_cam2,-1);
    mvDepth_cam2 = vector<float>(N_cam2,-1);

    for(int i=0; i<N_cam2; i++)
    {
        const cv::KeyPoint &kp = mvKeys_cam2[i]; //关键点i
        const cv::KeyPoint &kpU = mvKeysUn_cam2[i]; //已校正关键点i

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth2.at<float>(v,u); //关键点i的深度

        if(d>0)
        {
            mvDepth_cam2[i] = d;  //关键点i的深度
            mvDepth_total[i+N] = d;
            mvuRight_cam2[i] = kpU.pt.x-mbf/d;  //设置了虚拟右影像上的 u 坐标值 . 因为 ur=ul-bf/d
            mvuRight_total[i+N] = mvuRight_cam2[i];
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;//像素坐标
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;//相机坐标系上的坐标
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
//        cv::Mat x3Dw = mRwc*x3Dc+mOw;
//        return x3Dw;
        return mRwc*x3Dc+mOw; //moW==mtwc
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereo_cam2(const int &i)
{
    const float z = mvDepth_cam2[i];
    if(z>0)
    {
        const float u = mvKeysUn_cam2[i].pt.x;
        const float v = mvKeysUn_cam2[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc_cam2 = (cv::Mat_<float>(3,1) << x, y, z);
//        cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);

        return mRwc*(mRcam12*x3Dc_cam2+mtcam12)+mOw;//世界坐标
//        return mRwc*x3Dc_cam2+mOw;
    }
    else
        return cv::Mat();
}

//特征点所在的相机编号,特征点的总编号
cv::Mat Frame::UnprojectStereo_camid(int &cam, const int &i)
{
    const float z = mvDepth_total[i];
    if(z>0)
    {
        const float u = mvKeysUn_total[i].pt.x;//像素坐标
        const float v = mvKeysUn_total[i].pt.y;
        const float x = (u-cx)*z*invfx;//相机坐标
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
//        cv::Mat x3Dw = mRwc*x3Dc+mOw;
//        return x3Dw;
        if(cam<1)
        {
            return mRwc*x3Dc+mOw; //moW==mtwc
        }
        else if(cam>=1)
        {
            return mRwc*(mRcam12*x3Dc+mtcam12)+mOw; //moW==mtwc
        }
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
