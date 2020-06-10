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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPose_cam2();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetCameraCenter_cam2();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetRotation_cam2();
    cv::Mat GetTranslation_cam2();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void AddConnection_cam1(KeyFrame* pKF, const int &weight); //only cam1
    void AddConnection_cam2(KeyFrame* pKF, const int &weight); //cam2
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    void UpdateBestCovisibles_cam1(); //only cam1
    void UpdateBestCovisibles_cam2(); //cam2
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::set<KeyFrame *> GetConnectedKeyFrames_cam1();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames_cam1();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames_cam2();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames_cam1(const int &N);
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames_cam2(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    std::vector<KeyFrame*> GetCovisiblesByWeight_cam1(const int &w);
    int GetWeight(KeyFrame* pKF);
    int GetWeight_cam1(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    std::vector<MapPoint*> GetMapPointMatches_cam1();//仅返回相机1的点
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    //返回特征点坐标
    std::vector<cv::Vec3f> GetKeyPointsRays() const;
    cv::Vec3f GetKeyPointRay(const size_t &idx) const; // 3D ray
//    cv::Mat GetDescriptor(int& cam, size_t &idx) const;//plc 获得相机c下idx的描述子
    cv::Mat GetDescriptor(const int& cam, const size_t &idx) const;//plc 获得相机c下idx的描述子
//    cv::Mat GetDescriptor_cam2(const int& cam, const size_t &idx) const//plc

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    //多相机  (这里的&表示形参是引用)
    std::vector<size_t> GetFeaturesInArea(const int & cam,
                                          const float &x, const float  &y,
                                          const float  &r) const;
    cv::Mat UnprojectStereo(int i);
    cv::Mat UnprojectStereo_cam2(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId; // 作为关键帧的id,连续
    const long unsigned int mnFrameId; // 作为普通帧的帧id,在关键帧中不连续

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords; // 记录pKFi与pKF(当前帧)具有相同word的个数
    float mLoopScore;
    long unsigned int mnRelocQuery;//pKFi是否已经加入帧F的重定位候选帧,防止重复加入
    long unsigned int mnRelocQuery_cam2;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N,N_cam2,N_total;

    cv::Mat mRcam12;
    cv::Mat mtcam12;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeys_total;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<cv::KeyPoint> mvKeys_cam2;
    const std::vector<cv::KeyPoint> mvKeysUn_cam2;
    const std::vector<cv::KeyPoint> mvKeysUn_total;

    //表示第i个特征点的坐标(按相机顺序一次存在一组数组里)
    std::vector<cv::Vec3f> mvKeysRays;

    //mvuRight是虚拟右影像上的 u 坐标值（即把rgbd相机模拟成双目相机后的虚拟右相机中的点像素坐标）
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvuRight_cam2;
    const std::vector<float> mvuRight_total;
    const std::vector<float> mvDepth; // negative value for monocular points
    const std::vector<float> mvDepth_cam2;
    const std::vector<float> mvDepth_total;
    const cv::Mat mDescriptors;
    const cv::Mat mDescriptors_cam2; //plc
    const vector<cv::Mat> mDescriptors_total; //[0],[1],描述子用的局部相机下索引

    // BowVec就是描述一张图像的一系列视觉词汇，视觉词汇的id和它的权重值: <wordid, wordvalue>
    // FeatVec就是节点的id和每个节点拥有的特征索引:map<NodeId, vector<int>>  (node指词袋节点?)
    DBoW2::BowVector mBowVec ,mBowVec_cam1, mBowVec_cam2;//mBowVec是所有的描述子转换来的
    DBoW2::FeatureVector mFeatVec , mFeatVec_cam1 , mFeatVec_cam2;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;  //todo 两相机的该变量一样吗?? 尺度层数,都是8 好像
    const float mfScaleFactor; // 1.2 ?
    const float mfLogScaleFactor; // log1.2 ?
    const std::vector<float> mvScaleFactors; // 尺度因子，等于scale^n，scale=1.2，n为层数
    const std::vector<float> mvLevelSigma2; // mvLevelSigma2[i]金字塔第i层尺度因子的平方
    const std::vector<float> mvInvLevelSigma2; //mvInvLevelSigma2[i]金字塔第i层尺度因子的平方的逆

    // Image bounds and calibration
    // 以下各量也可能略有畸变
    const int mnMinX;//0
    const int mnMinY;//0
    const int mnMaxX;//图像列数640
    const int mnMaxY;//图像行数480
    const cv::Mat mK; //外部传进来的相机矩阵?

    // this hashmap holds the mapping between keypoint ID and camera
    // it was observed in
    // [key_id : cam_id]..<该特征点在该帧的总编号，所在相机的编号>
    std::unordered_map<size_t, int> keypoint_to_cam;//<该特征点在该帧的总编号，所在相机的编号>
    // this hashmap holds the mapping between the continous indexing of all
    // descriptors and keypoints and the image wise indexes
    // it was observed in
    // [cont_id : local_image_id]
    std::unordered_map<size_t, int> cont_idx_to_local_cam_idx;//<点在多相机帧系统的总编号，在各相机分别的编号>

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc; //相机坐标系到世界坐标系的变换, 是相机在世界坐标系下的旋转+平移
    cv::Mat Ow;//应该就是twc //Ow_cam2=
    cv::Mat Tcw_cam2;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
        //对应关键点的地图点?
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary ; //plc ? mpORBvocabulary_cam2

    // Grid over the image to speed up feature matching
    // 将提取的特征分配到mGrid网格（在网格下看关键点是否正确），可以加速匹配过程
    std::vector< std::vector <std::vector<size_t> > > mGrid; //相机1?
    std::vector<std::vector< std::vector <std::vector<size_t> > > >mGrids;//多相机的grids,每格储存数个特征点的全局索引

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights_cam1;
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights_cam2;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames_cam1;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames_cam2;
    std::vector<int> mvOrderedWeights;
    std::vector<int> mvOrderedWeights_cam1;
    std::vector<int> mvOrderedWeights_cam2;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    KeyFrame* mpParent_cam2;
    std::set<KeyFrame*> mspChildrens;
    // 当前帧对应的未优化的闭环关键帧？
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexConnections1;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
