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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor.
    // 复制构造函数参数为类对象本身的引用，用于根据一个已存在的对象复制出一个新的该类的对象，
    // 一般在函数中会将已存在对象的数据成员的值复制一份到新创建的对象中
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for multi RGB-D cameras.
    Frame(const cv::Mat &imGray1, const cv::Mat &imDepth1, const cv::Mat &imGray2, const cv::Mat &imDepth2,
          const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef,
          const float &bf, const float &thDepth, const cv::Mat &CalibMatrix);

    Frame(const cv::Mat &imGray1, const cv::Mat &imDepth1, const cv::Mat &imGray2, const cv::Mat &imDepth2,
          const double &timeStamp, ORBextractor* extractor,ORBextractor* extractor_cam2,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef,
          const float &bf, const float &thDepth, const cv::Mat &CalibMatrix);


    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);
    void ExtractORB_cam1(int flag, const cv::Mat &im);
    void ExtractORB_cam2(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();
    void ComputeBoW_cam1();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
    vector<size_t> GetFeaturesInArea(const int cam, const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);
    void ComputeStereoFromRGBD_cam1(const cv::Mat &imDepth1);
    void ComputeStereoFromRGBD_cam2(const cv::Mat &imDepth2);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);
    cv::Mat UnprojectStereo_cam2(const int &i); //plc
    cv::Mat UnprojectStereo_camid(int &cam, const int &i); //plc

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    //plc
//    ORBextractor* mpORBextractorLeft_cam1, *mpORBextractorRight_cam1;
    ORBextractor* mpORBextractorLeft_cam2, *mpORBextractorRight_cam2;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;
    int N_total;
    int N_cam1,N_cam2; //plc

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeys_total;
    std::vector<cv::KeyPoint> mvKeys_cam1, mvKeysRight_cam1,mvKeys_cam2, mvKeysRight_cam2;   // plc
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<cv::KeyPoint> mvKeysUn_total;
    std::vector<cv::KeyPoint> mvKeysUn_cam1, mvKeysUn_cam2; //plc

    //特征点坐标(所有相机的全部特征点)
//    std::vector<cv::Vec3f>  mvKeysRays; // 3D observation rays.//这个变量没用到?

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvuRight_cam2 ;
    std::vector<float> mvuRight_total; //note 这个是多相机
    std::vector<float> mvDepth;
    // plc
    std::vector<float> mvDepth_cam1, mvDepth_cam2 ;
    std::vector<float> mvDepth_total;

    // Bag of Words Vector structures.
    //BowVec就是描述一张图像的一系列视觉词汇，视觉词汇的id和它的权重值: <wordid, wordvalue>
    //FeatVec就是节点的id和每个节点拥有的特征索引:map<NodeId, vector<int>>  (node指词袋节点?)
    DBoW2::BowVector mBowVec; //已经是所有特征点了?
    DBoW2::BowVector mBowVec_cam1 , mBowVec_cam2; //plc
    DBoW2::FeatureVector mFeatVec ; //已经是所有特征点了?
    DBoW2::FeatureVector mFeatVec_cam1 , mFeatVec_cam2; //plc

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    std::vector<cv::Mat> mDescriptors_total; //多相机关键帧描述子(局部相机索引)

    //plc
//    cv::Mat mDescriptors_cam1, mDescriptorsRight_cam1;
    cv::Mat mDescriptors_cam2, mDescriptorsRight_cam2;//实际用的_total

    // MapPoints associated to keypoints, NULL pointer if no association.
    // 每个特征点对应的MapPoint
    std::vector<MapPoint*> mvpMapPoints;//若特征点i没有地图点则mvpMapPoints[i]=null?
    std::vector<MapPoint*> mvpMapPoints_cam1, mvpMapPoints_cam2;
    // ↑ 普通帧的mvpMapPoints是它与参考关键帧的匹配点 + 一些比较近的地图点?

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;//包含全部点 note 位姿优化时误差较大的点设为外点
    std::vector<bool> mvbOutlier_cam2;    //plc 没用到?

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    // mGrid将提取的特征分配到网格（在网格下看关键点是否正确），可以加速匹配过程
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    std::vector<std::size_t> mGrid_cam2[FRAME_GRID_COLS][FRAME_GRID_ROWS];//plc 保存该帧特征点的全局索引
//    std::vector<std::vector<std::size_t> > mGrids[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    //多相机的grids
    std::vector<std::vector< std::vector <std::vector<size_t> > > >mGrids;//每个格子里是特征点全局索引

    // Camera pose.
    cv::Mat mTcw;

    // Rcam12 = Rc1w * Rwc2 ,
    // tcam12 = Rc1w * (twc2 - twc1)
    //两相机标定的变换矩阵
    cv::Mat mRcam12= cv::Mat_<float>(3,3);
    cv::Mat mtcam12 = cv::Mat_<float>(3,1);

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels; // 尺度总层数,yaml中是8
    float mfScaleFactor; //对应金字塔图像的尺度因子
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;//0
    static float mnMaxX;//图像列数640
    static float mnMinY;//0
    static float mnMaxY;//图像行数480

    static bool mbInitialComputations;

    // this variable holds the mapping between keypoint ID and camera
    // it was observed in
    // [key_id : cam_id] //<该特征点在该帧的总编号，所在的相机编号>
    std::unordered_map<size_t, int> keypoint_to_cam;
    // this variable holds the mapping between the continous indexing of all
    // descriptors and keypoints and the image wise indexes
    // it was observed in
    // [cont_id : local_image_id] //<点在多相机帧系统的总编号，在各相机分别的编号>
    std::unordered_map<size_t, int> cont_idx_to_local_cam_idx;

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    void UndistortKeyPoints_cam1();
    void UndistortKeyPoints_cam2();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
