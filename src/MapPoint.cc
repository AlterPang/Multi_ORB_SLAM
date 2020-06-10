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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

/**
 * @brief 给定坐标与keyframe构造MapPoint
 *
 * 双目：StereoInitialization()，CreateNewKeyFrame()，LocalMapping::CreateNewMapPoints()
 * 单目：CreateInitialMapMonocular()，LocalMapping::CreateNewMapPoints()
 * @param Pos    MapPoint的坐标（wrt世界坐标系）
 * @param pRefKF KeyFrame
 * @param pMap   Map
 */
MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;//mnId: Global ID for MapPoint
}

//MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap , int cam_2):
//        mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
//        mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
//        mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
//        mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
//{
//    Pos.copyTo(mWorldPos);
//}

/**
 * @brief 给定坐标与frame构造MapPoint
 *
 * 双目：UpdateLastFrame()
 * @param Pos    MapPoint的坐标（wrt世界坐标系）
 * @param pMap   Map
 * @param pFrame Frame
 * @param idxF   MapPoint在Frame中的索引，即对应的特征点的编号
 */
    //参考帧是普通帧，该地图点只与普通帧有关
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn_total[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    int cam = pFrame->keypoint_to_cam.find(idxF)->second;
    int descIdx = pFrame->cont_idx_to_local_cam_idx.find(idxF)->second;//点在局部相机的索引
//    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);//
    pFrame->mDescriptors_total[cam].row(descIdx).copyTo(mDescriptor);////////////注意用局部索引

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

/**
 * @brief 添加观测
 *
 * 记录哪些KeyFrame的那个特征点能观测到该MapPoint \n
 * 并增加观测的相机数目nObs，单目+1，双目或者grbd+2
 * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些MapPoints的关键帧是共视关键帧
 * @param pKF KeyFrame
 * @param idx MapPoint在KeyFrame中的索引
 */
void MapPoint::AddObservation(KeyFrame* pKF, size_t idx) //相机1、2间应该不算共视？？
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))// 若地图点能观测到某KF,则返回;若观测不到,则添加观测
        return;

    //添加该帧到观测
    mObservations[pKF]=idx;

    //添加该帧到相机1的观测
    if (idx < pKF->N)
    {
        mObservations_cam1[pKF]=idx;//仅cam1
    }
//    else
//    {
//        mObservations_cam2[pKF]=idx-pKF->N;
//    }

    //idx为第一个相机或者第二个相机
//    if ((idx < pKF->N) && (pKF->mvuRight[idx]>=0))
//        nObs+=2;
//    else if ((idx >= pKF->N) && pKF->mvuRight_cam2[idx-(pKF->N)]>=0) //双目或rgbd // plc
    if(pKF->mvuRight_total[idx]>=0)
        nObs+=2;
    else //单目
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
//            if(pKF->mvuRight[idx]>=0 || pKF->mvuRight_cam2[idx-(pKF->N)]>=0)
            if(pKF->mvuRight_total[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);
            if (idx < pKF->N)
            {
                mObservations_cam1.erase(pKF);//仅cam1
                //nObs_cam2-=2 点被观测数
            }
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;//在 AddObservation() 中得到
}

map<KeyFrame*, size_t> MapPoint::GetObservations_cam1()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations_cam1;//在 AddObservation() 中得到
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

int MapPoint::Observations_cam1()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}


MapPoint* MapPoint::GetReplaced()
{
//    cout<<"MapPoint.cc::L229 GetReplaced()"<<endl;
    //调用本函数时锁对象 mMutexFeatures 和 mMutexPos 保持上锁状态
    unique_lock<mutex> lock1(mMutexFeatures);
//    cout<<"MapPoint.cc::L232"<<endl;
    unique_lock<mutex> lock2(mMutexPos);
//    cout<<"MapPoint.cc::L234"<<endl;
    return mpReplaced;
}
// 在形成闭环的时候，会更新KeyFrame与MapPoint之间的关系
//主要是在使用闭环时，调整地图点和关键帧，建立新的关系
//如果被替换,当前地图点的mbBad会被记为true
//************将当前地图点（this）替换成pMp。
void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
//    cout<<"MapPoint.cc:: IncreaseVisible() ..."<<endl;
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

//计算当前点的最佳描述子（同个点在各关键帧的描述子不一样）
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors; // vDescriptors_cam2;

    map<KeyFrame*,size_t> observations; //observations_cam2;//关键帧,地图点在该关键帧上的索引?

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
//        observations_cam2=mObservations_cam2;
    }

//    if(observations.empty() && observations_cam2.empty())
    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());
//    vDescriptors_cam2.reserve(observations_cam2.size());

    // 遍历观测到3d点的所有关键帧，获得orb描述子，并插入到vDescriptors中
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
        {
            size_t l = mit->second;//特征点在关键帧的索引
            int cam = pKF->keypoint_to_cam.find(l)->second;//特征点所在的相机编号
            int descIdx = pKF->cont_idx_to_local_cam_idx.find(l)->second;//特征点的局部编号

            vDescriptors.push_back(pKF->GetDescriptor(cam, descIdx));//将该3d点所有描述子储存在这里

//            vDescriptors.push_back(pKF->mDescriptors_total.row(mit->second));//储存该点的所有描述子
//            vDescriptors_cam2.push_back(pKF->mDescriptors_cam2.row(mit->second));
        }
    }

//    if(vDescriptors.empty() && vDescriptors_cam2.empty())
    if(vDescriptors.empty())
        return;

    // Compute distances between them
    // 获得这些描述子两两之间的距离
    const size_t N = vDescriptors.size();
//    const size_t N_cam2 = vDescriptors_cam2.size(); //plc
//    const size_t N_total = N + N_cam2;

    float Distances[N][N];
    for(size_t i=0; i<N; i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
//        for(size_t j=N;j<N_total;j++)//两个相机之间描述子距离 plc
//        {
//            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors_cam2[j-N]);
//            Distances[i][j]=distij;
//            Distances[j][i]=distij;
//        }
    }

//    for(size_t i=N;i<N_total;i++)
//    {
//        Distances[i][i]=0;
//        for(size_t j=i+1;j<N_total;j++)
//        {
//            int distij = ORBmatcher::DescriptorDistance(vDescriptors_cam2[i-N],vDescriptors_cam2[j-N]);
//            Distances[i][j]=distij;
//            Distances[j][i]=distij;
//        }
//    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        // 第i个描述子到其它所有描述子之间的距离
        vector<int> vDists(Distances[i],Distances[i]+N); //用Distances[i]第0个到第N-1个（共N个）元素来初始化
        sort(vDists.begin(),vDists.end());
        // 获得中值
        int median = vDists[0.5*(N-1)];

        // 寻找最小的中值
        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;//第i个描述子到其它所有描述子之间的距离最短
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        // 最好的描述子，该描述子相对于其他描述子有最小的距离中值
        // 简化来讲，中值代表了这个描述子到其它描述子的平均距离
        // 最好的描述子就是和其它描述子的平均距离最小
        mDescriptor = vDescriptors[BestIdx].clone();
//        if(BestIdx<N)
//        {
//            mDescriptor = vDescriptors[BestIdx].clone();
//        }
//        else
//        {
//            mDescriptor = vDescriptors_cam2[BestIdx-N].clone(); //plc
//        }
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

//返回点在关键帧pKF中的索引
int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}
int MapPoint::GetIndexInKeyFrame_cam1(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations_cam1.count(pKF))
        return mObservations_cam1[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}
/**
 * @brief 更新地图点的平均观测方向以及观测距离范围
 *
 * 由于一个MapPoint会被许多相机观测到，因此在插入关键帧后，需要更新相应变量
 * @see III - C2.2 c2.4
 */
 //深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取时金字塔放大尺度，
 // 得到最大距离mfMaxDistance；最大距离除以整个金字塔最高层的放大尺度得到最小距离mfMinDistance。
 // 通常来说，距离较近的地图点，将在金字塔层数较高的地方提取出，距离较远的地图点，
 // 在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）。
 // 因此通过地图点的信息（主要是对应描述子），我们可以获得该地图点对应的金字塔层级
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations; // 获得观测到该3d点的所有关键帧//< 观测到该MP的KF和该MP在KF中的索引
//        observations_cam2 =mObservations_cam2; // (相机2)
        pRefKF=mpRefKF;   // 观测到该点的参考关键帧
        Pos = mWorldPos.clone(); // 3d点在世界坐标系中的位置
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;//能观察到当前mp的关键帧
        size_t idx = mit->second; //该点在关键帧的索引
        int cam =pKF->keypoint_to_cam.find(idx)->second;
//        cv::Mat Owi = pKF->GetCameraCenter();
        vector<cv::Mat> Owi = {pKF->GetCameraCenter(),pKF->GetCameraCenter_cam2()};
//        if(mWorldPos.at<float>(0,0))
//        {
//            cout<<"该点在观测到该点的关键帧中编号为 "<<mit->second<<",该点世界坐标为:"<<mWorldPos<<endl;
//        }
        cv::Mat normali = mWorldPos - Owi[cam];
        normal = normal + normali/cv::norm(normali); // 对所有关键帧对该点的观测方向归一化为单位向量进行求和
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter(); // 参考关键帧相机指向3D点的向量
    const float dist = cv::norm(PC);  // 该点到参考关键帧相机的距离
    const int level = pRefKF->mvKeysUn_total[observations[pRefKF]].octave;//关键点提取的金字塔层数
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];//mvScaleFactors[n] = 1.2^n
    const int nLevels = pRefKF->mnScaleLevels;//金字塔层数:tum1.yaml里是8

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor; // 观测到该点的距离下限
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1]; // 观测到该点的距离上限
        mNormalVector = normal/n;  // 获得平均的观测方向
    }

    //cam2不需要?
//    if(!observations_cam2.empty())//cam2
//    {
//        cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
//        int n=0;
//        for(map<KeyFrame*,size_t>::iterator mit=observations_cam2.begin(), mend=observations_cam2.end(); mit!=mend; mit++)
//        {
//            KeyFrame* pKF = mit->first;
//            cv::Mat Owi = pKF->GetCameraCenter();
//            cv::Mat normali = mWorldPos - Owi;//3D坐标吧？
//            normal = normal + normali/cv::norm(normali); // 对所有关键帧对该点的观测方向归一化为单位向量进行求和
//            n++;
//        }
//
//        cv::Mat PC = Pos - pRefKF->GetCameraCenter(); // 参考关键帧相机指向3D点的向量
//        const float dist = cv::norm(PC);  // 该点到参考关键帧相机的距离
//        const int level = pRefKF->mvKeysUn_cam2[observations_cam2[pRefKF]].octave;
//        const float levelScaleFactor =  pRefKF->mvScaleFactors[level];//尺度因子，不变
//        const int nLevels = pRefKF->mnScaleLevels;// 金字塔层数,不变
//
//        {
//            unique_lock<mutex> lock3(mMutexPos);
//            mfMaxDistance_cam2 = dist*levelScaleFactor; // 观测到该点的距离下限
//            mfMinDistance_cam2 = mfMaxDistance_cam2/pRefKF->mvScaleFactors[nLevels-1]; // 观测到该点的距离上限
//            mNormalVector_cam2 = normal/n;  // 获得平均的观测方向
//        }
//    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

//              ____
// Nearer      /____\     level:n-1 --> dmin (最下层)
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax (最上层)
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
// 由当前特征点的距离，推测所在的层级
// 根据深度预测地图点在帧图像上的尺度,深度大尺度小,深度小尺度大
// mvscalefactor = 1.2^level , 距离越近,level越高, scale越大
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor); //log(ratio/1.2)
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1; //最高层设为 层数-1

    //距离越小,scale越大?
    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
