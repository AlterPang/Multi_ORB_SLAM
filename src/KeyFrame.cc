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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocQuery_cam2(0),
    mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), N_cam2(F.N_cam2),N_total(F.N_total),
    mRcam12(F.mRcam12),mtcam12(F.mtcam12),//标定的变换矩阵
    mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvKeys_total(F.mvKeys_total),
    mvKeys_cam2(F.mvKeys_cam2), mvKeysUn_cam2(F.mvKeysUn_cam2),//plc
    mvKeysUn_total(F.mvKeysUn_total),
    mvuRight_total(F.mvuRight_total),//plc
    mvuRight(F.mvuRight), mvDepth(F.mvDepth),
    mvDepth_total(F.mvDepth_total),
    mvuRight_cam2(F.mvuRight_cam2), mvDepth_cam2(F.mvDepth_cam2),//plc
    mDescriptors(F.mDescriptors.clone()), mDescriptors_cam2(F.mDescriptors_cam2.clone()),
    mDescriptors_total(F.mDescriptors_total),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec),
    mBowVec_cam1(F.mBowVec_cam1), mFeatVec_cam1(F.mFeatVec_cam1), //plc
    mBowVec_cam2(F.mBowVec_cam2), mFeatVec_cam2(F.mFeatVec_cam2), //plc
//    mvKeysRays(F.mvKeysRays),//plc
    mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints),
    keypoint_to_cam(F.keypoint_to_cam), cont_idx_to_local_cam_idx(F.cont_idx_to_local_cam_idx),//plc
    mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;
    mGrid.resize(mnGridCols);//TODO mGrid可以删掉了吧
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }
    mGrids.resize(2);//按相机数分配大小
    for(int c=0;c<2;++c)
    {
        mGrids[c].resize(mnGridCols);
        for(int i=0; i<mnGridCols;i++)
        {
            mGrids[c][i].resize(mnGridRows);
            for(int j=0; j<mnGridRows; j++)
                mGrids[c][i][j] = F.mGrids[c][i][j];
        }
    }

    SetPose(F.mTcw);    
}
/**
 * @brief Bag of Words Representation
 *
 * 计算mBowVec，并且将描述子分散在第4层上，即mFeatVec记录了属于第i个node的ni个描述子
 */
void KeyFrame::ComputeBoW() //todo +cam2
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        //toDescriptorVector()将描述子矩阵mDescriptors转换成一串单行的描述子向量
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors_total);
//        vector<cv::Mat> vCurrentDesc_cam2 = Converter::toDescriptorVector(mDescriptors_cam2);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        //BowVec就是描述一张图像的一系列视觉词汇，视觉词汇的id和它的权重值
        //FeatVec就是节点的id和每个节点拥有的特征索引:map<NodeId, vector<int>>
        //mpORBvocabulary来自外部文件,是固定的
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4); //todo cam2
//        mpORBvocabulary->transform(vCurrentDesc_cam2,mBowVec_cam2,mFeatVec_cam2,4); //???
        //新增的,仅cam1; plc
        vector<cv::Mat> vCurrentDesc_cam1 = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc_cam1,mBowVec_cam1,mFeatVec_cam1,4); //todo cam2
    }
}

//添加了Tcw_cam2
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw; //twc
    // Rwc2 = Rwc1 * mRcam12;
    cv::Mat twc2 = Rwc*mtcam12 + Ow;
    cv::Mat Rc2w = mRcam12.t() * Rcw;
    cv::Mat tc2w = -Rc2w * twc2;
    Tcw_cam2 = cv::Mat::eye(4,4,Tcw.type());
    Rc2w.copyTo(Tcw_cam2.rowRange(0,3).colRange(0,3));
    tc2w.copyTo(Tcw_cam2.rowRange(0,3).col(3));

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center; //Cw是双目的中心
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}
cv::Mat KeyFrame::GetPose_cam2()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_cam2.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()//twc
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetCameraCenter_cam2()//twc2=-Rc2w.inv*tc2w = -Rwc2*tc2w
{
    unique_lock<mutex> lock(mMutexPose);
    return -Tcw_cam2.rowRange(0,3).colRange(0,3).clone().t()*Tcw_cam2.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}
cv::Mat KeyFrame::GetRotation_cam2()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_cam2.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()//tcw
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetTranslation_cam2() //tc2w
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw_cam2.rowRange(0,3).col(3).clone();
}
//为关键帧之间添加连接
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}
//每个相机分别添加连接
void KeyFrame::AddConnection_cam1(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections1);
        if(!mConnectedKeyFrameWeights_cam1.count(pKF))
            mConnectedKeyFrameWeights_cam1[pKF]=weight;
        else if(mConnectedKeyFrameWeights_cam1[pKF]!=weight)
            mConnectedKeyFrameWeights_cam1[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles_cam1();
}
//更新最佳共视
void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size()); //mConnectedKeyFrameWeights是<KeyFrame*,int>
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

void KeyFrame::UpdateBestCovisibles_cam1()
{
    unique_lock<mutex> lock(mMutexConnections1);
    vector<pair<int,KeyFrame*> > vPairs_cam1;
    vPairs_cam1.reserve(mConnectedKeyFrameWeights_cam1.size()); //mConnectedKeyFrameWeights是<KeyFrame*,int>
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights_cam1.begin(), mend=mConnectedKeyFrameWeights_cam1.end(); mit!=mend; mit++)
        vPairs_cam1.push_back(make_pair(mit->second,mit->first));

    sort(vPairs_cam1.begin(),vPairs_cam1.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs_cam1.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs_cam1[i].second);
        lWs.push_front(vPairs_cam1[i].first);
    }

    mvpOrderedConnectedKeyFrames_cam1 = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights_cam1 = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames_cam1()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights_cam1.begin();mit!=mConnectedKeyFrameWeights_cam1.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()//返回已排序的共视关键帧
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;//已按权重排序的共视关键帧,在 UpdateConnections()生成
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames_cam1()
{
    unique_lock<mutex> lock(mMutexConnections1);
    return mvpOrderedConnectedKeyFrames_cam1;
}
//TODO 这里只取综合的最大共视,两相机都要取最佳共视以增强鲁棒性?
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)//TODO
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}
//plc cam1
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames_cam1(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections1);
    if((int)mvpOrderedConnectedKeyFrames_cam1.size()<N)
        return mvpOrderedConnectedKeyFrames_cam1;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames_cam1.begin(),mvpOrderedConnectedKeyFrames_cam1.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight_cam1(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames_cam1.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights_cam1.begin(),mvOrderedWeights_cam1.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights_cam1.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights_cam1.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames_cam1.begin(), mvpOrderedConnectedKeyFrames_cam1.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

int KeyFrame::GetWeight_cam1(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights_cam1.count(pKF))
        return mConnectedKeyFrameWeights_cam1[pKF];
    else
        return 0;
}

/**
 * @brief Add MapPoint to KeyFrame
 * @param pMP MapPoint
 * @param idx MapPoint在KeyFrame中的索引
 */
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

//这里只返回有效的地图点. 另一个函数返回所有地图点,包括null
set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;//size应为特征点总数, 未生成点的为null
}

vector<MapPoint*> KeyFrame::GetMapPointMatches_cam1()
{
    unique_lock<mutex> lock(mMutexFeatures);
//    std::vector<MapPoint*> MapPoints_cam1;
//    MapPoints_cam1.resize(N);
//    for(int i=0; i<N; i++)
//    {
//        MapPoints_cam1[i]=mvpMapPoints[i];
//    }
//    return MapPoints_cam1;//size应为特征点总数, 未生成点的为null
    std::vector<MapPoint*> MapPoints_cam1(mvpMapPoints.begin(),mvpMapPoints.begin()+N);
    return MapPoints_cam1;
}

//plc 获得多相机描述子
//相机编号, 特征点的局部索引
cv::Mat KeyFrame::GetDescriptor(const int& cam, const size_t &idx) const//plc
{
    return mDescriptors_total[cam].row(idx);
}



//获得第idx个特征点坐标（全部相机中）
cv::Vec3f KeyFrame::GetKeyPointRay(const size_t &idx) const
{
    return mvKeysRays[idx];//表示全部相机的特征点综合的第dix个特征点坐标(按相机顺序依次存在一组数组里)
}

vector<cv::Vec3f> KeyFrame::GetKeyPointsRays() const
{
    return mvKeysRays;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}
//为关键帧之间添加连接，通过关键帧之间的weight连接，wight指的是两个关键帧之间共同观测到的地图点
//todo 多相机+单相机
void KeyFrame::UpdateConnections()
{
//    cout<<"KeyFrame.cc::L421: UpdateConnections()..."<<endl;
    map<KeyFrame*,int> KFcounter; // 关键帧-权重，权重为其它关键帧与当前关键帧共视3d点的个数
    map<KeyFrame*,int> KFcounter_cam1; //cam1

    vector<MapPoint*> vpMP;
    vector<MapPoint*> vpMP_cam1; //cam1
    vpMP_cam1.resize(N);

    {
        // 获得该关键帧的所有地图点
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;

        // 仅相机1的地图点
        for(int i=0; i<N; i++)
        {
            vpMP_cam1[i] = mvpMapPoints[i];
        }
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    // 通过3D点间接统计可以观测到这些3D点的所有关键帧之间的共视程度
    // 即统计每一个关键帧都有多少关键帧与它存在共视关系，统计结果放在KFcounter
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;//遍历地图点

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        // 对于每一个MapPoint点，observations记录了可以观测到该MapPoint的所有关键帧
        map<KeyFrame*,size_t> observations = pMP->GetObservations();// 函数返回mObservations

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)//共视关键帧的mnId等于当前关键帧的mnId
                continue;
            KFcounter[mit->first]++;
        }
    }
    // 仅cam1
    //遍历该关键帧中相机1得到的地图点
    for(vector<MapPoint*>::iterator vit=vpMP_cam1.begin(), vend=vpMP_cam1.end(); vit!=vend; vit++)
    {
        MapPoint* pMP_cam1 = *vit;

        if(!pMP_cam1)
            continue;

        if(pMP_cam1->isBad())
            continue;

        map<KeyFrame*,size_t> observations_cam1 = pMP_cam1->GetObservations_cam1(); //相机1的观测

        for(map<KeyFrame*,size_t>::iterator mit=observations_cam1.begin(), mend=observations_cam1.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)//共视关键帧的mnId和当前关键帧的mnId
                continue;
            KFcounter_cam1[mit->first]++;
        }
    }

    // This should not happen
//    if(KFcounter.empty() && KFcounter_cam2.empty())
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int nmax_cam1=0;
    KeyFrame* pKFmax_cam1=NULL;
    int th = 15;

    // vPairs记录与其它关键帧共视帧数大于th的关键帧
    // pair<int,KeyFrame*>将关键帧的权重写在前面，关键帧写在后面方便后面排序
    vector<pair<int,KeyFrame*> > vPairs, vPairs_cam1;
    vPairs.reserve(KFcounter.size());//KFcounter: 关键帧-权重
    vPairs_cam1.reserve(KFcounter_cam1.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            // 找到对应权重最大的关键帧（共视程度最高的关键帧）
            pKFmax=mit->first;
        }
        if(mit->second>=th)//mit: 关键帧-权重
        {
            // 对应权重需要大于阈值，对这些关键帧建立连接
            vPairs.push_back(make_pair(mit->second,mit->first));
            // 更新KFcounter中该关键帧的mConnectedKeyFrameWeights
            // 更新其它KeyFrame的mConnectedKeyFrameWeights，更新其它关键帧与当前帧的连接权重
            (mit->first)->AddConnection(this,mit->second);//this:与当前关键帧共视的关键帧
        }
    }

    //cam1 //KFcounter: 关键帧-权重
    for(map<KeyFrame*,int>::iterator mit=KFcounter_cam1.begin(), mend=KFcounter_cam1.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax_cam1)
        {
            nmax_cam1=mit->second;
            pKFmax_cam1=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs_cam1.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection_cam1(this,mit->second); //<关键帧，权重>
        }
    }

    // 如果没有超过阈值的权重，则对权重最大的关键帧建立连接
    if(vPairs.empty())
    {
        // 如果每个关键帧与它共视的关键帧的个数都少于th，
        // 那就只更新与其它关键帧共视程度最高的关键帧的mConnectedKeyFrameWeights
        // 这是对之前th这个阈值可能过高的一个补丁
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    if(vPairs_cam1.empty()) //cam1
    {
        vPairs_cam1.push_back(make_pair(nmax_cam1,pKFmax_cam1));
        pKFmax_cam1->AddConnection_cam1(this,nmax_cam1);
    }

    // vPairs里存的都是相互共视程度比较高的关键帧和共视权重，由大到小
    sort(vPairs.begin(),vPairs.end());
    sort(vPairs_cam1.begin(),vPairs_cam1.end());//cam1
    list<KeyFrame*> lKFs;
    list<int> lWs;
    list<KeyFrame*> lKFs_cam1;
    list<int> lWs_cam1;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    for(size_t i=0; i<vPairs_cam1.size();i++)
    {
        lKFs_cam1.push_front(vPairs_cam1[i].second);
        lWs_cam1.push_front(vPairs_cam1[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        // 更新图的连接(权重)
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
        //cam1
        mConnectedKeyFrameWeights_cam1 = KFcounter_cam1;
        mvpOrderedConnectedKeyFrames_cam1 = vector<KeyFrame*>(lKFs_cam1.begin(),lKFs_cam1.end());
        mvOrderedWeights_cam1 = vector<int>(lWs_cam1.begin(), lWs_cam1.end());

        // 更新生成树的连接
        if(mbFirstConnection && mnId!=0)
        {
            // 初始化该关键帧的父关键帧为共视程度最高的那个关键帧
//            mpParent = mvpOrderedConnectedKeyFrames.front();

            // todo 初始化该关键帧的父关键帧,使父帧为相机1的共视程度最高的那个关键帧
            mpParent = mvpOrderedConnectedKeyFrames_cam1.front();
//            mpParent_cam2 = mvpOrderedConnectedKeyFrames.front();
            // 建立双向连接关系
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

//更改父帧
void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

// plc: mpParent是多相机帧的父帧, 用起来有问题
KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

// 具体删除一个关键帧的步骤是这样的：
//　1) 初始mbNotErase状态是true，那么调用SetBadFlag后，将mbToBeErased状态置为true，
// 然后return，并没有执行SetBadFlag()中后面的代码。
//　2) 调用SetErase()，这时首先要检查mspLoopEdges是否是空的！因为如果当前帧维护了一个回环，删了该关键帧回环就没了。。。
// 通常情况下是空的，那么把mbNotErase置为false，此时再在SetErase()中调用SetBagFlag时，就会真正去执行删除该帧的代码了。
//
//总结一下就是，首先设置为坏帧，如果该帧不是回环帧，则可以真的删掉；如果该帧是回环帧，怎么都删不掉的。。。
void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        //如果该帧的闭环候选帧为空怎删除该帧？
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        // 设置坏帧的目的是在删除前该帧处理号父帧与子帧的关系（坏帧如果是回环帧则不删）
        SetBadFlag();
    }
}

//(父节点是与当前关键帧共视程度最高的关键帧,其他关键帧就是子节点)
//设置坏的标志
//KeyFrame中比较难理解的是setFlag()函数，真实删除当前关键帧之前，
//需要处理好父亲和儿子关键帧关系，不然会造成整个关键帧维护图的断裂或者混乱，不能够为后端提供较好的初值
//理解起来就是当前帧(父亲)挂了，儿子节点a,b,c需要找新的父亲节点，
//在候选的父亲节点中，当前帧的父亲节点（a的父亲的父亲）肯定在a的候选节点中
//说先设置成坏帧，且不是回环帧，则可以删除当前帧，如果是回环帧，那么则无法删除该帧
void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();
        mvpOrderedConnectedKeyFrames_cam1.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        // 为当前帧的儿子帧寻找新的父帧...
        // 遍历当前帧的所有儿子，然后遍历儿子A的每个共视帧，如果其中有候选父亲，则将该候选父亲设为A的父亲，
        // 并且将A放入其他儿子的候选父亲中（因为这时候A已经将整个图联系起来了）；如果没有，break。
        // 如果遍历一圈下来，发现有的儿子还没有找到新父亲，例如儿子B的共视帧不是候选父亲里的任何一个,
        // 这种情况出现在： B和当前帧的父亲不存在共视关系（速度太快，旋转太急，匹配跟丢）。
        // 并且B与当前帧的儿子之间也没有共视关系：当前帧不是一个好的关键帧，本来就没有多少儿子；
        // 或者B本身是个例外，恩，反正B是个孤家寡人。。。那么直接将当前帧的父亲设置为B的父帧
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            //遍历当前帧的子帧
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                // 遍历子帧的共视帧
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames_cam1();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        // 如果该子帧的共视帧中有候选父帧
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            // 选择权重最大的那帧
                            int w = pKF->GetWeight_cam1(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF; //该子帧
                                pP = vpConnected[i]; //该共视帧作为父帧
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);//该子帧更换父帧
                sParentCandidates.insert(pC);//该子帧加入其他子帧的候选父帧
                mspChildrens.erase(pC);//当前帧删除该子帧
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}
//在指定的区域内获取特征 // r为边长（半径）
vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N_total);

    // floor向下取整，mfGridElementWidthInv为每个像素占多少个格子
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    // ceil向上取整
    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy]; //只有相机1
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn_total[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

//多相机 //在指定的区域内获取特征 // r为边长（半径）
vector<size_t> KeyFrame::GetFeaturesInArea(const int & cam,
                                           const float &x, const float &y,
                                           const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N_total);

    //得到x,y的半径r范围的所在的格子
    // floor向下取整，mfGridElementWidthInv为每个像素占多少个格子
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    // ceil向上取整
    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        //mGrid[cam][ix][iy]里保存了该格子里的数个特征点的索引.(索引对每个相机都是0~N_cami)
        vector<std::vector<size_t>> vCell = mGrids[cam][ix];//网格储存关键帧特征点的全局索引
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
//            const vector<size_t> vCell = mGrids[cam][ix][iy];
            for(size_t j=0, jend=vCell[iy].size(); j<jend; j++)//遍历一个网格下所有特征点
            {
                const cv::KeyPoint &kpUn = mvKeysUn_total[vCell[iy][j]];//vCell[iy][j]是该特征点索引
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[iy][j]);//特征点全局编号
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);//(0-640,0-480)
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth_total[i];
    if(z>0)
    {
        const float u = mvKeys_total[i].pt.x;
        const float v = mvKeys_total[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        if (i>=0 && i<N)
        {
            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
        }
        else if (i>=N ) //&& i<N_total
        {
            unique_lock<mutex> lock(mMutexPose);
            //X3D = Rwc1 * (Rcam12 * X3Dc2 + tcam12) + twc1
            return Twc.rowRange(0,3).colRange(0,3)*(mRcam12*x3Dc+mtcam12)+Twc.rowRange(0,3).col(3);
        }
        else
            return cv::Mat();
    }
    else
        return cv::Mat();
}

//cv::Mat KeyFrame::UnprojectStereo_cam2(int i)
//{
//    const float z = mvDepth_total[i];
//    if(z>0)
//    {
//        const float u = mvKeys_cam2[i-N].pt.x;
//        const float v = mvKeys_cam2[i-N].pt.y;
//        const float x = (u-cx)*z*invfx;
//        const float y = (v-cy)*z*invfy;
//        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
//        cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);
//        unique_lock<mutex> lock(mMutexPose);
//        return Twc.rowRange(0,3).colRange(0,3)*Rcam12*x3Dc+Twc.rowRange(0,3).col(3);
//    }
//    else
//        return cv::Mat();
//}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
