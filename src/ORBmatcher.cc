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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;
/**
 * Constructor
 * @param nnratio  ratio of the best and the second score。最佳和次佳的比例
 * @param checkOri check orientation
 */
ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}
/**
 * @brief 通过投影，对Local MapPoint进行跟踪
 *
 * 将Local MapPoint投影到当前帧中, 由此增加当前帧的MapPoints \n
 * 在SearchLocalPoints()中已经将Local MapPoints重投影（isInFrustum()）到当前帧 \n
 * 并标记了这些点是否在当前帧的视野中，即mbTrackInView \n
 * 对这些MapPoints，在其投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
 * @param  F           当前帧
 * @param  vpMapPoints Local MapPoints
 * @param  th          阈值
 * @return             成功匹配的数量
 * @see SearchLocalPoints() isInFrustum()
 */
 //用于tracking 的 TrackLocalMap
int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    //遍历局部地图点
    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)//在isInFrustum()判断了
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        // 搜索窗口的大小取决于视角, 若当前视角和平均视角夹角接近0度时, r取一个较小的值
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        // 通过投影点(投影到当前帧,见isInFrustum())以及搜索窗口和预测的尺度进行搜索, 找出附近的兴趣点
        const vector<size_t> vIndices =//TODO 加上cam
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);//用于tracking中

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

/**
  * @brief  检查 给出在匹配点对 是否在 极线范围内
  * @Param kp1   帧1上的关键点kp1
  * @Param  kp2  帧2 pKF2   上的关键点kp2
  * @Param F12   帧1到帧2的基本矩阵F12    p2转置 * F * p1 = 0
  * @Param pKF2  帧2 pKF2
  * @return kp2 距离 kp1 在帧2图像上极线 的距离在合理范围内 足够小 认为有可能匹配
  */
bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];// 距离在合理范围内   3.84卡方约束
}
// 当前帧 和 参考关键帧中的地图点 进行特征匹配  匹配到已有地图点
//  关键帧和 当前帧 均用 字典单词线性表示
// 对应单词的 描述子 肯定比较相近 取对应单词的描述子进行匹配可以加速匹配
// 当前帧每个关键点的描述子 和 参考关键帧每个地图点的描述子匹配
// 保留距离最近的匹配地图点 且最短距离和 次短距离相差不大 （ mfNNratio）
// 如果需要考虑关键点的方向信息
// 统计当前帧 关键点的方向 到30步长 的方向直方图
// 保留方向直方图中最高的三个bin中 关键点 匹配的 地图点 匹配点对
/**
  * @brief 通过词包，对参考关键帧的地图点进行跟踪
  *
  * 通过bow对pKF和F中的点描述子 进行快速匹配（不属于同一node(词典单词)的特征点直接跳过匹配） \n
  * 对属于同一node(词典单词)的特征点通过描述子距离进行匹配 \n
  * 根据匹配，用参考关键帧pKF中特征点对应的MapPoint更新 当前帧F 中特征点对应的MapPoints \n
  * 每个特征点都对应一个MapPoint，因此pKF中每个特征点的MapPoint也就是F中对应点的MapPoint \n
  * 通过 距离阈值、比例阈值 和 角度投票进行剔除误匹配
  * @param  pKF                KeyFrame        参考关键帧
  * @param  F                  Current Frame  当前帧
  * @param  vpMapPointMatches  当前帧 F中关键点 匹配到的地图点MapPoints ，NULL表示未匹配
  * @return                    成功匹配的数量
*/ // 用于tracking (重定位和跟踪参考帧)中
int ORBmatcher:: SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
//    cout<<"ORBmatcher.cc::L205 SearchByBoW()..."<<endl;
// 参考关键帧 的地图点
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

//    vpMapPointMatches = vector<MapPoint*>(F.N_total,static_cast<MapPoint*>(NULL));//cam2 不参与跟踪?
    // 当前帧关键点个数 个 匹配点 (对应原关键帧 中的地图点
//    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));  //plc
    vpMapPointMatches = vector<MapPoint*>(F.N_total,static_cast<MapPoint*>(NULL));  //plc

    // 参考关键帧的地图点描述子的特征向量
//    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec_cam1;//特征向量是<节点id, 该节点下的多个特征点编号> ?
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;//特征向量是<节点id, 该节点下的多个特征点编号> ?

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];// 方向向量 直方图
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    // 关键帧和 当前帧 均用 字典单词线性表示
    // 对应单词的 描述子 肯定比较相近 取对应单词的描述子进行匹配可以加速匹配
    // 将属于同一节点(特定层)的ORB特征进行匹配
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
//    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec_cam1.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
//    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec_cam1.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

//    cout<<"ORBmatcher.cc::L236 SearchByBoW() ..."<<endl;

    //遍历特征向量FeatVec 遍历所有相机
    while(KFit != KFend && Fit != Fend)
    {
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node(单词)，才有可能是匹配点)
        if(KFit->first == Fit->first)// 同一个节点
        {
            const vector<unsigned int> vIndicesKF = KFit->second;//某一节点下的多个特征点的编号?
            const vector<unsigned int> vIndicesF = Fit->second;

        // 步骤2：遍历关键帧KF中属于该node的地图点 其对应一个描述子
//            cout<<"ORBmatcher.cc::L242 SearchByBoW() ..."<<endl;
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                //关键帧KF特征点编号
                const unsigned int realIdxKF = vIndicesKF[iKF];
                ///////////////////////////////////////////////
//                if (realIdxKF >= pKF->N)//plc 加上这句以剔除相机2的特征点
//                    continue;
                //////////////////////////////////////////////

                // 取出KF中该特征对应的MapPoint
                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;

                // 取出关键帧KF中该特征对应的描述子 //这里仅相机1
//                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);

                    //下面三行用于多相机
//                int camIdx1 = pKF->keypoint_to_cam[realIdxKF];//<size_t, int>,该帧总编号为realIdxKF的点，该点所在相机编号为
                int camIdx1 = pKF->keypoint_to_cam.find(realIdxKF)->second;
//       //         int descIdx1 = pKF->cont_idx_to_local_cam_idx[realIdxKF];//在各相机下的局部编号
                int descIdx1 = pKF->cont_idx_to_local_cam_idx.find(realIdxKF)->second;
                const cv::Mat &dKF= pKF->mDescriptors_total[camIdx1].row(descIdx1);//该点描述子


                int bestDist1=256;
                int bestIdxF =-1 ;
                int bestDist2=256;

//                cout<<"ORBmatcher.cc::L74..."<<endl;
                // 步骤3：遍历当前帧 F 中属于该node的特征点，找到最佳匹配点
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    //////////////////////////////////////////////
//                    if (realIdxF >= F.N)//plc 加上这句以剔除相机2的特征点
//                        continue;
                    //////////////////////////////////////////////

                    // 表明这个特征点已经被匹配过了，不再匹配，加快速度
                    if(vpMapPointMatches[realIdxF])
                        continue;

                    //下面三行用于多相机
                    //<size_t, int>,该帧总编号为realIdxKF的点，该点所在相机编号为
                    int camIdx2 = F.keypoint_to_cam.find(realIdxF)->second;
                    //在各相机下的局部编号
                    int descIdx2 = F.cont_idx_to_local_cam_idx.find(realIdxF)->second;
                    const cv::Mat &dF= F.mDescriptors_total[camIdx2].row(descIdx2);// 取出F中该特征对应的描述子,用的局部相机索引

//                    const cv::Mat &dF = F.mDescriptors.row(realIdxF); // 取出F中该特征对应的描述子

                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

//                cout<<"ORBmatcher.cc::L287 ..."<<endl;
                if(bestDist1<=TH_LOW)
                {
                    // mfNNratio: 最佳和次佳的比例
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF]=pMP;//与参考关键帧的pMP匹配上了

                        const cv::KeyPoint &kp = pKF->mvKeysUn_total[realIdxKF];//plc
//                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];//plc

                        if(mbCheckOrientation)
                        {
//                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;//plc
                            float rot = kp.angle-F.mvKeys_total[bestIdxF].angle;//plc
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
//            Fit = F.mFeatVec_cam1.lower_bound(KFit->first);
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }
//    cout<<"OEBmatcher.cc::L387: 通过BOW搜索到当前帧与参考关键帧有 <"<<nmatches<<"> 个匹配点"<<endl;
    return nmatches;
}
//only cam1
int ORBmatcher:: SearchByBoW_cam1(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
//    cout<<"ORBmatcher.cc::L205 SearchByBoW()..."<<endl;
// 参考关键帧 的地图点
//    cout<<"OEBmatcher.cc::L402: SearchByBoW_cam1()"<<endl;
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches_cam1();

    // 当前帧关键点个数 个 匹配点 (对应原关键帧 中的地图点
    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));  //cam2 不参与跟踪?

    // 参考关键帧的地图点描述子的特征向量
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec_cam1;//特征向量是<节点id, 该节点下的多个特征点编号> ?

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];// 方向向量 直方图
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    // 关键帧和 当前帧 均用 字典单词线性表示
    // 对应单词的 描述子 肯定比较相近 取对应单词的描述子进行匹配可以加速匹配
    // 将属于同一节点(特定层)的ORB特征进行匹配
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec_cam1.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec_cam1.end();

//    cout<<"ORBmatcher.cc::L236 SearchByBoW() ..."<<endl;

    //遍历特征向量FeatVec 遍历所有相机
    while(KFit != KFend && Fit != Fend)
    {
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node(单词)，才有可能是匹配点)
        if(KFit->first == Fit->first)// 同一个节点
        {
            const vector<unsigned int> vIndicesKF = KFit->second;//某一节点下的多个特征点的编号?
            const vector<unsigned int> vIndicesF = Fit->second;

            // 步骤2：遍历关键帧KF中属于该node的地图点 其对应一个描述子
//            cout<<"ORBmatcher.cc::L242 SearchByBoW() ..."<<endl;
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];//关键帧KF特征点编号
                ///////////////////////////////////////////////
                if (realIdxKF >= pKF->N)//plc 加上这句以剔除相机2的特征点
                    continue;
                //////////////////////////////////////////////
                MapPoint* pMP = vpMapPointsKF[realIdxKF];// 取出KF中该特征对应的MapPoint

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;

                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);//该点描述子


                int bestDist1=256;
                int bestIdxF =-1 ;
                int bestDist2=256;

//                cout<<"ORBmatcher.cc::L74..."<<endl;
                // 步骤3：遍历当前帧 F 中属于该node的特征点，找到最佳匹配点
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    //////////////////////////////////////////////
                    if (realIdxF >= F.N)//plc 加上这句以剔除相机2的特征点
                        continue;
                    //////////////////////////////////////////////

                    // 表明这个特征点已经被匹配过了，不再匹配，加快速度
                    if(vpMapPointMatches[realIdxF])
                        continue;


                    const cv::Mat &dF = F.mDescriptors.row(realIdxF); // 取出F中该特征对应的描述子

                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

//                cout<<"ORBmatcher.cc::L287 ..."<<endl;
                if(bestDist1<=TH_LOW)
                {
                    // mfNNratio: 最佳和次佳的比例
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF]=pMP;//与参考关键帧的pMP匹配上了

                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];//plc

                        if(mbCheckOrientation)
                        {
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;//plc
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec_cam1.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }
//    cout<<"OEBmatcher.cc::L387: 通过BOW搜索到当前帧与参考关键帧有 <"<<nmatches<<"> 个匹配点"<<endl;
    return nmatches;
}


/**
* @brief   为关键帧pKF中 还没有匹配到3D地图点的2D特征点 从所给的地图点中匹配地图点
* 根据Sim3变换转化到欧式变换，
* 将每个vpPoints投影到参考关键帧pKF的图像像素坐标系上，并根据尺度确定一个搜索区域，
* 根据该MapPoint的描述子 与 该区域内的 特征点 进行匹配
* 如果匹配误差小于TH_LOW即匹配成功，更新vpMatched
* @param  pKF           KeyFrame 参考关键帧(当前关键帧?)
* @param  Scw          参考关键帧的相似变换[s*R t] (当前帧的Scw,或者说Tcw)
* @param  vpPoints     地图点(闭环地图点)
* @param  vpMatched    参考关键帧特征点 对应的匹配点(大小为当前关键帧的地图点数?)
* @param  th           匹配距离 阈值
* @return              成功匹配的数量
*/
// 根据Sim3变换，将每个mvpLoopMapPoints投影到mpCurrentKF上，并根据尺度确定一个搜索区域，
//用于loop closing 线程 //多相机
int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw,
                                   const vector<MapPoint*> &vpPoints, vector<int> &vLoopMPCams,
                                   vector<MapPoint*> &vpMatched, int th, const cv::Mat CalibMatrix)
{
    cout<<"ORBmatcher.cc::L577 SearchByProjection()"<<endl;
    // Get Calibration Parameters for later projection
    // 相机内参数
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
//    const cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);//
    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//todo 检查其他类这部分公式对不对
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
//    const cv::Mat tcam12 = CalibMatrix.row(3).t();
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat tcam21 = -Rcam21 * tcam12;

    // Decompose Scw
    // 步骤1：相似变换转换到欧式变换 归一化相似变换矩阵
    // | s*R  t|
    // |  0   1|
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);// 相似变换旋转矩阵 s*R
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));// 计算相似变换矩阵的尺度s
    cv::Mat Rcw = sRcw/scw;// 归一化的 旋转矩阵
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;//  归一化的 计算相似变换矩阵
    cv::Mat Ow = -Rcw.t()*tcw;// pKF坐标系下，世界坐标系到pKF的位移，方向由世界坐标系指向pKF
    // Rwc * twc  用来计算 地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度

    // Set of MapPoints already found in the KeyFrame
    // 步骤2： 使用set类型，并去除没有匹配的点，用于快速检索某个MapPoint是否有匹配
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));//去除null(未匹配点)

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    // 步骤3：遍历所有的MapPoints(闭环关键帧及其相邻帧的地图点?)
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP]; // 闭环地图点

        // Discard Bad MapPoints and already found
        // 丢弃坏的MapPoints和已经匹配上的MapPoints
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

//        int camIdx = vLoopMPCams[iMP];// 闭环地图点的相机编号 todo 错了,要的不是点所在的相机,而是匹配点在当前帧pKF的相机
//        int camIdx = pKF->keypoint_to_cam.find(iMP)->second; // 这个也不行啊?

        // Get 3D Coords.
        // 步骤4：地图点根据变换 转到当前帧相机坐标系下
        // 地图点的世界坐标
        cv::Mat p3Dw = pMP->GetWorldPos();
//        vector<int> bestIdxs;
        int bestDist = 256;//距离上限
        int bestIdxs = -1;

        // 遍历相机+++++++++++++++++++++++++++++++++
        for(int camidx =0; camidx<2;++camidx)
        {
            // Transform into Camera Coords. //转到当前帧相机坐标系
            cv::Mat p3Dc= Rcw*p3Dw+tcw;

            if(camidx==1) //相机2的点
            {
                p3Dc = Rcam21*p3Dc+tcam21;//相机2的相机坐标
//            p3Dc = Rcam21*Rcw*p3Dw+Rcam21*tcw+tcam21;
            }

            // Depth must be positive
            if(p3Dc.at<float>(2)<0.0)
                continue;

            // Project into Image
            // 步骤5：根据相机内参数 投影到 当前帧的图像像素坐标系下
            const float invz = 1/p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0)*invz;
            const float y = p3Dc.at<float>(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            // 地图点投影过来如果不在图像范围内 就没有匹配点
            if(!pKF->IsInImage(u,v))
                continue;

            // Depth must be inside the scale invariance region of the point
            //步骤6：  判断距离是否在尺度协方差范围内 剔除
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw-Ow;
            //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
            const float dist = cv::norm(PO);

            if(dist<minDistance || dist>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            // 观察视角 必须小于 60度
            cv::Mat Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist)
                continue;

            // 步骤7： 根据尺度确定搜索半径 进而在图像上确定 候选 关键点
            int nPredictedLevel = pMP->PredictScale(dist,pKF);//更加距离预测点处于的 图像金字塔尺度

            // Search in a radius
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
            // 在图像上确定候选关键点 (vIndices储存的是特征点的索引)
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(camidx,u,v,radius);//TODO camidx

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            // 步骤8：遍历候选关键点 地图点 和 关键帧上 候选关键点 进行描述子匹配 计算距离 保留最近距离的匹配
            const cv::Mat dMP = pMP->GetDescriptor();// 地图点对应的描述子

//            int bestDist = 256;//距离上限
            int bestIdx = -1;
            // 遍历搜索区域内所有候选特征点，与该MapPoint的描述子进行匹配
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
            {
                const size_t idx = *vit; // idx是全局索引
                //跳过已经匹配上的MapPoints的像素点
                if(vpMatched[idx])
                    continue;

                // 候选关键点 不在 由地图点预测的尺度到 最高尺度范围内 直接跳过
                const int &kpLevel= pKF->mvKeysUn_total[idx].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                // 计算距离 保存 最短的距离 对应的 关键点
                int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;// 特征点的局部索引
                const cv::Mat &dKF = pKF->mDescriptors_total[camidx].row(descIdx);// 关键点对应的描述子

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist; //两相机的最佳匹配距离
//                    bestIdx = idx; //当前相机下,若无最佳匹配则为-1
                    bestIdxs = idx; //两相机的最佳匹配idx
                }
            }
//            if (bestIdx < 0)
//                continue;
//            if (bestDist <= TH_LOW)
//            {
//                bestIdxs.push_back(bestIdx); //闭环地图点在当前帧的最佳匹配
////                cam2bestIdxs.push_back(cam);
//            }

        } // 遍历相机结束 +++++++++++++++++++++++++++++++++

        if(bestDist<=TH_LOW)//50
        {
            vpMatched[bestIdxs]=pMP;// 该 特征点 匹配到的 地图点
            nmatches++;
        }
//        if(bestIdxs.size() > 0)//50
//        {
//            if(bestIdxs.size() ==2 )
//            {
//
//            }
//
//            for (int f = 0; f < bestIdxs.size(); ++f)
//            {
//                vpMatched[bestIdxs[f]]=pMP;// 该特征点 匹配到的 地图点
//                nmatches++;
//            }
//        }
    }

    return nmatches;
}
// 根据sim3投影找到更多匹配点
//loop closing , 单相机
int ORBmatcher::SearchByProjection_cam1(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int &kpLevel= pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);//

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);//

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}
/**
  * @brief 通过词包，对关键帧的特征点进行跟踪，该函数用于闭环检测时两个关键帧间的特征点匹配
  *
  * 通过bow对pKF1和pKF2中的特征点进行快速匹配（不属于同一node(单词)的特征点直接跳过匹配） \n
  * 对属于同一node的特征点通过描述子距离进行匹配 \n
  * 根据匹配，更新vpMatches12 \n
  * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
  * @param  pKF1               KeyFrame1
  * @param  pKF2               KeyFrame2
  * @param  vpMatches12        pKF1中与pKF2匹配的MapPoint，null表示没有匹配
  * @return                    成功匹配的数量
  */ //只用与闭环检测线程的计算sim3 //todo 仅相机1
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{//TODO 改成双/多相机
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn_total;// 关键帧1 特征点
    //特征向量类型: map<NodeId, vector<unsigned int> > (不同node上的特征点)
    // 关键帧1 特征点词典描述向量(所有相机)
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
//    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec_cam1;
    //关键帧1特征点匹配的地图点
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
//    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
    //关键帧1特征点的描述子矩阵
//    const cv::Mat &Descriptors1 = pKF1->mDescriptors;
    const vector<cv::Mat> &Descriptors1_total = pKF1->mDescriptors_total;//所有相机描述子

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn_total;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
//    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec_cam1;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
//    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches_cam1();
//    const cv::Mat &Descriptors2 = pKF2->mDescriptors;
    const vector<cv::Mat> &Descriptors2_total = pKF2->mDescriptors_total;

    // 为关键帧1的地图点 初始化 匹配点
    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);// 关键帧地图点 匹配标记

    // 统计匹配点对的 方向差值  同一个匹配 方向相差不大
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    // 将属于同一节点(特定层)的ORB特征进行匹配 //todo 要不要修改成Multicol的
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
        if(f1it->first == f2it->first)// node id
        {
        // 步骤2：遍历KF1中属于该node的特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                //kf1的一个特征点索引
                const size_t idx1 = f1it->second[i1];

                // 取出KF1中该特征对应的地图点
                MapPoint* pMP1 = vpMapPoints1[idx1];
                // 没有匹配的地图点跳过
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                // 取出KF1中该特征对应的描述子
                //特征点所在相机编号
                int camidx1 = pKF1->keypoint_to_cam.find(idx1)->second;
                //特征点在局部相机的编号
                int descidx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;
                const cv::Mat &d1 = Descriptors1_total[camidx1].row(descidx1);
//                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;

             // 步骤3：遍历KF2中属于该node的特征点，找到了最佳匹配点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    //对应的地图点
                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    // 已经和 KF1中某个点匹配过了 或者 该地图点是坏点, 跳过
                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    // 步骤4：求描述子的距离 保留最小和次小距离对应的匹配点
                    // 取出F中该特征对应的描述子
                    //特征点所在相机编号
                    int camidx2 = pKF2->keypoint_to_cam.find(idx2)->second;
                    //特征点在局部相机的编号
                    int descidx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;
                    const cv::Mat &d2 = Descriptors2_total[camidx2].row(descidx2); //局部相机索引
//                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);

                    if(dist<bestDist1)// dist < bestDist1 < bestDist2，更新bestDist1 bestDist2
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;// 最小的距离
                        bestIdx2=idx2;// 对应 KF2 地图点下标
                    }
                    else if(dist<bestDist2)// bestDist1 < dist < bestDist2，更新bestDist2
                    {
                        bestDist2=dist;// 次短的距离
                    }
                }

                // 步骤4：根据阈值 和 角度投票剔除误匹配 ,//todo MultiCOl里不根据角度剔除误匹配
                if(bestDist1<TH_LOW)
                {
                    // 最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];// 匹配到的对应KF2中的地图点
                        vbMatched2[bestIdx2]=true;// KF2 中的地图点已经和KF1中的某个地图点匹配

                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}


/**
  * @brief 通过词包，对关键帧的特征点进行跟踪，该函数用于闭环检测时两个关键帧间的特征点匹配
  *
  * 通过bow对pKF1和pKF2中的特征点进行快速匹配（不属于同一node(单词)的特征点直接跳过匹配） \n
  * 对属于同一node的特征点通过描述子距离进行匹配 \n
  * 根据匹配，更新vpMatches12 \n
  * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
  * @param  pKF1               KeyFrame1
  * @param  pKF2               KeyFrame2
  * @param  vpMatches12        pKF1中与pKF2匹配的MapPoint，null表示没有匹配
  * @return                    成功匹配的数量
  */ //只用与闭环检测线程的计算sim3 //仅相机1
int ORBmatcher::SearchByBoW_cam1(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;// 关键帧1 特征点
    //特征向量类型: map<NodeId, vector<unsigned int> > (不同node上的特征点)
    // 关键帧1 特征点词典描述向量(所有相机)
//    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec_cam1;
    //关键帧1特征点匹配的地图点
//    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
    //关键帧1特征点的描述子矩阵
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;
//    const vector<cv::Mat> &Descriptors1_total = pKF1->mDescriptors_total;//所有相机描述子

//    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn_total;
    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
//    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec_cam1;
//    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches_cam1();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;
//    const vector<cv::Mat> &Descriptors2_total = pKF2->mDescriptors_total;

    // 为关键帧1的地图点 初始化 匹配点
    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);// 关键帧地图点 匹配标记

    // 统计匹配点对的 方向差值  同一个匹配 方向相差不大
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    // 将属于同一节点(特定层)的ORB特征进行匹配 //todo 要不要修改成Multicol的
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        //步骤1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
        if(f1it->first == f2it->first)// node id
        {
            // 步骤2：遍历KF1中属于该node的特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                //kf1的一个特征点索引
                const size_t idx1 = f1it->second[i1];

                // 取出KF1中该特征对应的地图点
                MapPoint* pMP1 = vpMapPoints1[idx1];
                // 没有匹配的地图点跳过
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                // 取出KF1中该特征对应的描述子
                //特征点所在相机编号
//                int camidx1 = pKF1->keypoint_to_cam.find(idx1)->second;
                //特征点在局部相机的编号
//                int descidx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;
//                const cv::Mat &d1 = Descriptors1_total[camidx1].row(descidx1);
                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;

                // 步骤3：遍历KF2中属于该node的特征点，找到了最佳匹配点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    //对应的地图点
                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    // 已经和 KF1中某个点匹配过了 或者 该地图点是坏点, 跳过
                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    // 步骤4：求描述子的距离 保留最小和次小距离对应的匹配点
                    // 取出F中该特征对应的描述子
                    //特征点所在相机编号
//                    int camidx2 = pKF2->keypoint_to_cam.find(idx2)->second;
                    //特征点在局部相机的编号
//                    int descidx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;
//                    const cv::Mat &d2 = Descriptors2_total[camidx2].row(descidx2); //局部相机索引
                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);

                    if(dist<bestDist1)// dist < bestDist1 < bestDist2，更新bestDist1 bestDist2
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;// 最小的距离
                        bestIdx2=idx2;// 对应 KF2 地图点下标
                    }
                    else if(dist<bestDist2)// bestDist1 < dist < bestDist2，更新bestDist2
                    {
                        bestDist2=dist;// 次短的距离
                    }
                }

                // 步骤4：根据阈值 和 角度投票剔除误匹配 ,//todo MultiCOl里不根据角度剔除误匹配
                if(bestDist1<TH_LOW)
                {
                    // 最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];// 匹配到的对应KF2中的地图点
                        vbMatched2[bestIdx2]=true;// KF2 中的地图点已经和KF1中的某个地图点匹配

                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

/**
 * @brief 利用基本矩阵F12，在两个关键帧之间未匹配的特征点中产生新的3d点
 *
 * @param pKF1          关键帧1
 * @param pKF2          关键帧2
 * @param F12           基础矩阵
 * @param vMatchedPairs 存储匹配特征点对，特征点用其在关键帧中的索引表示
 * @param bOnlyStereo   在双目和rgbd情况下，要求特征点在右图存在匹配
 * @param vbCam         plc: 多相机中参与三角化的相机, 由基线条件判断
 * @return              成功匹配的数量
 */
//利用三角化，在两个关键帧之间恢复出一些地图点SearchForTriangulation
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                       vector<pair<size_t, size_t> > &vMatchedPairs,
                                       const bool bOnlyStereo, vector<bool> vbCam)
{
//    cout<<"ORBmatcher.cc::L883: SearchForTriangulation() ..."<<endl;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++计算F12部分
//    vector<vector<cv::Mat>> F12s(2);
    vector<cv::Mat> F12s(2);
    // Essential Matrix 本质矩阵: t12叉乘R12
    // Fundamental Matrix 基础及裤子: inv(K1)*E*inv(K2)
    //Rcw1  tcw1
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R1w_cam2 = pKF1->GetRotation_cam2();
    cv::Mat t1w_cam2 = pKF1->GetTranslation_cam2();
    //Rcw2  tcw2
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat R2w_cam2 = pKF2->GetRotation_cam2();
    cv::Mat t2w_cam2 = pKF2->GetTranslation_cam2();

//    cv::Mat R12 = R1w*R2w.t();
//    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

//    vector<vector<cv::Mat>> R12s(2); //todo 不同相机间的基础矩阵
//    vector<vector<cv::Mat>> t12s(2);
    vector<cv::Mat> R12s(2);
    vector<cv::Mat> t12s(2);
    //各相机间的R12 t12;  i j表示两帧间的相机i j
    //todo 暂时只有同相机的点三角化 i=0,j=0 或 i=1,j=1
//    R12s[0][0] = R1w*R2w.t();
    R12s[0]= R1w*R2w.t();
//    t12s[0][0] = -R1w*R2w.t()*t2w+t1w;
    t12s[0]= -R1w*R2w.t()*t2w+t1w;
//    R12s[0][1] = R1w*R2w_cam2.t();
//    t12s[0][1] = -R1w*R2w_cam2.t()*t2w_cam2+t1w;
//    R12s[1][0] = R1w_cam2*R2w.t();
//    t12s[1][0] = -R1w_cam2*R2w.t()*t2w+t1w_cam2;
//    R12s[1][1] = R1w_cam2*R2w_cam2.t();
    R12s[1]= R1w_cam2*R2w_cam2.t();
//    t12s[1][1] = -R1w_cam2*R2w_cam2.t()*t2w_cam2+t1w_cam2;
    t12s[1]= -R1w_cam2*R2w_cam2.t()*t2w_cam2+t1w_cam2;

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

//    cv::Mat t12x = SkewSymmetricMatrix(t12);
    vector<cv::Mat> t12xs(2);
    for (int i=0;i<2;i++)
    {
//        vector<cv::Mat> temp(2);
//        for(int j=0;j<2; j++) //j=0;j<2
//        {
//            t12xs[j] = SkewSymmetricMatrix(t12s[i][j]); //反对称矩阵
//            temp[j] =  K1.t().inv()*t12xs[j]*R12s[i][j]*K2.inv();
//        }
//        F12s[i] = temp;
        t12xs[i] = SkewSymmetricMatrix(t12s[i]);
        F12s[i] = K1.t().inv()*t12xs[i]*R12s[i]*K2.inv();
    }
//    cout<<"ORBmatcher.cc::L933..."<<endl;
//    F12 =  K1.t().inv()*t12x*R12*K2.inv();
    //++++++++++++++++++++++++++++++++++++++++++++++++++计算F12部分

    //特征向量//map<NodeId, vector<unsigned int> >??;FeatVec就是节点的id和节点拥有的特征的索引
    //mFeatVec->second是每一层node拥有的vCurrentDesc的索引
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    // 计算KF1的相机中心在KF2图像平面的坐标，即极点坐标
    cv::Mat Cw = pKF1->GetCameraCenter();// twc1； 相机1到世界坐标系
    cv::Mat Cw_cam2 = pKF1->GetCameraCenter_cam2();// twc1_cam2；

//    cv::Mat R2w = pKF2->GetRotation(); // Rc2w; 世界坐标系到相机2
//    cv::Mat t2w = pKF2->GetTranslation(); // tc2w; 世界坐标系到相机2
    cv::Mat C2 = R2w*Cw+t2w; // tc2c1 KF1的相机中心在KF2坐标系的表示
    cv::Mat C2_cam2 = R2w_cam2*Cw_cam2+t2w_cam2; // tc2c1 KF1 cam2的相机中心在KF2 cam2坐标系的表示
    //坐标z的倒数1/z
    const float invz = 1.0f/C2.at<float>(2);
    // 步骤0：得到KF1的相机光心在KF2中的坐标（极点坐标）
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

    const float invz_cam2 = 1.0f/C2_cam2.at<float>(2);
    // 步骤0：得到KF1的相机光心在KF2中的坐标（极点坐标）
    const float ex_cam2 =pKF2->fx*C2_cam2.at<float>(0)*invz_cam2+pKF2->cx;
    const float ey_cam2 =pKF2->fy*C2_cam2.at<float>(1)*invz_cam2+pKF2->cy;

    //TODO //添加多相机功能(>=3)
    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<cv::KeyPoint> vKeys1 = pKF1->mvKeysUn_total;//kf1的所有特征点
//    vector<cv::KeyPoint> vKeys1_cam2 = pKF1->mvKeysUn_cam2;//
//    vector<cv::Vec3f> vKeysRays1 = pKF1->GetKeyPointsRays();//特征点坐标？ 没用到

    vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    vector<cv::KeyPoint> vKeys2 = pKF2->mvKeysUn_total;
//    vector<cv::KeyPoint> vKeys2_cam2 = pKF2->mvKeysUn_cam2;
//    vector<cv::Vec3d> vKeysRays2 = pKF2->GetKeyPointsRays();

    int nmatches=0;
    vector<bool> vbMatched2(vKeys2.size(), false);//kf2特征点是否已匹配
    vector<int> vMatches12(vKeys1.size(), -1); //kf1的特征点上匹配到kf2的点

    //rotHist是什么？Rotation Histogram (to check rotation consistency)旋转直方图（检查旋转一致性）？？
    vector<int> rotHist[HISTO_LENGTH];//30
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

//    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
//    // 将属于同一节点(特定层)的ORB特征进行匹配
//    // FeatureVector的数据结构类似于：{(node1,feature_vector1) (node2,feature_vector2)...}
//    // f1it->first对应node编号，f1it->second对应属于该node的所有特特征点编号
        // 用特征向量目的是为了匹配同个节点下的特征点?
//    cout<<"ORBmatcher.cc::L999..."<<endl;
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();
    // 步骤1：遍历pKF1和pKF2中词典线性表示的特征向量树中 的node(节点)
    while(f1it!=f1end && f2it!=f2end)
    {
        // 如果f1it和f2it属于同一个node节点
        if(f1it->first == f2it->first)
        {
            // 步骤2：遍历该node(节点)(f1it->first)下kf1的所有特征点(f1it->second)
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                // 获取pKF1中属于该node的特征点索引
                const size_t idx1 = f1it->second[i1];

//                if() TODO 增加vbcam

                // 步骤2.1：通过特征点索引idx1在pKF1中取出对应的MapPoint
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);

                // If there is already a MapPoint skip
                // 由于寻找的是未匹配的特征点，所以pMP1应该为NULL (已经存在的pMP则跳过)
                if(pMP1)
                    continue;

                //<size_t, int> <该特征点在该帧的总编号，所在的相机编号>
                int camIdx1 = pKF1->keypoint_to_cam.find(idx1)->second;//kf1中得到特征点在其所在的相机编号

                if (vbCam[camIdx1]==false) continue; // todo 如果该相机不参与三角化,跳过

                //<size_t, int>为<点在多相机帧系统的总编号，在各相机下分别的编号>
                int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;//得到点在相机下的编号

                // 如果mvuRight中的值大于0，表示是双目，且该特征点有深度值
                const bool bStereo1 = pKF1->mvuRight_total[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;

                // 步骤2.2：通过特征点索引idx1在pKF1中取出对应的特征点
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn_total[idx1];

                // 步骤2.3：通过特征点索引idx1在pKF1中取出对应的特征点的描述子
                const cv::Mat &d1 = pKF1->GetDescriptor(camIdx1, descIdx1);//得到该特征(idx1)的描述子
//                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                int bestDist = TH_LOW;
                int bestIdx2 = -1;

                // 步骤3：遍历该node节点下(f2it->first)的kf2所有特征点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    // 获取pKF2中属于该node节点的所有特征点索引(编号)
                    size_t idx2 = f2it->second[i2];

                    // 步骤3.1：通过特征点索引idx2在pKF2中取出对应的MapPoint
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

                    // If we have already matched or there is a MapPoint skip
                    // 如果pKF2当前特征点索引idx2已经被匹配过或者对应的3d点非空
                    // 那么这个索引idx2就不能被考虑
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    int camIdx2 = pKF2->keypoint_to_cam.find(idx2)->second;//kf2中该特征点所在的相机编号
                    //TODO 暂仅匹配同个相机间的点
                    if (camIdx1 != camIdx2)//两帧中的点的相机编号，若不是同一个相机的点，则跳过
                        continue;
                    int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;//得到该点在所在相机下的编号

                    const bool bStereo2 = pKF2->mvuRight_total[idx2]>=0;

                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;

                    // 步骤3.2：通过特征点索引idx2在pKF2中取出对应的特征点的描述子
//                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                    const cv::Mat &d2 = pKF2->GetDescriptor(camIdx2, descIdx2);//得到该特征(idx2)的描述子

                    // 计算idx1与idx2在两个关键帧中对应特征点的描述子距离
                    const int dist = DescriptorDistance(d1,d2);

                    if(dist>TH_LOW || dist>bestDist)
                        continue;

                    // 步骤3.3：通过特征点索引idx2在pKF2中取出对应的特征点
//                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[idx2];

                    if(!bStereo1 && !bStereo2)//单目时
                    {
                        //ex、ey是KF1的相机光心在KF2中的坐标（极点坐标）//像素坐标?？
                        float distex[2];
                        float distey[2];
                        distex[0] = ex-kp2.pt.x;
                        distey[0] = ey-kp2.pt.y;
                        distex[1] = ex_cam2-kp2.pt.x;
                        distey[1] = ey_cam2-kp2.pt.y;
                        // 该特征点距离极点太近，表明kp2对应的MapPoint距离pKF1相机太近
//                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                        if(distex[camIdx2]*distex[camIdx2]+distey[camIdx2]*distey[camIdx2]<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }

                    // 步骤4：计算特征点kp2到kp1极线（kp1对应pKF2的一条极线）的距离是否小于阈值
                    if(CheckDistEpipolarLine(kp1,kp2,F12s[camIdx1],pKF2)) //todo F12s[camIdx1][camIdx2]
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                // 步骤1、2、3、4总结下来就是：将图像1的每个特征点与图像2同一node节点的所有特征点
                // 依次检测，判断是否满足对极几何约束，满足约束就是匹配的特征点
                // 详见SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)函数步骤4
                if(bestIdx2>=0)
                {
                    //通过特征点索引bestidx2在pKF2中取出对应的特征点
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[bestIdx2];
                    //匹配kf1的编号idx1的特征点与kf2的bestidx2（即找到最佳匹配？）
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        // trick!
                        // angle：每个特征点在提取描述子时的旋转主方向角度，如果图像旋转了，这个角度将发生改变
                        // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                        float rot = kp1.angle-kp2.angle;// 该特征点的角度变化值
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);// 将rot分配到bin组
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
            //如果f1it节点编号小于f2it
        else if(f1it->first < f2it->first)
        {
            //lower_bound:指向一个不小于()内的iterator, 这里即不小于f2it->first
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }


//    //遍历kf1的地图点
//    for (size_t idx1 = 0; idx1 < vpMapPoints1.size(); ++idx1)
//    {
//        MapPoint* pMP1 = vpMapPoints1[idx1];
//        // If there is already a MapPoint associated skip
//        if (pMP1)//特征点存在相应地图点则跳过,否则构造该地图点
//            continue;
//
//        // get observations
//        const cv::KeyPoint &kp1 = vKeys1[idx1];//kf1的特征点
////        const cv::Vec3d &ray1 = vKeysRays1[idx1];//kf1的特征点坐标？ 没用到
//
//        //<size_t, int> <该特征点在该帧的总编号，所在的相机编号>
//        int camIdx1 = pKF1->keypoint_to_cam.find(idx1)->second;//kf1中得到特征点在其所在的相机编号,这里是0或1
//        //<size_t, int>为<点在多相机帧系统的总编号，在各相机下分别的编号>
//        int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;//得到点在相机下的编号
//                //TODO cam2呢
//        const cv::Mat d1 = pKF1->GetDescriptor(camIdx1, descIdx1);//得到该特征(idx1)的描述子
////        const cv::Mat* d1 = pKF1->GetDescriptor(camIdx1, descIdx1);//这里用指针吗?
////        const uint64_t* d1_mask = 0;
////        if (havingMasks)
////            d1_mask = pKF1->GetDescriptorMaskRowPtr(camIdx1, descIdx1);//描述子掩码,什么用?
//        int bestDist = TH_LOW;
//        int bestIdx2 = -1;
//
//        vector<pair<int, size_t> > vDistIndex;//kf2的某个描述子距离,该特征点编号
//        vector<int> vDistCamIndex; // cam idx of match;相机编号
//        // match so second multikeyframe
//        for (size_t idx2 = 0; idx2 < vpMapPoints2.size(); ++idx2)//遍历kf2地图点
//        {
//            MapPoint* pMP2 = vpMapPoints2[idx2];
//
//            // If we have already matched or there is a MapPoint skip
//            // 如果pKF2当前特征点索引idx2已经被匹配过或者对应的3d点非空
//            // 那么这个索引idx2就不能被考虑
//            if (vbMatched2[idx2] || pMP2)
//                continue;
//            // get corresponding descriptor in second image
//            int camIdx2 = pKF2->keypoint_to_cam.find(idx2)->second;//kf2中该特征点所在的相机编号
//            //TODO for the moment take only matches between the same camera
//            if (camIdx1 != camIdx2)//两帧中的点的相机编号，若不是同一个相机的点，则跳过<<<<<<<<<<<<
//                continue;
//            int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;//得到该点在所在相机下的编号
//
//                    //TODO cam2呢
//            // 步骤3.2：通过特征点索引idx2在pKF2中取出对应的特征点的描述子
//            const cv::Mat d2 = pKF2->GetDescriptor(camIdx2, descIdx2);//得到该特征(idx2)的描述子
//
////            int dist = 0;
//            // 计算idx1与idx2在两个关键帧中对应特征点的描述子距离
//            const int dist = DescriptorDistance(d1,d2);//两帧中同相机下描述子距离
////            if (havingMasks)
////            {
////                // 计算idx1与idx2在两个关键帧中对应特征点的描述子距离
////                const uint64_t* d2_mask = pKF2->GetDescriptorMaskRowPtr(camIdx2, descIdx2);
////                dist = DescriptorDistance64Masked(d1, d2, d1_mask, d2_mask, mbFeatDim);
////            }
////            else
////                dist = DescriptorDistance64(d1, d2, mbFeatDim);
//
//            if (dist > TH_LOW || dist>bestDist)
//                continue;
//
//            vDistIndex.push_back(make_pair(dist, idx2));//kf2的描述子idx2与kf1的idx1距离,idx2编号
//            vDistCamIndex.push_back(camIdx2);//kf2特征点所在的相机编号
//
//            // 步骤3.3：通过特征点索引idx2在pKF2中取出对应的特征点
//            const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[idx2];
//
//            if (vDistIndex.empty())
//                continue;
//
//
//            // 步骤4：计算特征点kp2到kp1极线（kp1对应pKF2的一条极线）的距离是否小于阈值
//            if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
//            {
//                bestIdx2 = idx2;
//                bestDist = dist;
//            }
//        }
//        // 步骤1、2、3、4总结下来就是：将左图像的每个特征点与右图像同一node节点的所有特征点
//        // 依次检测，判断是否满足对极几何约束，满足约束就是匹配的特征点
//
//        // 详见SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)函数步骤4
//        if(bestIdx2>=0)
//        {
//            //通过特征点索引bestidx2在pKF2中取出对应的特征点
//            const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[bestIdx2];
//            //匹配kf1的编号idx1的特征点与kf2的bestidx2（即找到最佳匹配？）
//            vMatches12[idx1]=bestIdx2;
//            nmatches++;
//
//            if(mbCheckOrientation)
//            {
//                // trick!
//                // angle：每个特征点在提取描述子时的旋转主方向角度，如果图像旋转了，这个角度将发生改变
//                // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
//                float rot = kp1.angle-kp2.angle;// 该特征点的角度变化值
//                if(rot<0.0)
//                    rot+=360.0f;
//                int bin = round(rot*factor);// 将rot分配到bin组
//                if(bin==HISTO_LENGTH)
//                    bin=0;
//                assert(bin>=0 && bin<HISTO_LENGTH);
//                rotHist[bin].push_back(idx1);
//            }
//        }
//    }

//    cout<<"ORBmatcher.cc::L1253..."<<endl;
    // 根据方向剔除误匹配的点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        // 计算rotHist中最大的三个的index
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            // 如果特征点的旋转角度变化量属于这三个组，则保留
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            // 将除了ind1 ind2 ind3以外的匹配点去掉
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)//vMatches12[idx1]=-1(无匹配)或idx2(对应的匹配)
    {
        if(vMatches12[i]<0)//非匹配点则跳过
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}

////当前关键帧MapPoints投影到相邻关键帧  TODO
//int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
//{
//    cv::Mat Rcw = pKF->GetRotation();
//    cv::Mat tcw = pKF->GetTranslation();
//
//    const float &fx = pKF->fx;
//    const float &fy = pKF->fy;
//    const float &cx = pKF->cx;
//    const float &cy = pKF->cy;
//    const float &bf = pKF->mbf;
//    const cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);
//    const cv::Mat Rcam21 = Rcam12.inv();
//
//    cv::Mat Ow = pKF->GetCameraCenter();
//
//    int nFused=0;// 融合地图点的数量
//
//    const int nMPs = vpMapPoints.size();
//
//    //步骤1： 遍历当前关键帧所有的MapPoints
//    for(int i=0; i<nMPs; i++)
//    {
//        MapPoint* pMP = vpMapPoints[i];
//
//        if(!pMP)
//            continue;
//
//        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
//            continue;
//
//        // 步骤3： 将当前帧地图点投影在相邻关键帧 图像像素坐标上
//        cv::Mat p3Dw = pMP->GetWorldPos(); // 当前帧地图点在世界坐标系下的坐标
//        cv::Mat p3Dc;//地图点在关键帧下相机坐标系下的坐标
//        vector<int> bestIdxs;
//
//        for (int cam = 0; cam < 2 ; cam++)//TODO
//        {
//            if (cam=0) //相机1
//                p3Dc = Rcw*p3Dw + tcw;
//            else if (cam=1) //相机2
//                p3Dc = Rcam21*Rcw*p3Dw + Rcam21*tcw; //TODO 多相机系统标定后需要修改
//
//            // Depth must be positive
//            if(p3Dc.at<float>(2)<0.0f)
//                continue;
//
//            const float invz = 1/p3Dc.at<float>(2);
//            const float x = p3Dc.at<float>(0)*invz;// 相机(坐标系)归一化尺度下的坐标
//            const float y = p3Dc.at<float>(1)*invz;
//
//            const float u = fx*x+cx;
//            const float v = fy*y+cy; // 得到MapPoint在图像上的投影坐标
//
//            // Point must be inside the image
//            // 像素坐标 必须在 图像尺寸范围内
//            if(!pKF->IsInImage(u,v))
//                continue;
//
//            const float ur = u-bf*invz;
//
//            //步骤4：  判断距离是否在尺度协方差范围内
//            const float maxDistance = pMP->GetMaxDistanceInvariance();
//            const float minDistance = pMP->GetMinDistanceInvariance();
//            //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
//            cv::Mat PO = p3Dw-Ow;
//            const float dist3D = cv::norm(PO);
//
//            // Depth must be inside the scale pyramid of the image
//            if(dist3D<minDistance || dist3D>maxDistance )
//                continue;
//
//            // Viewing angle must be less than 60 deg
//            //步骤5：观察视角 必须小于 60度
//            cv::Mat Pn = pMP->GetNormal();//观测方向(单位向量)
//
//            if(PO.dot(Pn)<0.5*dist3D)
//                continue;
//
//            // 根据深度预测地图点在帧图像上的尺度,深度大尺度小,深度小尺度大
//            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);//返回金字塔尺度
//
//            // Search in a radius
//            // 步骤6：根据该MapPoint的深度确定尺度，从而确定搜索范围,进而在图像上确定 候选关键点
//            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
//
//            // 收集当前关键帧pKF在该区域内的特征点
//            const vector<size_t> vIndices = pKF->GetFeaturesInArea(cam,u,v,radius);
//
//            if(vIndices.empty())
//                continue;
//
//            // Match to the most similar keypoint in the radius
//            // 步骤7：遍历候选关键点,计算与地图点描述子匹配 计算距离 保留最近距离的匹配
//            const cv::Mat dMP = pMP->GetDescriptor();// 相邻帧的地图点描述子
//
//            int bestDist = 256;
//            int bestIdx = -1;
//            // 遍历搜索范围内的features
//            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++) // 遍历搜索范围内的features
//            {
//                const size_t idx = *vit;
//
//                // 关键点的尺度 需要在预测尺度 之上
//                const cv::KeyPoint &kp = pKF->mvKeysUn_total[idx];
//
//                const int &kpLevel= kp.octave; // 关键点尺度
//
//                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
//                    continue;
//
//                //步骤8:  计算MapPoint投影的坐标与这个区域特征点的距离，如果偏差很大，直接跳过特征点匹配
//                // 深度/双目相机  有 右图像 匹配点横坐标差值
//                if(pKF->mvuRight_total[idx]>=0)//(当前帧)
//                {
//                    // Check reprojection error in stereo
//                    const float &kpx = kp.pt.x;
//                    const float &kpy = kp.pt.y;
//                    const float &kpr = pKF->mvuRight_total[idx];
//                    const float ex = u-kpx;
//                    const float ey = v-kpy;
//                    const float er = ur-kpr;
//                    const float e2 = ex*ex+ey*ey+er*er;
//
//                    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
//                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
//                        continue;
//                }
//                else
//                {
//                    const float &kpx = kp.pt.x;
//                    const float &kpy = kp.pt.y;
//                    const float ex = u-kpx;
//                    const float ey = v-kpy;
//                    const float e2 = ex*ex+ey*ey;
//
//                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
//                        continue;
//                }
//
//                const cv::Mat &dKF = pKF->mDescriptors_total[cam].row(idx);
//
//                const int dist = DescriptorDistance(dMP,dKF);
//
//                if(dist<bestDist) // 找MapPoint在该区域最佳匹配的特征点
//                {
//                    bestDist = dist;
//                    bestIdx = idx;
//                }
//            }
//            if (bestIdx < 0)
//                continue;
//            if (bestDist <= TH_LOW)
//                bestIdxs.push_back(bestIdx);
//        }//相机循环结束
//
//        // If there is already a MapPoint replace otherwise add new measurement
//        MapPoint* pMPinKF;
//        if(bestIdxs.size() > 0) // 找到了MapPoint在该区域最佳匹配的特征点
//        {
//            for (int f = 0; f < bestIdxs.size(); ++f)
//            {
//                pMPinKF = pKF->GetMapPoint(bestIdxs[f]);  //pKF是当前正在处理的关键帧
//                // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
//                if(pMPinKF) // 如果这个点有对应的MapPoint
//                {
//                    if(!pMPinKF->isBad()) // 如果这个MapPoint不是bad，选择哪一个呢？
//                    {
//                        if(pMPinKF->Observations()>pMP->Observations()) //替换观测大的那个点???? (pMP是正在遍历的特征点)
//                            pMP->Replace(pMPinKF);
//                        else
//                            pMPinKF->Replace(pMP);
//                    }
//                }
//                    // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
//                else // 如果这个点没有对应的MapPoint
//                {
//                    pMP->AddObservation(pKF,bestIdxs[f]);
//                    pKF->AddMapPoint(pMP,bestIdxs[f]);
//                }
//                nFused++;// 融合次数++
//            }
//        }
//    }
//
//    return nFused;
//}

// 将MapPoints投影到关键帧pKF中，并判断是否有重复的MapPoints
/**
* @brief 将MapPoints投影到关键帧pKF中，并判断是否有重复的MapPoints
* 1.如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
* 2.如果MapPoint能匹配关键帧 的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
* @param  pKF          相邻关键帧/当前关键帧
* @param  vpMapPoints 需要融合的当前帧/相邻帧上MapPoints
* @param  th           搜索半径的因子
* @return              重复MapPoints的数量
*/ //Local Mapping线程
int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const cv::Mat CalibMatrix, const float th)
{
//    cout<<"ORBmatcher.cc::L1262: Fuse()..."<<endl; //todo 暂只有同相机的点融合
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();


    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat tcam21 = -Rcam21 * tcam12;

    cv::Mat Ow_cam1 = pKF->GetCameraCenter();
    cv::Mat Ow_cam2 = pKF->GetCameraCenter_cam2();
    vector<cv::Mat> Ow={Ow_cam1,Ow_cam2};

    int nFused=0;// 融合地图点的数量

    const int nMPs = vpMapPoints.size();

    //步骤1： 遍历 当前帧/相邻帧 上要融合的MapPoints
    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        // 步骤3： 将 相邻帧地图点 投影在 当前关键帧图像像素坐标上; 或者 当前帧地图点 投影在 相邻帧
        cv::Mat p3Dw = pMP->GetWorldPos(); // 相邻帧/当前帧地图点在世界坐标系下的坐标
        cv::Mat p3Dc;//地图点在关键帧下相机坐标系下的坐标
        vector<int> bestIdxs;
        for (int cam = 0; cam < 2 ; cam++)//+++++++++++++++++++++++将地图点分别投影到关键帧的相机1及相机2上
        {
            if (cam == 0)
            {
                p3Dc = Rcw*p3Dw + tcw;
            }
            else if (cam = 1) //相机2
            {
//                p3Dc = Rcam21*Rcw*p3Dw + Rcam21*tcw;
                p3Dc = Rcam21*Rcw*p3Dw+Rcam21*tcw+tcam21;//相机2的相机坐标 done√
            }

            // Depth must be positive
            if(p3Dc.at<float>(2)<0.0f)
                continue;

            // 相机(坐标系)归一化尺度下的坐标
            const float invz = 1/p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0)*invz;
            const float y = p3Dc.at<float>(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy; // 得到MapPoint在图像上的投影坐标

            // Point must be inside the image
            // 像素坐标必须在图像尺寸范围内
            if(!pKF->IsInImage(u,v))//删掉非本相机上的点???
                continue;

            const float ur = u-bf*invz;

            //步骤4：  判断距离是否在尺度协方差范围内
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            //地图点 距离相机的距离,进而推断在图像金字塔中可能的尺度 越远尺度小 越近尺度大
            cv::Mat PO = p3Dw-Ow[cam];

            const float dist3D = cv::norm(PO);

            // Depth must be inside the scale pyramid of the image
            if(dist3D<minDistance || dist3D>maxDistance )
                continue;

            // Viewing angle must be less than 60 deg
            //步骤5：观察视角 必须小于60度(当前观察视角和平均观察视角的角度吗?)
            cv::Mat Pn = pMP->GetNormal();//平均观测方向(向量)(能观测到该点的所有关键帧对该点的观测方向的平均值)

            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            // 根据深度预测地图点在帧图像上的尺度,深度大尺度小,深度小尺度大
            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);//返回金字塔尺度

//            int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx)->second;//局部索引
            // Search in a radius
            // 步骤6：根据该MapPoint的深度确定尺度，从而确定搜索范围,进而在图像上确定 候选关键点
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            // 收集当前/相邻关键帧pKF在该区域内的特征点 //TODO 不用区分相机吧?几个相机都要搜索吧?
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(cam,u,v,radius);//uv是地图点的像素坐标

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            // 步骤7：遍历候选关键点,计算与地图点描述子匹配 计算距离 保留最近距离的匹配
            const cv::Mat dMP = pMP->GetDescriptor();// 地图点(相邻帧)描述子

            int bestDist = 256;
            int bestIdx = -1;
            // 遍历搜索范围内的features
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++) // 遍历搜索范围内的features
            {
                const size_t idx = *vit;//候选关键帧在区域内的特征点索引

                // 关键点的尺度 需要在预测尺度 之上
                const cv::KeyPoint &kp = pKF->mvKeysUn_total[idx];

                const int &kpLevel= kp.octave; // 关键点尺度

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                //步骤8:  计算MapPoint投影的坐标与这个区域特征点的距离，如果偏差很大，直接跳过特征点匹配
                // 深度/双目相机  有 右图像 匹配点横坐标差值
                if(pKF->mvuRight_total[idx]>=0)//(当前帧)
                {
                    // Check reprojection error in stereo
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float &kpr = pKF->mvuRight_total[idx];
                    const float ex = u-kpx;
                    const float ey = v-kpy;
                    const float er = ur-kpr;
                    const float e2 = ex*ex+ey*ey+er*er;

                    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                        continue;
                }
                else//单目相机?
                {
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float ex = u-kpx;
                    const float ey = v-kpy;
                    const float e2 = ex*ex+ey*ey;

                    if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                        continue;
                }

//                int cam = pKF->keypoint_to_cam.find(idx)->second;//特征点所在的相机编号
                int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;//局部索引
                const cv::Mat &dKF = pKF->mDescriptors_total[cam].row(descIdx);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist) // 找MapPoint在该区域最佳匹配的特征点
                {
                    bestDist = dist; //描述子距离最小
                    bestIdx = idx; //描述子距离最小的特征点
                }
            }
            if (bestIdx < 0)
                continue;
            if (bestDist <= TH_LOW)
                bestIdxs.push_back(bestIdx); //保存最佳匹配特征点索引
        }//相机循环结束

        // If there is already a MapPoint replace otherwise add new measurement
        MapPoint* pMPinKF;
        if(bestIdxs.size() > 0) // 找到了MapPoint在该区域最佳匹配的特征点
        {
            for (int f = 0; f < bestIdxs.size(); ++f)
            {
                //bestIdxs保存的是匹配关键点的全局索引
                pMPinKF = pKF->GetMapPoint(bestIdxs[f]);  //pKF是当前正在处理的关键帧
            // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
                if(pMPinKF) // 如果这个点有对应的MapPoint
                {
                    if(!pMPinKF->isBad()) // 如果这个MapPoint不是bad
                    {
                        if(pMPinKF->Observations()>pMP->Observations()) //替换成观测大的那个点???? (pMP是正在遍历的特征点)
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                }
            // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
                else // 如果这个点没有对应的MapPoint
                {
                    pMP->AddObservation(pKF,bestIdxs[f]);
                    pKF->AddMapPoint(pMP,bestIdxs[f]);
                }
                nFused++;// 融合次数++
            }
        }
    }

    return nFused;
}
/**
  * @brief 将MapPoints投影到 关键帧pKF 中，并判断是否有重复的MapPoints
  * Scw为世界坐标系到pKF机体坐标系的Sim3 相似变换变换 ，
  * 需要先将相似变换转换到欧式变换SE3 下  将世界坐标系下的vpPoints变换到机体坐标系
  * 1 地图点匹配到 帧 关键点 关键点有对应的地图点时， 用帧关键点对应的地图点 替换 原地图点
  * 2 地图点匹配到 帧 关键点 关键点无对应的地图点时，为该特征点 添加匹配到的地图点MapPoint
  * @param  pKF          相邻关键帧(闭环帧的相邻帧,要进行矫正的帧?) *当前帧及相邻帧
  * @param  Scw          世界坐标系到pKF相机坐标系的Sim3 相似变换变换  [s*R t]
  * @param  vpPoints     需要融合的MapPoints(闭环关键帧及其相邻帧的地图点)
  * @param  vLoopMPCams  @vpPoints 的各点所在的相机编号
  * @param  th           搜索半径的因子
  * @param  vpReplacePoint
  * @return              重复MapPoints的数量
  */
// Scw为世界坐标系到pKF机体坐标系的Sim3变换，用于将世界坐标系下的vpPoints变换到机体坐标系
//  用于 Loop Closing线程 的闭环后矫正
// ☆☆☆☆☆闭环关键字及其相邻帧的地图点投影到当前关键帧及相邻帧?☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆☆
// TODO 不能用之前点所在的相机投影啊,要投影到现在观察到该地图点的相机上
// TODO (如何判断地图点应该投影到哪个相机上?两相机都投影?)
int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw,
                     const vector<MapPoint *> &vpPoints, vector<int> &vLoopMPCams,
                     float th, vector<MapPoint *> &vpReplacePoint,const cv::Mat CalibMatrix) //4x3标定矩阵
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

//    const cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);
//    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//done
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
//    const cv::Mat tcam12 = CalibMatrix.row(3).t();
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat tcam21 = -Rcam21 * tcam12;

    // Decompose Scw
    // 将Sim3转化为SE3并分解
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0))); // 计算得到尺度s
    cv::Mat Rcw = sRcw/scw; // 除掉s
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw; // 除掉s
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    // 关键帧已有的匹配地图点
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();// 需要融合的地图点数量

    // For each candidate MapPoint project and match
    // 步骤1： 遍历所有需要融合的MapPoints (闭环关键帧及其相邻帧的地图点)
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        //步骤2： 跳过不好的地图点
        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

//        int cam = vLoopMPCams[iMP]; //该地图点所在相机 todo 这里应该用不到,应该投影到两相机再看是否在相机范围内

        // Get 3D Coords.
        //步骤3： 地图点 投影到 关键帧 像素平面上 不在平面内的 不考虑
        cv::Mat p3Dw = pMP->GetWorldPos();

        //todo 遍历各相机? 参考multicol
        //todo 遍历相机,因为要知道该地图点应投影到哪个相机上
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        vector<int> bestIdxs;
        for (int camidx = 0; camidx < 2; ++camidx)
        {
            cv::Mat p3Dc;
            if (camidx == 1)
                p3Dc = Rcam21*Rcw*p3Dw+Rcam21*tcw+tcam21; //相机2坐标系
            else
                p3Dc = Rcw*p3Dw+tcw;

            // Depth must be positive
            if(p3Dc.at<float>(2)<0.0f)
                continue;

            // Project into Image
            //投影到像素平面
            const float invz = 1.0/p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0)*invz;
            const float y = p3Dc.at<float>(1)*invz;

            // 地图点在相机1或2的像素坐标
            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(u,v))
                continue;

            // Depth must be inside the scale pyramid of the image
            //步骤4： 判断距离是否在尺度协方差范围内
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            //Ow 是 twc
            cv::Mat PO = p3Dw-Ow;

            // 相机2原点的世界坐标 Ow2 = Rwc1 *tcam12 + twc1= Rcw.t()*tcam12 + Ow
            if(camidx == 1)
                PO = PO- Rcw.t() *tcam12;

            //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
            const float dist3D = cv::norm(PO);

            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            //步骤5：观察视角 必须小于 60度cv::Mat p3Dc
            // 这个角度指的是当前相机与点的连线 与 对该点的平均观察方向的夹角 ??
            cv::Mat Pn = pMP->GetNormal();

            // TODO 60度应该不需要改
            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            // Compute predicted scale level
            // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // Search in a radius
            // 步骤6： 根据尺度确定搜索半径 进而在图像上确定 候选关键点
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            // 地图点投影到帧的camidx相机查找候选关键点(两个相机都投影一下)
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(camidx, u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            // 步骤7：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
            const cv::Mat dMP = pMP->GetDescriptor();//pMP是闭环关键帧及其相邻帧的地图点吧?

            int bestDist = INT_MAX;
            int bestIdx = -1;
            // 遍历候选关键点(方块区域内的点)
            for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
            {
                const size_t idx = *vit; //候选关键点的索引(已确定相机)
                // 关键点的尺度需要在预测尺度之上
                const int &kpLevel = pKF->mvKeysUn_total[idx].octave;// 特征点提取的金字塔层数

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                // 步骤8：计算地图点和 关键帧 特征点 描述子之间的距离 选出最近距离的 关键点
//                int camIdx = pKF->keypoint_to_cam.find(idx)->second;// 候选关键点所在的相机
                int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
                const cv::Mat &dKF = pKF->mDescriptors_total[camidx].row(descIdx);//只计算同个相机下的描述子距离?
//            const cv::Mat &dKF = pKF->mDescriptors.row(idx);// 关键点描述子

                int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;// 对应的描述子的下标
                }
            }
            if (bestIdx < 0)
                continue;

            if (bestDist <= TH_LOW)
                bestIdxs.push_back(bestIdx);

        }// 相机遍历完毕 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


        // 原来的代码 +++++++++++++
//        cv::Mat p3Dc;
//        if (cam == 1)
//            p3Dc = Rcam21*Rcw*p3Dw+Rcam21*tcw+tcam21; //相机2的坐标系
//        else
//            p3Dc = Rcw*p3Dw+tcw;
//
//        // Depth must be positive
//        if(p3Dc.at<float>(2)<0.0f)
//            continue;
//
//        // Project into Image
//        //投影到像素平面
//        const float invz = 1.0/p3Dc.at<float>(2);
//        const float x = p3Dc.at<float>(0)*invz;
//        const float y = p3Dc.at<float>(1)*invz;
//
//        const float u = fx*x+cx;
//        const float v = fy*y+cy;
//
//        // Point must be inside the image
//        if(!pKF->IsInImage(u,v))
//            continue;
//
//        // Depth must be inside the scale pyramid of the image
//        //步骤4： 判断距离是否在尺度协方差范围内
//        const float maxDistance = pMP->GetMaxDistanceInvariance();
//        const float minDistance = pMP->GetMinDistanceInvariance();
//        //Ow 是 twc
//        cv::Mat PO = p3Dw-Ow;
//
//        // 相机2原点的世界坐标 Ow2 = Rwc1 *tcam12 + twc1= Rcw.t()*tcam12 + Ow
//        if(cam == 1)
//            PO = PO- Rcw.t() *tcam12;
//
//        //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
//        const float dist3D = cv::norm(PO);
//
//        if(dist3D<minDistance || dist3D>maxDistance)
//            continue;
//
//        // Viewing angle must be less than 60 deg
//        //步骤5：观察视角 必须小于 60度cv::Mat p3Dc
//        // 这个角度指的是当前相机与点的连线 与 对该点的平均观察方向的夹角 ??
//        cv::Mat Pn = pMP->GetNormal();
//
//        // TODO 60度应该不需要改
//        if(PO.dot(Pn)<0.5*dist3D)
//            continue;
//
//        // Compute predicted scale level
//        // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
//        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);
//
//        // Search in a radius
//        // 步骤6： 根据尺度确定搜索半径 进而在图像上确定 候选关键点
//        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
//
//        const vector<size_t> vIndices = pKF->GetFeaturesInArea(cam, u,v,radius);//cam是待融合的地图点所在相机
//
//        if(vIndices.empty())
//            continue;
//
//        // Match to the most similar keypoint in the radius
//        // 步骤7：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
//        const cv::Mat dMP = pMP->GetDescriptor();//pMP是闭环关键帧及其相邻帧的地图点吧?
//
//        int bestDist = INT_MAX;
//        int bestIdx = -1;
//        // 遍历候选关键点(方块区域内的点)
//        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
//        {
//            const size_t idx = *vit; //候选关键点的索引(已确定相机)
//            // 关键点的尺度需要在预测尺度之上
//            const int &kpLevel = pKF->mvKeysUn_total[idx].octave;// 特征点提取的金字塔层数
//
//            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
//                continue;
//
//            // 步骤8：计算地图点和 关键帧 特征点 描述子之间的距离 选出最近距离的 关键点
//            int camIdx = pKF->keypoint_to_cam.find(idx)->second;// 候选关键点所在的相机
//            int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
//            const cv::Mat &dKF = pKF->mDescriptors_total[camIdx].row(descIdx);//只计算同个相机下的描述子距离?
////            const cv::Mat &dKF = pKF->mDescriptors.row(idx);// 关键点描述子
//
//            int dist = DescriptorDistance(dMP,dKF);
//
//            if(dist<bestDist)
//            {
//                bestDist = dist;
//                bestIdx = idx;// 对应的描述子的下标
//            }
//        }
//
//        // If there is already a MapPoint replace otherwise add new measurement
//        // 找到了地图点MapPoint在该区域最佳匹配的特征点
//        if(bestDist<=TH_LOW)//50
//        {
//            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
//            // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，
//            if(pMPinKF)
//            {
//                if(!pMPinKF->isBad())
//                    vpReplacePoint[iMP] = pMPinKF;// 用关键点对应的地图点 替换 原地图点
//            }
//            // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
//            // 关键帧 特征点还没有匹配的地图点 把匹配到的地图点 对应上去
//            else
//            {
//                pMP->AddObservation(pKF,bestIdx);// pMP 地图点 观测到了 帧pKF 上第 bestIdx 个 特征点
//                pKF->AddMapPoint(pMP,bestIdx);// 帧 的 第 bestIdx 个 特征点 对应pMP地图点
//            }
//            nFused++;
//        }
        // 原来的代码结束 ++++++++++++++++++++++++++++++++

        MapPoint* pMPinKF = NULL;
        if(bestIdxs.size() > 0) // 投影到多相机中至少存在一个最佳匹配点
        {
            for (int f = 0; f< bestIdxs.size(); ++f)
            {
                pMPinKF = pKF->GetMapPoint(bestIdxs[f]);
                // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，
                if(pMPinKF)
                {
                    if(!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;// 用关键点对应的地图点 替换 原地图点
                }
                    // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
                    // 关键帧 特征点还没有匹配的地图点 把匹配到的地图点 对应上去
                else
                {
                    pMP->AddObservation(pKF,bestIdxs[f]);// pMP 地图点 观测到了 帧pKF 上第 bestIdx 个 特征点
                    pKF->AddMapPoint(pMP,bestIdxs[f]);// 帧 的 第 bestIdx 个 特征点 对应pMP地图点
                }
                nFused++; //这里的nFused这里无论是替代还是添加新的都+1; MultiCol里是只有替代才+1
            }
        }
    }

    return nFused;
}

//用于 Loop Closing线程 的闭环后矫正// todo 这个应该多相机fuse
int ORBmatcher::Fuse_cam1(KeyFrame *pKF, cv::Mat Scw,
                     const vector<MapPoint *> &vpPoints,
                     float th, vector<MapPoint *> &vpReplacePoint) //4x3标定矩阵
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;


    // Decompose Scw
    // 将Sim3转化为SE3并分解
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0))); // 计算得到尺度s
    cv::Mat Rcw = sRcw/scw; // 除掉s
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw; // 除掉s
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    // 关键帧已有的匹配地图点
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();// 需要融合的地图点数量

    // For each candidate MapPoint project and match
    // 步骤1： 遍历所有需要融合的MapPoints (闭环关键帧及其相邻帧的地图点)
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        //步骤2： 跳过不好的地图点
        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

//        int cam = vLoopMPCams[iMP]; //该地图点所在相机 todo 这里应该用不到,应该投影到两相机再看是否在相机范围内

        // Get 3D Coords.
        //步骤3： 地图点 投影到 关键帧 像素平面上 不在平面内的 不考虑
        cv::Mat p3Dw = pMP->GetWorldPos();

        //todo 遍历各相机? 参考multicol
        //todo 遍历相机,因为要知道该地图点应投影到哪个相机上
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            cv::Mat p3Dc = Rcw*p3Dw+tcw;
            // Depth must be positive
            if(p3Dc.at<float>(2)<0.0f)
                continue;

            // Project into Image
            //投影到像素平面
            const float invz = 1.0/p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0)*invz;
            const float y = p3Dc.at<float>(1)*invz;

            // 地图点在相机1或2的像素坐标
            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(u,v))
                continue;

            // Depth must be inside the scale pyramid of the image
            //步骤4： 判断距离是否在尺度协方差范围内
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            //Ow 是 twc
            cv::Mat PO = p3Dw-Ow;

            //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
            const float dist3D = cv::norm(PO);

            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            //步骤5：观察视角 必须小于 60度cv::Mat p3Dc
            // 这个角度指的是当前相机与点的连线 与 对该点的平均观察方向的夹角 ??
            cv::Mat Pn = pMP->GetNormal();

            // TODO 60度应该不需要改
            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            // Compute predicted scale level
            // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // Search in a radius
            // 步骤6： 根据尺度确定搜索半径 进而在图像上确定 候选关键点
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            // 地图点投影到帧的camidx相机查找候选关键点(两个相机都投影一下)
//            const vector<size_t> vIndices = pKF->GetFeaturesInArea(camidx, u,v,radius);
            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            // 步骤7：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
            const cv::Mat dMP = pMP->GetDescriptor();//pMP是闭环关键帧及其相邻帧的地图点吧?

            int bestDist = INT_MAX;
            int bestIdx = -1;
            // 遍历候选关键点(方块区域内的点)
            for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
            {
                const size_t idx = *vit; //候选关键点的索引(已确定相机)
                // 关键点的尺度需要在预测尺度之上
//                const int &kpLevel = pKF->mvKeysUn_total[idx].octave;// 特征点提取的金字塔层数
                const int &kpLevel = pKF->mvKeysUn[idx].octave;// 特征点提取的金字塔层数

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                // 步骤8：计算地图点和 关键帧 特征点 描述子之间的距离 选出最近距离的 关键点
//                int camIdx = pKF->keypoint_to_cam.find(idx)->second;// 候选关键点所在的相机
//                int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
//                const cv::Mat &dKF = pKF->mDescriptors_total[camidx].row(descIdx);//只计算同个相机下的描述子距离?
            const cv::Mat &dKF = pKF->mDescriptors.row(idx);// 关键点描述子

                int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;// 对应的描述子的下标
                }
            }


        // 原来的代码 +++++++++++++
//        cv::Mat p3Dc;
//        if (cam == 1)
//            p3Dc = Rcam21*Rcw*p3Dw+Rcam21*tcw+tcam21; //相机2的坐标系
//        else
//            p3Dc = Rcw*p3Dw+tcw;
//
//        // Depth must be positive
//        if(p3Dc.at<float>(2)<0.0f)
//            continue;
//
//        // Project into Image
//        //投影到像素平面
//        const float invz = 1.0/p3Dc.at<float>(2);
//        const float x = p3Dc.at<float>(0)*invz;
//        const float y = p3Dc.at<float>(1)*invz;
//
//        const float u = fx*x+cx;
//        const float v = fy*y+cy;
//
//        // Point must be inside the image
//        if(!pKF->IsInImage(u,v))
//            continue;
//
//        // Depth must be inside the scale pyramid of the image
//        //步骤4： 判断距离是否在尺度协方差范围内
//        const float maxDistance = pMP->GetMaxDistanceInvariance();
//        const float minDistance = pMP->GetMinDistanceInvariance();
//        //Ow 是 twc
//        cv::Mat PO = p3Dw-Ow;
//
//        // 相机2原点的世界坐标 Ow2 = Rwc1 *tcam12 + twc1= Rcw.t()*tcam12 + Ow
//        if(cam == 1)
//            PO = PO- Rcw.t() *tcam12;
//
//        //地图点 距离相机的距离 进而推断 在图像金字塔中可能的尺度 越远尺度小 越近尺度大
//        const float dist3D = cv::norm(PO);
//
//        if(dist3D<minDistance || dist3D>maxDistance)
//            continue;
//
//        // Viewing angle must be less than 60 deg
//        //步骤5：观察视角 必须小于 60度cv::Mat p3Dc
//        // 这个角度指的是当前相机与点的连线 与 对该点的平均观察方向的夹角 ??
//        cv::Mat Pn = pMP->GetNormal();
//
//        // TODO 60度应该不需要改
//        if(PO.dot(Pn)<0.5*dist3D)
//            continue;
//
//        // Compute predicted scale level
//        // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
//        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);
//
//        // Search in a radius
//        // 步骤6： 根据尺度确定搜索半径 进而在图像上确定 候选关键点
//        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
//
//        const vector<size_t> vIndices = pKF->GetFeaturesInArea(cam, u,v,radius);//cam是待融合的地图点所在相机
//
//        if(vIndices.empty())
//            continue;
//
//        // Match to the most similar keypoint in the radius
//        // 步骤7：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
//        const cv::Mat dMP = pMP->GetDescriptor();//pMP是闭环关键帧及其相邻帧的地图点吧?
//
//        int bestDist = INT_MAX;
//        int bestIdx = -1;
//        // 遍历候选关键点(方块区域内的点)
//        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
//        {
//            const size_t idx = *vit; //候选关键点的索引(已确定相机)
//            // 关键点的尺度需要在预测尺度之上
//            const int &kpLevel = pKF->mvKeysUn_total[idx].octave;// 特征点提取的金字塔层数
//
//            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
//                continue;
//
//            // 步骤8：计算地图点和 关键帧 特征点 描述子之间的距离 选出最近距离的 关键点
//            int camIdx = pKF->keypoint_to_cam.find(idx)->second;// 候选关键点所在的相机
//            int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
//            const cv::Mat &dKF = pKF->mDescriptors_total[camIdx].row(descIdx);//只计算同个相机下的描述子距离?
////            const cv::Mat &dKF = pKF->mDescriptors.row(idx);// 关键点描述子
//
//            int dist = DescriptorDistance(dMP,dKF);
//
//            if(dist<bestDist)
//            {
//                bestDist = dist;
//                bestIdx = idx;// 对应的描述子的下标
//            }
//        }
//
        // If there is already a MapPoint replace otherwise add new measurement
        // 找到了地图点MapPoint在该区域最佳匹配的特征点
        if(bestDist<=TH_LOW)//50
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;// 用关键点对应的地图点 替换 原地图点
            }
            // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
            // 关键帧 特征点还没有匹配的地图点 把匹配到的地图点 对应上去
            else
            {
                pMP->AddObservation(pKF,bestIdx);// pMP 地图点 观测到了 帧pKF 上第 bestIdx 个 特征点
                pKF->AddMapPoint(pMP,bestIdx);// 帧 的 第 bestIdx 个 特征点 对应pMP地图点
            }
            nFused++;
        }
        // 原来的代码结束 ++++++++++++++++++++++++++++++++

//        MapPoint* pMPinKF = NULL;
//        if(bestIdxs.size() > 0) // 投影到多相机中至少存在一个最佳匹配点
//        {
//            for (int f = 0; f< bestIdxs.size(); ++f)
//            {
//                pMPinKF = pKF->GetMapPoint(bestIdxs[f]);
//                // 步骤10： 如果MapPoint能匹配关键帧的特征点，并且该特征点有对应的MapPoint，
//                if(pMPinKF)
//                {
//                    if(!pMPinKF->isBad())
//                        vpReplacePoint[iMP] = pMPinKF;// 用关键点对应的地图点 替换 原地图点
//                }
//                    // 步骤11：如果MapPoint能匹配关键帧的特征点，并且该特征点没有对应的MapPoint，那么为该特征点点添加地图点MapPoint
//                    // 关键帧 特征点还没有匹配的地图点 把匹配到的地图点 对应上去
//                else
//                {
//                    pMP->AddObservation(pKF,bestIdxs[f]);// pMP 地图点 观测到了 帧pKF 上第 bestIdx 个 特征点
//                    pKF->AddMapPoint(pMP,bestIdxs[f]);// 帧 的 第 bestIdx 个 特征点 对应pMP地图点
//                }
//                nFused++; //这里的nFused这里无论是替代还是添加新的都+1; MultiCol里是只有替代才+1
//            }
//        }
    }

    return nFused;
}

/**
 * @brief  通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，
 * 同理，确定pKF2的特征点在pKF1中的大致区域
 * 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，
 * 更新vpMatches12（之前使用SearchByBoW进行特征点匹配时会有漏匹配）
 * @param pKF1          关键帧1(当前关键帧)
 * @param pKF2          关键帧2(候选关键帧)
 * @param vpMatches12   两帧原有匹配点  帧1 特征点 匹配到 帧2 的地图点
 * @param s12              帧2->帧1 相似变换 尺度, stereo/rgbd尺度固定为1
 * @param R12             帧2->帧1  欧式变换 旋转矩阵
 * @param t12              帧2->帧1 欧式变换 平移向量
 * @param th       		 搜索半径参数
 * @return                     成功匹配的数量
 */ // 通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，获得更多匹配点
 //用于Closing Looping 线程 通过得到的sim3来搜索
//todo Multicol似乎是只在同相机间闭环搜索sim3
int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame * pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12,
                             const float th, const cv::Mat CalibMatrix) //todo 4x3标定矩阵
{

    cout<<"ORBmatcher.cc::L2356 SearchBySim3()"<<endl;
    // 步骤1：变量初始化-----------------------------------------------------
    //相机内参数
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//done
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat tcam21 = -Rcam21 * tcam12;
    // Camera 1 from world
    // 世界坐标系 到 帧1 的欧式变换Rcw tcw
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    // 世界坐标系 到 帧2 的 欧式变换
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    // 相似变换 旋转矩阵 平移向量
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();//帧1->帧2相似变换旋转矩阵 = 帧1->帧2相似变换尺度 * 帧1->帧2欧式变换旋转矩阵
    cv::Mat t21 = -sR21*t12;

    // 帧1地图点数量  关键点数量
//    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();
    // 帧2地图点数量 关键点数量
//    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches_cam1();
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();
    // 来源于两帧先前已有的匹配
    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    // 步骤2：用vpMatches12更新已有的匹配 vbAlreadyMatched1和vbAlreadyMatched2------------------------
    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];// 帧1特征点到帧2的已匹配的地图点
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;// 帧1特征点已经有匹配到的地图点了
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
//            int idx2 = pMP->GetIndexInKeyFrame_cam1(pKF2);// 帧2的地图点在帧2中对应的下标
            if(idx2>=0 && idx2<N2)//在帧2特征点个数范围内的话
                vbAlreadyMatched2[idx2]=true;// 帧2地图点在帧1中也已经有匹配
        }
    }

    // 新寻找的匹配
    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    // 步骤3：通过Sim变换，确定pKF1的地图点在pKF2帧图像中的大致区域，
    //  在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新vpMatches12
    //  （之前使用SearchByBoW进行特征点匹配时会有漏匹配）
    // 每一个帧1中的地图点 投影到 帧2 上
    // 遍历帧1地图点
    for(int i1=0; i1<N1; i1++)//遍历每一个帧1中的地图点
    {
        //步骤3.1： 跳过已有的匹配 和 不存在的点 以及坏点
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        int camIdx1 = pKF1->keypoint_to_cam.find(i1)->second; // todo 帧1的点在帧1的相机编号
        //步骤3.2： 帧1地图点(世界坐标系)--> 帧1地图点(帧1坐标系)--> 帧1地图点(帧2坐标系)-->帧2像素坐标
        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;

        // 帧1地图点投影到帧2的相机坐标系 TODO 经过sim3变换后
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;
        // todo 帧2中相机2的点再转换到相机2
        if (camIdx1 == 1)
        {
            p3Dc2 = Rcam21 * p3Dc2 +tcam21;
        }

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        // 投影到帧2像素坐标
        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        // 坐标必须在 图像平面尺寸内
        if(!pKF2->IsInImage(u,v))
            continue;

        //步骤3.3：  判断帧1地图点距帧2的距离 是否在尺度协方差范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        // 步骤3.4：根据深度确定尺度 再根据尺度确定搜索半径 进而在图像上确定候选关键点
        //根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);// 尺度 也就是在 金字塔哪一层

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];// 再根据尺度确定搜索半径

        // TODO camIdx1表示两帧间的同相机
        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(camIdx1,u,v,radius);
//        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        // 步骤3.5：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
        // 帧1地图点描述子
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        // 遍历搜索 帧2该区域内的所有特征点，与帧1地图点pMP进行描述子匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;//帧2候选区域内的特征点索引
            //  关键点的尺度 需要在 预测尺度nPredictedLevel 之上

            const cv::KeyPoint &kp = pKF2->mvKeysUn_total[idx];// 帧2候选区域内的关键点
//            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];// 帧2候选区域内的关键点

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            // 帧2 关键点描述子
//            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

//            int camIdx = pKF2->keypoint_to_cam.find(idx)->second;//TODO 这里不求帧2中对应特征点的相机索引, 因为已经确定是同相机的闭环
            int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
            const cv::Mat &dKF = pKF2->mDescriptors_total[camIdx1].row(descIdx2);//只计算同个相机下的描述子距离?
            // 帧1 地图点描述子 和 帧2关键点 描述子 距离
            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)//100
        {
            vnMatch1[i1]=bestIdx;// 帧1地图点 匹配到的 帧2的关键点(也对应一个地图点)
        }
    }

    // Transform from KF2 to KF2 and search
    // 步骤4：通过Sim变换，确定pKF2的地图点在pKF1帧图像中的大致区域，
    //   在该区域内通过描述子进行匹配捕获pKF2和pKF1之前漏匹配的特征点，更新vpMatches12
    //  （之前使用SearchByBoW进行特征点匹配时会有漏匹配）
    // 每一个帧2中的地图点 投影到 帧1 上
    for(int i2=0; i2<N2; i2++)
    {
        //步骤4.1：跳过已有的匹配 和 不存在的点 以及坏点
        MapPoint* pMP = vpMapPoints2[i2];// 帧2 关键点匹配的 地图点

        if(!pMP || vbAlreadyMatched2[i2])// 不存在匹配的地图点 或者 已经和 帧1匹配了 跳过
            continue;

        if(pMP->isBad())
            continue;

        int camIdx2 = pKF2->keypoint_to_cam.find(i2)->second; //帧2的点本身所在的相机
        //步骤4.2： 帧2地图点(世界坐标系)--> 帧2地图点(帧2坐标系)--> 帧2地图点(帧1坐标系)-->帧1像素坐标系下
        cv::Mat p3Dw = pMP->GetWorldPos();
        //帧2地图点在帧2的相机坐标
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;

        // 帧2地图点投影到帧1坐标系
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;
        //再投影到帧1各相机坐标
        if(camIdx2==1)
        {
            p3Dc1 = Rcam21 * p3Dc1 +tcam21;
        }

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        // 帧2地图点 投影到帧1 像素平面上
        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        //步骤4.3： 判断帧2地图点距帧1的距离 是否在尺度协方差范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        // 步骤4.4： 根据深度确定尺度 再根据 尺度确定搜索半径 进而在图像上确定 候选 关键点
        // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);// 尺度

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];// 半径

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(camIdx2,u,v,radius);// 在搜索区域的候选点(同相机下)
//        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        // 步骤4.5：遍历候选关键点  计算与地图点  描述子匹配 计算距离 保留最近距离的匹配
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        // 遍历搜索 帧1区域内的所有特征点，与帧2地图点pMP进行描述子匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            //  关键点的尺度 需要在 预测尺度nPredictedLevel 之上
            const cv::KeyPoint &kp = pKF1->mvKeysUn_total[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;
            // 帧1 关键点描述子
//            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
            const cv::Mat &dKF = pKF1->mDescriptors_total[camIdx2].row(descIdx1);//camidx2表示同相机下计算描述子
            // 帧2 地图点描述子 和 帧1关键点 描述子 距离
            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)//100
        {
            vnMatch2[i2]=bestIdx;// 帧2  地图点 匹配到的 帧1 的关键点(也对应一个地图点
        }
    }

    // Check agreement
    // 步骤5 检查两者的匹配 是否对应起来
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];// 帧1  地图点 匹配到的 帧2 的关键点(也对应一个地图点) 下标

        if(idx2>=0)// 帧1  地图点 匹配到的 帧2 关键点 下标
        {
            int idx1 = vnMatch2[idx2];// 帧2 关键点  对应的 帧1 地图点下标
            if(idx1==i1)// 匹配 相互符合
            {
                vpMatches12[i1] = vpMapPoints2[idx2];// 更新帧1 在帧2 中匹配的 地图点
                nFound++;
            }
        }
    }
    cout<<"ORBmatcher.cc::L2653 SearchBySim3()得到匹配点："<<nFound<<endl;

    return nFound;
}

/**
 * @brief  通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，
 * 同理，确定pKF2的特征点在pKF1中的大致区域
 * 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，
 * 更新vpMatches12（之前使用SearchByBoW进行特征点匹配时会有漏匹配）
 * @param pKF1          关键帧1(当前关键帧)
 * @param pKF2          关键帧2(候选关键帧)
 * @param vpMatches12   两帧原有匹配点  帧1 特征点 匹配到 帧2 的地图点
 * @param s12              帧2->帧1 相似变换 尺度, stereo/rgbd尺度固定为1
 * @param R12             帧2->帧1  欧式变换 旋转矩阵
 * @param t12              帧2->帧1 欧式变换 平移向量
 * @param th       		 搜索半径参数
 * @return                     成功匹配的数量
 */ // 通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，获得更多匹配点
    //用于Closing Looping 线程 通过得到的sim3来搜索
// todo only cam1
int ORBmatcher::SearchBySim3_cam1(KeyFrame *pKF1, KeyFrame * pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12,
                             const float th) //todo 4x3标定矩阵
{

    cout<<"ORBmatcher.cc::L2356 SearchBySim3()"<<endl;
    // 步骤1：变量初始化-----------------------------------------------------
    //相机内参数
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    // 世界坐标系 到 帧1 的欧式变换Rcw tcw
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    // 世界坐标系 到 帧2 的 欧式变换
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    // 相似变换 旋转矩阵 平移向量
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();//帧1->帧2相似变换旋转矩阵 = 帧1->帧2相似变换尺度 * 帧1->帧2欧式变换旋转矩阵
    cv::Mat t21 = -sR21*t12;

    // 帧1地图点数量  关键点数量
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
    const int N1 = vpMapPoints1.size();
    // 帧2地图点数量 关键点数量
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches_cam1();
    const int N2 = vpMapPoints2.size();
    // 来源于两帧先前已有的匹配
    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    // 步骤2：用vpMatches12更新已有的匹配 vbAlreadyMatched1和vbAlreadyMatched2------------------------
    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];// 帧1特征点到帧2的已匹配的地图点
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;// 帧1特征点已经有匹配到的地图点了
            int idx2 = pMP->GetIndexInKeyFrame_cam1(pKF2);// 帧2的地图点在帧2中对应的下标
            if(idx2>=0 && idx2<N2)//在帧2特征点个数范围内的话
                vbAlreadyMatched2[idx2]=true;// 帧2地图点在帧1中也已经有匹配
        }
    }

    // 新寻找的匹配
    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    // 步骤3：通过Sim变换，确定pKF1的地图点在pKF2帧图像中的大致区域，
    //  在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新vpMatches12
    //  （之前使用SearchByBoW进行特征点匹配时会有漏匹配）
    // 每一个帧1中的地图点 投影到 帧2 上
    // 遍历帧1地图点
    for(int i1=0; i1<N1; i1++)//遍历每一个帧1中的地图点
    {
        //步骤3.1： 跳过已有的匹配 和 不存在的点 以及坏点
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

//        int camIdx1 = pKF1->keypoint_to_cam.find(i1)->second; // todo 帧1的点在帧1的相机编号
        //步骤3.2： 帧1地图点(世界坐标系)--> 帧1地图点(帧1坐标系)--> 帧1地图点(帧2坐标系)-->帧2像素坐标
        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;

        // 帧1地图点投影到帧2的相机坐标系 TODO 经过sim3变换后
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;
        // todo 帧2中相机2的点再转换到相机2
//        if (camIdx1 == 1)
//        {
//            p3Dc2 = Rcam21 * p3Dc2 +tcam21;
//        }

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        // 投影到帧2像素坐标
        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        // 坐标必须在 图像平面尺寸内
        if(!pKF2->IsInImage(u,v))
            continue;

        //步骤3.3：  判断帧1地图点距帧2的距离 是否在尺度协方差范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        // 步骤3.4：根据深度确定尺度 再根据尺度确定搜索半径 进而在图像上确定候选关键点
        //根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);// 尺度 也就是在 金字塔哪一层

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];// 再根据尺度确定搜索半径

        // TODO camIdx1表示两帧间的同相机
//        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(camIdx1,u,v,radius);
        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        // 步骤3.5：遍历候选关键点 计算与地图点 描述子匹配 计算距离 保留最近距离的匹配
        // 帧1地图点描述子
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        // 遍历搜索 帧2该区域内的所有特征点，与帧1地图点pMP进行描述子匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;//帧2候选区域内的特征点索引
            //  关键点的尺度 需要在 预测尺度nPredictedLevel 之上

//            const cv::KeyPoint &kp = pKF2->mvKeysUn_total[idx];// 帧2候选区域内的关键点
            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];// 帧2候选区域内的关键点

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            // 帧2 关键点描述子
            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

//            int camIdx = pKF2->keypoint_to_cam.find(idx)->second;//TODO 这里不求帧2中对应特征点的相机索引, 因为已经确定是同相机的闭环
//            int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
//            const cv::Mat &dKF = pKF2->mDescriptors_total[camIdx1].row(descIdx2);//只计算同个相机下的描述子距离?
            // 帧1 地图点描述子 和 帧2关键点 描述子 距离
            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)//100
        {
            vnMatch1[i1]=bestIdx;// 帧1地图点 匹配到的 帧2的关键点(也对应一个地图点)
        }
    }

    // Transform from KF2 to KF2 and search
    // 步骤4：通过Sim变换，确定pKF2的地图点在pKF1帧图像中的大致区域，
    //   在该区域内通过描述子进行匹配捕获pKF2和pKF1之前漏匹配的特征点，更新vpMatches12
    //  （之前使用SearchByBoW进行特征点匹配时会有漏匹配）
    // 每一个帧2中的地图点 投影到 帧1 上
    for(int i2=0; i2<N2; i2++)
    {
        //步骤4.1：跳过已有的匹配 和 不存在的点 以及坏点
        MapPoint* pMP = vpMapPoints2[i2];// 帧2 关键点匹配的 地图点

        if(!pMP || vbAlreadyMatched2[i2])// 不存在匹配的地图点 或者 已经和 帧1匹配了 跳过
            continue;

        if(pMP->isBad())
            continue;

//        int camIdx2 = pKF2->keypoint_to_cam.find(i2)->second; //帧2的点本身所在的相机
        //步骤4.2： 帧2地图点(世界坐标系)--> 帧2地图点(帧2坐标系)--> 帧2地图点(帧1坐标系)-->帧1像素坐标系下
        cv::Mat p3Dw = pMP->GetWorldPos();
        //帧2地图点在帧2的相机坐标
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;

        // 帧2地图点投影到帧1坐标系
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;
        //再投影到帧1各相机坐标
//        if(camIdx2==1)
//        {
//            p3Dc1 = Rcam21 * p3Dc1 +tcam21;
//        }

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        // 帧2地图点 投影到帧1 像素平面上
        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        //步骤4.3： 判断帧2地图点距帧1的距离 是否在尺度协方差范围内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        // 步骤4.4： 根据深度确定尺度 再根据 尺度确定搜索半径 进而在图像上确定 候选 关键点
        // 根据深度预测地图点在帧图像上的尺度  深度大尺度小,深度小尺度大
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);// 尺度

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];// 半径

//        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(camIdx2,u,v,radius);// 在搜索区域的候选点(同相机下)
        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        // 步骤4.5：遍历候选关键点  计算与地图点  描述子匹配 计算距离 保留最近距离的匹配
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        // 遍历搜索 帧1区域内的所有特征点，与帧2地图点pMP进行描述子匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            //  关键点的尺度 需要在 预测尺度nPredictedLevel 之上
//            const cv::KeyPoint &kp = pKF1->mvKeysUn_total[idx];
            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;
            // 帧1 关键点描述子
            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

//            int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx)->second;//特征点的局部索引
//            const cv::Mat &dKF = pKF1->mDescriptors_total[camIdx2].row(descIdx1);//camidx2表示同相机下计算描述子
            // 帧2 地图点描述子 和 帧1关键点 描述子 距离
            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)//100
        {
            vnMatch2[i2]=bestIdx;// 帧2  地图点 匹配到的 帧1 的关键点(也对应一个地图点
        }
    }

    // Check agreement
    // 步骤5 检查两者的匹配 是否对应起来
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];// 帧1  地图点 匹配到的 帧2 的关键点(也对应一个地图点) 下标

        if(idx2>=0)// 帧1  地图点 匹配到的 帧2 关键点 下标
        {
            int idx1 = vnMatch2[idx2];// 帧2 关键点  对应的 帧1 地图点下标
            if(idx1==i1)// 匹配 相互符合
            {
                vpMatches12[i1] = vpMapPoints2[idx2];// 更新帧1 在帧2 中匹配的 地图点
                nFound++;
            }
        }
    }
    cout<<"ORBmatcher.cc::L2653 SearchBySim3()得到匹配点："<<nFound<<endl;

    return nFound;
}

/**
 * @brief 通过投影，对上一帧的特征点(地图点)进行跟踪
 * 运动跟踪模式
 * 上一帧中包含了MapPoints，对这些MapPoints进行跟踪tracking，由此增加当前帧的MapPoints \n
 * 1. 将上一帧的MapPoints投影到当前帧(根据速度模型可以估计当前帧的Tcw)
 * 2. 在投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
 * @param  CurrentFrame 当前帧
 * @param  LastFrame       上一帧
 * @param  th                      搜索半径参数
 * @param  bMono             是否为单目
 * @return                           成功匹配的数量
 * @see SearchByBoW()
 */ // 用于tracking 中的TrackWithMotionModel() NOTE only cam1
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th,
                                   const bool bMono, cv::Mat CalibMatrix)
{
//    cout<<"ORBmatcher.cc::L1927 SearchByProjection"<<endl;
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    // 步骤1：变量初始化----------------------------------------------------------
    // 匹配点 观测方向差 直方图 统计 用来筛选 最好的 匹配
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    //NOTE 标定矩阵 4x3 cam12
    //+++++++++++++++++++++++++++++
    cv::Mat Rcam12= cv::Mat_<float>(3,3);
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
    Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    mRcam21 = Rcam12.t();
    mtcam21 = -mRcam21 * tcam12;
    //++++++++++++++++++++++++++++++

    // 当前帧 旋转 平移矩阵
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat twc = -Rcw.t()*tcw;

    // 上一帧 旋转及平移矩阵
    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw; // 当前帧到上一帧的 平移向量

    // 判断前进还是后退
    // 非单目情况，如果Z>0且大于基线，则表示前进
//    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
//    const bool bForward_cam2 = tlc.at<float>(0)>CurrentFrame.mb && !bMono;
//    // 非单目情况，如果Z<0,且绝对值大于基线，则表示后退
//    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;
//    const bool bBackward_cam2 = -tlc.at<float>(0)>CurrentFrame.mb && !bMono;

    vector<bool> bForward(2);
    bForward[0] = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    bForward[1] = tlc.at<float>(0)>CurrentFrame.mb && !bMono;
    vector<bool> bBackward(2);
    bBackward[0] = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    bBackward[1] = -tlc.at<float>(0)>CurrentFrame.mb && !bMono;

    // 步骤2：遍历上一帧所有的关键点(对应 地图点)------------------------------------------
    for(int i=0; i<LastFrame.N_total; i++)  //todo 相机2
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];//上一帧  地图点

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])// 该地图点也不是外点 是内点 复合变换关系的点
            {
                // Project
        // 步骤3： 上一帧的地图点投影到当前帧像素平面上-----------------------------------------
                int cam = LastFrame.keypoint_to_cam.find(i)->second;
                cv::Mat x3Dw = pMP->GetWorldPos();// 上一帧地图点（世界坐标系下）
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//上一帧地图点（当前帧相机坐标系下）
                if(cam==1){
                    x3Dc = mRcam21 * x3Dc + mtcam21;
                }

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);// 深度>0 逆深度>0

                if(invzc<0)
                    continue;

                // 上一帧地图点投影到当前帧像素坐标
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;// 需要在 图像尺寸内
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

        // 步骤4： 在当前帧上确定候选点 (上一帧地图点投影到当前帧像素坐标)----------------------------------------------
                // NOTE 尺度越大,图像越小 (点越近,使用的尺度越大(降采样越多),用来保持近点和远点的尺度相对一致)
                // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                // 当前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                // 因此m>=n，对应前进的情况，nCurOctave>=nLastOctave。后退的情况可以类推
                int nLastOctave = LastFrame.mvKeys_total[i].octave;//上一帧地图点对应特征点所处的 尺度(金字塔层数)

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];//尺度越大，搜索范围越大

                vector<size_t> vIndices2;// 当前帧 上 投影点附近的 候选点

                if(bForward[cam])// 前进,则上一帧兴趣点在所在的尺度nLastOctave <= nCurOctave< 8(更近了 尺度大 层数高(图更小)也可以看见
                    vIndices2 = CurrentFrame.GetFeaturesInArea(cam, u,v, radius, nLastOctave);
                else if(bBackward[cam])// 后退,则上一帧兴趣点在所在的尺度 0<= nCurOctave <= nLastOctave（远了 尺度降低）
                    vIndices2 = CurrentFrame.GetFeaturesInArea(cam, u,v, radius, 0, nLastOctave);
                else// 没怎么运动 在上一帧 尺度附加搜索
                    vIndices2 = CurrentFrame.GetFeaturesInArea(cam, u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

        // 步骤5：遍历候选关键点  计算与地图点  描述子匹配 计算距离 保留最近距离的匹配
                const cv::Mat dMP = pMP->GetDescriptor();// 上一帧地图点描述子

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])// 如果当前帧关键帧有地图点
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)//该对应地图点也有观测帧 则跳过
                            continue;//跳过不用再匹配地图点

                    // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                    if(CurrentFrame.mvuRight_total[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;//匹配点 右图的横坐标
                        const float er = fabs(ur - CurrentFrame.mvuRight_total[i2]);// 误差
                        if(er>radius)
                            continue;
                    }

                    //在各相机下的局部编号
                    int descIdx = CurrentFrame.cont_idx_to_local_cam_idx.find(i2)->second;
                    // 取出F中该特征对应的描述子,用的局部相机索引
                    const cv::Mat &d= CurrentFrame.mDescriptors_total[cam].row(descIdx);

//                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);// 当前帧 关键点描述子

                    const int dist = DescriptorDistance(dMP,d);// 描述子匹配距离

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)//100
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;// 为当前帧关键点匹配上一帧的地图点
                    nmatches++;

                    // 匹配点 观测方向差 一致性检测
                    if(mbCheckOrientation)
                    {
                        // 上一帧 地图点的观测方向   -  当前帧 特征点 的观测方向
                        float rot = LastFrame.mvKeysUn_total[i].angle-CurrentFrame.mvKeysUn_total[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);//统计到对应的 方向直方图上
                    }
                }
            }
        }
    }

    //Apply rotation consistency
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

// 用于tracking 中的TrackWithMotionModel()
//int ORBmatcher::SearchByProjection_cam1(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
//{
//    cout<<"ORBmatcher.cc::L1927 SearchByProjection"<<endl;
//    int nmatches = 0;
//
//    // Rotation Histogram (to check rotation consistency)
//    // 步骤1：变量初始化----------------------------------------------------------
//    // 匹配点 观测方向差 直方图 统计 用来筛选 最好的 匹配
//    vector<int> rotHist[HISTO_LENGTH];
//    for(int i=0;i<HISTO_LENGTH;i++)
//        rotHist[i].reserve(500);
//    const float factor = 1.0f/HISTO_LENGTH;
//
//    // 当前帧 旋转 平移矩阵
//    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
//    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
//
//    const cv::Mat twc = -Rcw.t()*tcw;//当前帧到上一帧的 平移向量
//
//    // 上一帧 旋转及平移矩阵
//    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
//    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);
//
//    const cv::Mat tlc = Rlw*twc+tlw;
//
//    // 判断前进还是后退
//    // 非单目情况，如果Z>0且大于基线，则表示前进
//    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
//    // 非单目情况，如果Z<0,且绝对值大于基线，则表示前进
//    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;
//
//    // 步骤2：遍历上一帧所有的关键点(对应 地图点)------------------------------------------
//    for(int i=0; i<LastFrame.N; i++)
//    {
//        MapPoint* pMP = LastFrame.mvpMapPoints[i];//上一帧  地图点
//
//        if(pMP)
//        {
//            if(!LastFrame.mvbOutlier[i])// 该地图点也不是外点 是内点 复合变换关系的点
//            {
//                // Project
//                // 步骤3： 上一帧的地图点投影到当前帧像素平面上-----------------------------------------
//                cv::Mat x3Dw = pMP->GetWorldPos();// 上一帧地图点（世界坐标系下）
//                cv::Mat x3Dc = Rcw*x3Dw+tcw;//上一帧地图点（当前帧相机坐标系下）
//
////                int cam = LastFrame.keypoint_to_cam.find(i)->second;
//                const float xc = x3Dc.at<float>(0);
//                const float yc = x3Dc.at<float>(1);
//                const float invzc = 1.0/x3Dc.at<float>(2);// 深度>0 逆深度>0
//
//                if(invzc<0)
//                    continue;
//
//                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;//像素坐标
//                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;
//
//                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
//                    continue;// 需要在 图像尺寸内
//                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
//                    continue;
//
//                // 步骤4： 在当前帧上确定候选点-----------------------------------------------------
//                // NOTE 尺度越大,图像越小
//                // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
//                // 当前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
//                // 因此m>=n，对应前进的情况，nCurOctave>=nLastOctave。后退的情况可以类推
//                int nLastOctave = LastFrame.mvKeys[i].octave;//上一帧地图点对应特征点所处的 尺度(金字塔层数)
//
//                // Search in a window. Size depends on scale
//                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];//尺度越大，搜索范围越大
//
//                vector<size_t> vIndices2;// 当前帧 上 投影点附近的 候选点
//
//                if(bForward)// 前进,则上一帧兴趣点在所在的尺度nLastOctave <= nCurOctave< 8(更近了 尺度大 层数高也可以看见
//                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
//                else if(bBackward)// 后退,则上一帧兴趣点在所在的尺度 0<= nCurOctave <= nLastOctave（远了 尺度降低）
//                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
//                else// 没怎么运动 在上一帧 尺度附加搜索
//                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);
//
//                if(vIndices2.empty())
//                    continue;
//
//                // 步骤5：遍历候选关键点  计算与地图点  描述子匹配 计算距离 保留最近距离的匹配
//                const cv::Mat dMP = pMP->GetDescriptor();// 上一帧地图点描述子
//
//                int bestDist = 256;
//                int bestIdx2 = -1;
//
//                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
//                {
//                    const size_t i2 = *vit;
//                    if(CurrentFrame.mvpMapPoints[i2])// 如果当前帧关键帧有地图点
//                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)//该对应地图点也有观测帧 则跳过
//                            continue;//跳过不用再匹配地图点
//
//                    // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
//                    if(CurrentFrame.mvuRight[i2]>0)
//                    {
//                        const float ur = u - CurrentFrame.mbf*invzc;//匹配点 右图的横坐标
//                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);// 误差
//                        if(er>radius)
//                            continue;
//                    }
//
//                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);// 当前帧 关键点描述子
//
//                    const int dist = DescriptorDistance(dMP,d);// 描述子匹配距离
//
//                    if(dist<bestDist)
//                    {
//                        bestDist=dist;
//                        bestIdx2=i2;
//                    }
//                }
//
//                if(bestDist<=TH_HIGH)//100
//                {
//                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;// 为当前帧关键点匹配上一帧的地图点
//                    nmatches++;
//
//                    // 匹配点 观测方向差 一致性检测
//                    if(mbCheckOrientation)
//                    {
//                        // 上一帧 地图点的观测方向   -  当前帧 特征点 的观测方向
//                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
//                        if(rot<0.0)
//                            rot+=360.0f;
//                        int bin = round(rot*factor);
//                        if(bin==HISTO_LENGTH)
//                            bin=0;
//                        assert(bin>=0 && bin<HISTO_LENGTH);
//                        rotHist[bin].push_back(bestIdx2);//统计到对应的 方向直方图上
//                    }
//                }
//            }
//        }
//    }
//
//    //Apply rotation consistency
//    if(mbCheckOrientation)
//    {
//        int ind1=-1;
//        int ind2=-1;
//        int ind3=-1;
//
//        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
//
//        for(int i=0; i<HISTO_LENGTH; i++)
//        {
//            if(i!=ind1 && i!=ind2 && i!=ind3)
//            {
//                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
//                {
//                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
//                    nmatches--;
//                }
//            }
//        }
//    }
//
//    return nmatches;
//}

//tracking中的重定位
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t()*tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches_cam1(); //todo
//    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Compute predicted scale level
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}
/**
 * @brief  统计直方图最高的三个bin保留，其他范围内的匹配点剔除。
		 另外，若最高的比第二高的高10倍以上，则只保留最高的bin中的匹配点。
		 若最高的比第 三高的高10倍以上，则 保留最高的和第二高bin中的匹配点。
 * @param  histo  直方图
 * @param  L         直方图的大小
 * @param  ind1   数量最高的一个bin
 * @param  ind2   数量次高的一个bin
 * @param  ind3   数量第三高的一个bin
 */
void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}
//反对称矩阵
cv::Mat ORBmatcher::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<  0,        -v.at<float>(2),   v.at<float>(1),
                              v.at<float>(2),          0,        -v.at<float>(0),
                             -v.at<float>(1),  v.at<float>(0),             0);
}
//
//// //先放着
//cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
//{
//    // Essential Matrix: t12叉乘R12
//    // Fundamental Matrix: inv(K1)*E*inv(K2)
//    cv::Mat R1w = pKF1->GetRotation();
//    cv::Mat t1w = pKF1->GetTranslation();
//    cv::Mat R2w = pKF2->GetRotation();
//    cv::Mat t2w = pKF2->GetTranslation();
//
//    cv::Mat R12 = R1w*R2w.t();
//    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
//
//    cv::Mat t12x = SkewSymmetricMatrix(t12);
//
//    const cv::Mat &K1 = pKF1->mK;
//    const cv::Mat &K2 = pKF2->mK;
//
//
//    return K1.t().inv()*t12x*R12*K2.inv();
//}
} //namespace ORB_SLAM
