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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular, const cv::Mat CalibMatrix):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mCalibMatrix(CalibMatrix.clone())
{
    //+++++++++++++++++++++++++++++++++++++++++++++++++++
    mRcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);
    mtcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    mtcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    mtcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    mtcam21 = -mRcam12.inv() * mtcam12;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++=
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
//    cout<<"LocalMapping.cc::L49 Run() 局部建图..."<<endl;
    // note 计算局部建图时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        // 步骤1：设置进程间的访问标志 告诉Tracking线程，LocalMapping线程正在处理新的关键帧，处于繁忙状态
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        // 等待处理的关键帧列表不为空
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            // 步骤2：计算关键帧特征点的词典单词向量BoW映射，将关键帧插入地图
            ProcessNewKeyFrame();
//            cout << "LocalMapping.cc::L74: 处理新关键帧后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

            // Check recent MapPoints
            // 步骤3：对新添加的地图点融合 对于ProcessNewKeyFrame 和CreateNewMapPoints中最近添加的MapPoints进行检查剔除
//            MapPointCulling();//移到步骤4之后?

            // Triangulate new MapPoints
            // 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
            // 步骤4： 创建新的地图点 相机运动过程中与相邻关键帧通过三角化恢复出一些新的地图点MapPoints
            CreateNewMapPoints();
//            cout << "LocalMapping.cc::L83: CreateNewMapPoints()后地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

            /////////////////////
            MapPointCulling();//从前面移到步骤4之后?//////////////////////////////////////////
            //////////////////////
//            cout << "LocalMapping.cc::L88: MapPointCulling()后地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;

            // 已经处理完队列中的最后的一个关键帧
            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                // 步骤5：相邻帧地图点融合 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
                SearchInNeighbors();//+++
//                cout << "LocalMapping.cc::L96: 相邻帧融合后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
            }

            mbAbortBA = false;

            // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                // 步骤6：局部地图优化 Local BA
                if(mpMap->KeyFramesInMap()>2)//
                {//todo 优化有问题
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);
//                    cout << "LocalMapping.cc::L109: 局部BA优化后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
                }
                // Check redundant local Keyframes
                // 步骤7： 关键帧融合 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                // 剔除的标准是：该关键帧的90%的MapPoints可以被其它关键帧观测到
                // Tracking中先把关键帧交给LocalMapping线程
                // 并且在Tracking中InsertKeyFrame函数的条件比较松，交给LocalMapping线程的关键帧会比较密
                // 在这里再删除冗余的关键帧
                KeyFrameCulling();
//                cout << "LocalMapping.cc::L118: 剔除冗余关键帧后现在地图中总共有 <" << mpMap->MapPointsInMap() << "> 个地图点" << endl;
            }

            // 步骤8：将当前帧加入到闭环检测队列中
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        // 步骤9：等待线程空闲 完成一帧关键帧的插入融合工作
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())//检查 是否完成
                break;
        }

        // 检查重置
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        // 步骤10：告诉Tracking线程 Local Mapping线程空闲,可一处理接收下一个关键帧
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }
    // note 计算建图时间
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tlba= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"局部建图成功，花费时间为： "<<tlba<<endl;

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)//Tracking中插入关键帧,初始化时一次,其他在CreateNewKeyFrame()插入
{
//    cout << "LocalMapping.cc::L140: InsertKeyFrame()..." << endl;
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;// BA优化停止
//    cout << "LocalMapping.cc::L140: InsertKeyFrame()完成" << endl;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();//得到BowVec,mFeatVec,BowVec_cam2,mFeatVec_cam2

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);//mObservations,mObservations_cam2
                    pMP->UpdateNormalAndDepth();
                    // 加入关键帧后，更新3d点的最佳描述子
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 将双目或RGBD跟踪过程中新插入的MapPoints放入mlpRecentAddedMapPoints，等待检查
                    // CreateNewMapPoints函数中通过三角化也会生成MapPoints
                    // 这些MapPoints都会经过MapPointCulling函数的检验
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}
/**
 * @brief 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入的质量不好的MapPoints
 * @see VI-B recent map points culling
 */
 // 剔除最近添加的质量不好的地图点?
void LocalMapping::MapPointCulling()
{
//    cout<<"LocalMapping.cc::L198: MapPointCulling()..."<<endl;
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    // 遍历等待检查的MapPoints
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            // 步骤1：已经是坏点的MapPoints直接从检查链表中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            // 步骤2：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件1：
            // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
            // IncreaseFound / IncreaseVisible < 25%，注意不一定是关键帧。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // 步骤3：将不满足VI-B条件的MapPoint剔除
            // VI-B 条件2：从该点建立开始，到现在已经过了不小于2帧，
            // 但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit); // 步骤4：从建立该点开始，已经过了3帧，放弃对该MapPoint的检测
        else
            lit++;
    }
}
// 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints
void LocalMapping::CreateNewMapPoints()
{
//    cout<<"LocalMapping.cc::L242 CreateNewMapPoints(): 三角化出新点..."<<endl;
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 15; //todo 原来是10
    if(mbMonocular)
        nn=20;
    // 步骤1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻帧vpNeighKFs(已排序)
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);//不同相机间的共视怎么办？
//    const vector<KeyFrame*> vpNeighKFs_cam2 = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames_cam2(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat Rcam21 = mRcam12.inv();
    cv::Mat Rcw1_cam2 = mpCurrentKeyFrame->GetRotation_cam2();
//    cv::Mat Rcw1_cam2 = Rcam21*Rcw1;//inv 矩阵的逆
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat tcw1_cam2 = mpCurrentKeyFrame->GetTranslation_cam2(); //tc2w
    cv::Mat Tcw1(3,4,CV_32F);
    cv::Mat Tcw1_cam2 (3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    Rcw1_cam2.copyTo(Tcw1_cam2.colRange(0,3));//0列开始共3列
    tcw1_cam2.copyTo(Tcw1_cam2.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    cv::Mat Ow1_cam2 = mpCurrentKeyFrame->GetCameraCenter_cam2(); //twc2

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    // 步骤2：遍历相邻关键帧vpNeighKFs
//    cout<<"LocalMapping.cc::L297 遍历相邻关键帧, 大小为:"<<vpNeighKFs.size()<<endl;
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
//        cout<<"LocalMapping.cc::L298 正在遍历第 <"<<i<<"> 个关键帧"<<endl;
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat Ow2_cam2 = pKF2->GetCameraCenter_cam2();
        cv::Mat vBaseline = Ow2-Ow1; // todo 基线:相机1 2之间的相机坐标怎么计算?
        cv::Mat vBaseline_cam2 = Ow2_cam2-Ow1_cam2; // todo 基线:相机1 2之间的相机坐标怎么计算?
        const float baseline = cv::norm(vBaseline); //长度?
        const float baseline_cam2 = cv::norm(vBaseline_cam2); //长度?

        vector<bool> istrian(2, false); //todo 两个相机分别是否要做三角化,分别判断两相机基线长度

        // 步骤3：判断相机运动的基线是不是足够长
        if(!mbMonocular) //双目or RGBD
        {
//            if(baseline<pKF2->mb)//mb: Stereo baseline in meters. //改成两相机分别判断基线
//                continue;
            if(baseline < pKF2->mb && baseline_cam2 < pKF2->mb)
                continue;
            if(baseline >= pKF2->mb) istrian[0]= true;
            if(baseline_cam2 >= pKF2->mb) istrian[1]= true;
        }
        else //单目
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);//地图点深度中位数
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }
        if (!istrian[0] && !istrian[1]) continue;

        // Compute Fundamental Matrix
        // 步骤4：根据两个关键帧的位姿计算它们之间的基础矩阵; F仅和R t K(内参矩阵)有关
        //垂直的双相机系统同个相机之间的F12应该相同吧 ??
//        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);// 基础矩阵的计算放入下面的SearchForTrianulation()
        cv::Mat F12;
        // Search matches that fullfil epipolar constraint
        // 步骤5：通过极线约束限制匹配时的搜索范围，进行特征点匹配
        vector<pair<size_t,size_t> > vMatchedIndices;//<idx1,idx2>匹配对,这里idx1不一定连续.
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false, istrian);//

        cv::Mat Rcw2 = pKF2->GetRotation();//pKF2是当前关键帧的相邻关键帧
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));
        //cam2
        cv::Mat Rcw2_cam2 = pKF2->GetRotation_cam2();
        cv::Mat Rwc2_cam2 = Rcw2_cam2.t();
        cv::Mat tcw2_cam2 = pKF2->GetTranslation_cam2();
        cv::Mat Tcw2_cam2(3,4,CV_32F);
        Rcw2_cam2.copyTo(Tcw2_cam2.colRange(0,3));
        tcw2_cam2.copyTo(Tcw2_cam2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        // 步骤6：对每对匹配通过三角化生成3D点,和 Initializer.cpp的Triangulate函数差不多
        const int nmatches = vMatchedIndices.size();
//        cout<<"LocalMapping.cc::L351 共有<"<<nmatches<<">对匹配点, 对它们进行三角化..."<<endl;
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // 步骤6.1：取出匹配特征点

            // 当前匹配对在当前关键帧中的索引  //idx1: 0~N_total
            const int &idx1 = vMatchedIndices[ikp].first;
            // 当前匹配对在邻接关键帧中的索引
            const int &idx2 = vMatchedIndices[ikp].second;

            // 当前匹配在当前关键帧中的特征点
            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn_total[idx1];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight_total[idx1];
            bool bStereo1 = kp1_ur>=0;

            // 当前匹配在邻接关键帧中的特征点
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[idx2];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp2_ur = pKF2->mvuRight_total[idx2];
            bool bStereo2 = kp2_ur>=0;

            //<size_t, int> <该特征点在该帧的总编号，所在的相机编号>
            int camIdx1 = mpCurrentKeyFrame->keypoint_to_cam.find(idx1)->second;//kf1中得到特征点在其所在的相机编号
            //<size_t, int>为<点在多相机帧系统的总编号，在各相机下分别的编号>
            int descIdx1 = mpCurrentKeyFrame->cont_idx_to_local_cam_idx.find(idx1)->second;//得到点在相机下的编号

            if(istrian[camIdx1]==false) continue; //跳过不参与三角化的点(基线过短)

            //不用求因为是同个相机
//            int camIdx2 = pKF2->keypoint_to_cam.find(idx2)->second;
//            int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;

            // Check parallax between rays
            // 步骤6.2：利用匹配点反投影得到视差角
            // 特征点反投影;↓这里xn=[X/Z,Y/Z,1].(P86);其中XYZ是点的相机坐标系中坐标
            // xn是归一化坐标?
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            // 由相机坐标系转到世界坐标系，得到视差角余弦值(视差角应该是三维点到两帧的相机原点的夹角吧?)
            cv::Mat ray1 = Rwc1*xn1;//这里是把两个关键帧上的特征点射线起点都放在世界坐标系下以得到夹角
            cv::Mat ray2 = Rwc2*xn2;//笔记见书本P43
//            vector<cv::Mat> ray1(2);
//            vector<cv::Mat> ray2(2);
//            cv::Vec3f rays1 = mpCurrentKeyFrame->mvKeysRays[idx1];//plc TODO mvkeysrays只是第一帧初始化的坐标?
//            cv::Vec3f rays2 = pKF2->mvKeysRays[idx2];//

            //视差??　dot():点乘; norm范数:这里应该是求元素的平方和
            //向量夹角的余弦值=a.b/(||a||*||b||)
//            const float cosParallaxRays = ray1[camIdx1].dot(ray2[camIdx1])/(cv::norm(ray1[camIdx1])*cv::norm(ray2[camIdx1]));
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            // 步骤6.3：对于双目，利用双目得到视差角（点在双目相机的两镜头间的夹角?）
            if(bStereo1)//双目，且有深度
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth_total[idx1]));
            else if(bStereo2)//双目，且有深度
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth_total[idx2]));

            // 得到双目观测的视差角
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

//            cout<<"LocalMapping.cc::L403 正在对第<"<<ikp<<">对点进行三角化"<<endl;
            // 步骤6.4：三角化恢复3D点
            cv::Mat x3D;
            // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视差角正常
            // cosParallaxRays<cosParallaxStereo表明视差角很小
            // 视差角度小时用三角法恢复3D点，视差角大时用双目恢复3D点（双目以及深度有效）
            // TODO 三角化得到3D点
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                // 见Initializer.cpp的Triangulate函数
                //     |p1 p2  p3  p4 |   |--p0--|
                //     |p5 p6  p7  p8 |==>|--p1--| 相机投影矩阵
                //     |p9 p10 p11 p12|   |--p2--|
                // |xp2  - p0 |
                // |yp2  - p1 | ===> A
                // |x'p2'- p0'|
                // |y'p2'- p1'|
                cv::Mat A(4,4,CV_32F);//这里xn=[X/Z,Y/Z,1];其中XYZ是点的相机坐标系中坐标?
                if(camIdx1==0){
                    A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);
                }
                else if (camIdx1==1){
                    A.row(0) = xn1.at<float>(0)*Tcw1_cam2.row(2)-Tcw1_cam2.row(0);
                    A.row(1) = xn1.at<float>(1)*Tcw1_cam2.row(2)-Tcw1_cam2.row(1);
                    A.row(2) = xn2.at<float>(0)*Tcw2_cam2.row(2)-Tcw2_cam2.row(0);
                    A.row(3) = xn2.at<float>(1)*Tcw2_cam2.row(2)-Tcw2_cam2.row(1);
                }
                //SVD分解
                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);// FULL_UV表示把U和VT补充称单位正交方阵;

                x3D = vt.row(3).t();//转置成纵向

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates//齐次点坐标 除去尺度
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            //  步骤6.4：对于双目 视差角较小时 二维点 利用深度值 反投影 成 三维点
            //对于双目,在当前帧和共视帧中选视差角大的那帧？
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
            }
            //增加三角化后下面4行删掉
//            if(bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)//对于双目,在当前帧和共视帧中选视差角大的那帧？
//            {
//                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
//            }
                //对于双目,在当前帧和共视帧中选视差角大的那帧？
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else// 单目 视差角 较小时 生成不了三维点
                continue; //No stereo and very low parallax
                //没有双目/深度,且两帧视角差太小, 三角测量也不合适 得不到三维点

            cv::Mat x3Dt = x3D.t();//转置成横向(行向量)

//            cout<<"LocalMapping.cc::L460..."<<endl;
            // 步骤6.5：检测生成的3D点是否在相机前方//下式等于求x3D的式子的逆运算
            //Check triangulation in front of cameras
            float z1,z2;
            if(camIdx1==0)
            {
                z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);//dot() 两向量点乘
                if(z1<=0)
                    continue;
                z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                if(z2<=0)
                    continue;
            }
            else if(camIdx1==1)
            {
                //相机2下生成的点
                z1 = Rcw1_cam2.row(2).dot(x3Dt)+tcw1_cam2.at<float>(2);//dot() 两向量点乘
//                z1 = (Rcam21.row(2)*Rcw1).dot(x3Dt)+Rcam21.row(2).dot(tcw1) + mtcam21.at<float>(2,0);
                if(z1<=0)
                    continue;
//                z2 = (Rcam21.row(2)*Rcw2).dot(x3Dt)+Rcam21.row(2).dot(tcw2) + mtcam21.at<float>(2,0);
                z2 = Rcw2_cam2.row(2).dot(x3Dt)+tcw2_cam2.at<float>(2);
                if(z2<=0)
                    continue;
            }

//            cout<<"LocalMapping.cc::L493..."<<endl;
            //Check reprojection error in first keyframe
            // 步骤6.6：计算3D点在当前关键帧下的重投影误差, 即投影到相机平面?   //误差 分布参数
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
//            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);//相机(坐标系)归一化坐标
//            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            float x1;
            float y1;

            if(camIdx1==0)
            {
                x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);//相机(坐标系)归一化坐标//.dot()点乘
                y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            }
            else if(camIdx1==1)
            {   //相机2
                x1 = Rcw1_cam2.row(0).dot(x3Dt)+tcw1_cam2.at<float>(0);//相机(坐标系)归一化坐标//.dot()点乘
                y1 = Rcw1_cam2.row(1).dot(x3Dt)+tcw1_cam2.at<float>(1);
//                x1 = (Rcam21.row(0)*Rcw1).dot(x3Dt)+Rcam21.row(0).dot(tcw1) + mtcam21.at<float>(0,0);//相机(坐标系)归一化坐标
//                y1 = (Rcam21.row(1)*Rcw1).dot(x3Dt)+Rcam21.row(1).dot(tcw1) + mtcam21.at<float>(1,0);
//                P3D1c = Rcam21*Rcw1*x3D + Rcam21*tcw1 +mtcam21;
            }
            const float invz1 = 1.0/z1;

            if(!bStereo1)// 单目
            {
                float u1 = fx1*x1*invz1+cx1;//像素坐标
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;//投影误差过大 跳过
            }
            else// 双目,深度相机   有右图像匹配点横坐标差值
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;//左图像坐标值 - 视差 = 右图像匹配点横坐标
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                // 基于卡方检验计算出的阈值（假设测量有一个一个像素的偏差）
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

//            cout<<"LocalMapping.cc::L540..."<<endl;
            //Check reprojection error in second keyframe
            // 计算3D点在另一个关键帧下的重投影误差
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
//            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
//            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            float x2;
            float y2;
            if(camIdx1==0)
            {
                x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);//相机(坐标系)归一化坐标
                y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            }
            else if(camIdx1==1)
            {   //相机2
                x2 = Rcw2_cam2.row(0).dot(x3Dt)+tcw2_cam2.at<float>(0);//相机(坐标系)归一化坐标
                y2 = Rcw2_cam2.row(1).dot(x3Dt)+tcw2_cam2.at<float>(1);
//                x2 = (Rcam21.row(0)*Rcw2).dot(x3Dt)+Rcam21.row(0).dot(tcw2)+ mtcam21.at<float>(0,0);//相机(坐标系)归一化坐标
//                y2 = (Rcam21.row(1)*Rcw2).dot(x3Dt)+Rcam21.row(1).dot(tcw2)+ mtcam21.at<float>(1,0);
            }
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                // 基于卡方检验计算出的阈值（假设测量有一个一个像素的偏差）
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            // 步骤6.7：检查尺度连续性
//            cout<<"LocalMapping.cc::L584..."<<endl;
            // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
//            cv::Mat normal1 = x3D-Ow1;
            cv::Mat normal1;
            cv::Mat normal2;

            if(camIdx1==0) {
                normal1 = x3D-Ow1;
                normal2 = x3D-Ow2;
            }
            else if (camIdx1==1) {
                normal1 = x3D-Ow1_cam2;
                normal2 = x3D-Ow2_cam2;
            }
            float dist1 = cv::norm(normal1);

//            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            // ratioDist是不考虑金字塔尺度下的距离比例
            const float ratioDist = dist2/dist1;
            // 金字塔尺度因子的比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            // ratioDist*ratioFactor < ratioOctave 或 ratioDist/ratioOctave > ratioFactor表明尺度变化是连续的
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            // 步骤6.8：三角化生成3D点成功，构造成MapPoint
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            // 步骤6.9：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围
            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            // 步骤6.8：将新产生的点放入检测队列
            // 这些MapPoints都会经过MapPointCulling函数的检验
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

// 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
void LocalMapping::SearchInNeighbors()
{
//    cout<<"LocalMapping.cc::L606 SearchInNeighbors(): 检查并融合重复点..."<<endl;
    // Retrieve neighbor keyframes
    // 步骤1：获得当前关键帧在covisibility图中权重排名前nn的一级邻接关键帧
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;// 之后要添加的最后合格的一级二级相邻关键帧
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        // 步骤2：获得当前关键帧在 其一级相邻帧的  covisibility图中权重排名前5的二级邻接关键帧
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;// 二级相邻关键帧是坏帧,在一级时已经加入 或者 又找回来了找到当前帧了 跳过
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    // 步骤3：将当前帧的MapPoints 分别与 其一级二级相邻帧的MapPoints 进行融合
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

//        cout<<"LocalMapping.cc::L668 Fuse(): 将当前帧的地图点投影到相邻关键帧中..."<<endl;
        // 将当前帧的MapPoints投影到相邻关键帧pKFi中，并判断是否有重复的MapPoints
        matcher.Fuse(pKFi,vpMapPointMatches,mCalibMatrix);//返回重复MapPoints的数量
    }

    // Search matches by projection from target KFs in current KF
    // 步骤4：将一级二级相邻帧的MapPoints分别与当前帧（的MapPoints）进行融合
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    // 遍历邻接关键帧
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;//每一个相邻关键帧

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();//获得该相邻关键帧的所有地图点

        // 遍历邻接关键帧中所有的MapPoints
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;//一个MapPoint
            if(!pMP)
                continue;
            // 判断MapPoints是否为坏点，或者是否已经加进集合vpFuseCandidates
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);//todo 添加地图点在该帧的索引?
        }
    }

//    cout<<"LocalMapping.cc::L699 Fuse(): 将相邻帧的地图点投影到当前帧中..."<<endl;
// 把相邻帧的MapPoints投影到当前帧上，在附加区域搜索匹配关键点，并判断是否有重复的地图点
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,mCalibMatrix); // 把相邻帧地图点投影到当前帧上


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

//根据两关键帧的位姿计算基本矩阵
cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    // Essential Matrix 本质矩阵: t12叉乘R12
    // Fundamental Matrix 基础及裤子: inv(K1)*E*inv(K2)
    //Rcw1  tcw1
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

//    cv::Mat R1w_cam2 = pKF1->GetPose_cam2();
//    cv::Mat t1w_cam2 = pKF1->GetTranslation_cam2();
//    cv::Mat Rw1=R1w.inv();
//    cv::Mat tw1_cam2 = Rw1*mtcam12 - Rw1*t1w; // twc2 = Rwc1 * tcam12 + twc1 ; 其中twc1 = -Rwc1 * tc1w;
//    cv::Mat R1w_cam2 = mRcam12.inv() * R1w;  // Rc2w = Rcam12.inv * Rc1w ;
//    cv::Mat t1w_cam2 = -R1w_cam2 * tw1_cam2;

    //Rcw2  tcw2
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

//    cv::Mat R2w_cam2 = pKF2->GetRotation_cam2();
//    cv::Mat t2w_cam2 = pKF2->GetTranslation_cam2();
//    cv::Mat Rw2=R2w.inv();
//    cv::Mat tw2_cam2 = Rw2*mtcam12 - Rw2*t2w; // twc2 = Rwc1 * tcam12 + twc1 ; 其中twc1 = -Rwc1 * tc1w;
//    cv::Mat R2w_cam2 = mRcam12.inv() * R2w;  // Rc2w = Rcam12.inv * Rc1w ;
//    cv::Mat t2w_cam2 = -R2w_cam2 * tw2_cam2;

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

//    vector<cv::Mat> R

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

//根据两关键帧的位姿计算基本矩阵
//vector<cv::Mat> LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
//{
//    // Essential Matrix 本质矩阵: t12叉乘R12
//    // Fundamental Matrix 基础及裤子: inv(K1)*E*inv(K2)
//    //Rcw1  tcw1
//    cv::Mat R1w = pKF1->GetRotation();
//    cv::Mat t1w = pKF1->GetTranslation();
//
//    cv::Mat R1w_cam2 = pKF1->GetPose_cam2();
//    cv::Mat t1w_cam2 = pKF1->GetTranslation_cam2();
////    cv::Mat Rw1=R1w.inv();
////    cv::Mat tw1_cam2 = Rw1*mtcam12 - Rw1*t1w; // twc2 = Rwc1 * tcam12 + twc1 ; 其中twc1 = -Rwc1 * tc1w;
////    cv::Mat R1w_cam2 = mRcam12.inv() * R1w;  // Rc2w = Rcam12.inv * Rc1w ;
////    cv::Mat t1w_cam2 = -R1w_cam2 * tw1_cam2;
//
//    //Rcw2  tcw2
//    cv::Mat R2w = pKF2->GetRotation();
//    cv::Mat t2w = pKF2->GetTranslation();
//
//    cv::Mat R2w_cam2 = pKF2->GetRotation_cam2();
//    cv::Mat t2w_cam2 = pKF2->GetTranslation_cam2();
////    cv::Mat Rw2=R2w.inv();
////    cv::Mat tw2_cam2 = Rw2*mtcam12 - Rw2*t2w; // twc2 = Rwc1 * tcam12 + twc1 ; 其中twc1 = -Rwc1 * tc1w;
////    cv::Mat R2w_cam2 = mRcam12.inv() * R2w;  // Rc2w = Rcam12.inv * Rc1w ;
////    cv::Mat t2w_cam2 = -R2w_cam2 * tw2_cam2;
//
//    cv::Mat R12 = R1w*R2w.t();
//    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
//
//    vector<vector<cv::Mat>> R12s(2);
//    vector<vector<cv::Mat>> t12s(2);
//
//    R12s[0][0] = R1w*R2w.t();
//
//    //反对称矩阵
//    cv::Mat t12x = SkewSymmetricMatrix(t12);
//
//    const cv::Mat &K1 = pKF1->mK;
//    const cv::Mat &K2 = pKF2->mK;
//
//
//    return K1.t().inv()*t12x*R12*K2.inv();
//}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

/**
 * @brief    关键帧剔除
 *  在Covisibility Graph 关键帧连接图 中的关键帧，
 *  其90%以上的地图点MapPoints能被其他关键帧（至少3个）观测到，
 *  则认为该关键帧为冗余关键帧。
 * @param  pKF1 关键帧1
 * @param  pKF2 关键帧2
 * @return 两个关键帧之间的基本矩阵 F
 */
void LocalMapping::KeyFrameCulling()
{
//    cout<<"LocalMapping.cc::L806 KeyFrameCulling(): 剔除冗余关键帧..."<<endl;
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    // 步骤1：根据Covisibility Graph 关键帧连接图提取当前帧的所有共视关键帧(关联帧)
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 遍历所有的当前帧的局部关键帧
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)//第一帧关键帧为 初始化世界关键帧 跳过
            continue;
        // 步骤2：提取每个共视关键帧的地图点 MapPoints
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        // 步骤3：遍历该局部关键帧的MapPoints，判断是否90%以上的MapPoints能被其它关键帧（至少3个）观测到
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {   // 对于双目，仅考虑近处的MapPoints，不超过mbf*35 / fx
                        if(pKF->mvDepth_total[i] > pKF->mThDepth || pKF->mvDepth_total[i]<0)
                            continue;
                    }

                    nMPs++;
                    // 地图点 MapPoints 至少被三个关键帧观测到
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn_total[i].octave;// 金字塔层数
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();// 局部 观测关键帧地图
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)// 跳过 原地图点的帧
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn_total[mit->second].octave;// 金字塔层数

                            // 尺度约束，要求MapPoint在该局部关键帧的特征尺度大于（或近似于）其它关键帧的特征尺度
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {   // 该MapPoint至少被三个关键帧观测到
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        // 步骤4：该局部关键帧90%以上的MapPoints能被其它关键帧（至少3个）观测到，则认为是冗余关键帧
        if(nRedundantObservations>0.90*nMPs)//plc: 原来是0.9
            pKF->SetBadFlag(); //设置坏帧的目的是在删除前处理号父帧与子帧的关系（坏帧如果是回环帧则不删）
    }
}

//反对称矩阵?
cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<    0,    -v.at<float>(2),   v.at<float>(1),
                               v.at<float>(2),      0,         -v.at<float>(0),
                               -v.at<float>(1),  v.at<float>(0),            0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
