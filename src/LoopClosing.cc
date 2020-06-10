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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale,
                         const cv::Mat CalibMatrix):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0),
    mCalibMatrix(CalibMatrix.clone()) //标定矩阵由system.cc传入(4x3矩阵,不是T矩阵4x4)
{
    mnCovisibilityConsistencyTh = 3;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++
    mRcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);
    mtcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    mtcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    mtcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++=
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        // Loop closing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
        // 在LocalMapping中通过InsertKeyFrame将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
        // 步骤1： 闭环检测队列mlpLoopKeyFrameQueue中的关键帧不为空,会一直处理
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            // 步骤2：检测是否发生闭环 有相似的关键帧出现
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
                //步骤3： 发生闭环 计算相似变换  [sR|t]
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
                   cout<<"LoopClosing::L88 CorrectLoop完成,此时帧id= "<<mpCurrentKF->mnId<<endl;
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
//    cout<<"LoopClosing.cc::L97 InsertKeyFrame(): 将当前帧插入闭环监测队列..."<<endl;
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}


/**
 * @brief    检测是否发生闭环 出现相识的 关键帧
 * @return 如果存在，返回true
 */
bool LoopClosing::DetectLoop()
{
    // 步骤1： 从队列中取出一个关键帧
    {
//        cout<<"PLC:LoopClosing::DetectLoop() L126 "<<endl;
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    // 步骤2：如果距离上次闭环没多久（小于10帧），或者map中关键帧总共还没有10帧，则不进行闭环检测
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
//        mpKeyFrameDB->add(mpCurrentKF);//加入关键帧数据库, 添加到<word id, KFs> (word id,包含该id的所有KF)
        mpKeyFrameDB->add_cam1(mpCurrentKF);//只添加cam1的数据
        mpCurrentKF->SetErase();//删除 不进行闭环检测
//        cout<<"PLC:LoopClosing::DetectLoop(): L140 Loop NOT detected!"<<endl;
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    // 步骤3：遍历所有共视关键帧，计算当前关键帧与每个共视关键帧的bow相似度得分，并得到最低得分minScore
    // 当前帧的所有共视关键帧 TODO 改成双/多相机
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames_cam1(); //todo
//    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec_cam1;//当前帧的 BoW 字典单词描述向量 //todo
//    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    // 遍历每一个共视关键帧
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue; //TODO ↓这里mBowVec来自所有描述子, 计算一个相机还是两个?两个的话合起来还是分开算
        const DBoW2::BowVector &BowVec = pKF->mBowVec_cam1;// 每一个共视关键帧的BoW 字典单词描述向量//todo
//        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;// 最低得分minScore
    }
    // 步骤4：在所有关键帧数据库中找出与当前帧按最低得分minScore匹配的闭环候选帧(多个)
    // Query the database imposing the minimum score
//    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates_cam1(mpCurrentKF, minScore);
//    cout<<"LoopClosing.cc::L172 DetectLoopCandidates() 获得闭环候选帧数为: "<<vpCandidateKFs.size()<<endl;

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())// 没有闭环候选帧
    {
//        mpKeyFrameDB->add(mpCurrentKF);
        mpKeyFrameDB->add_cam1(mpCurrentKF);//只添加cam1的数据
        mvConsistentGroups.clear();// 具有连续性的候选帧群组 清空
        mpCurrentKF->SetErase();
//        cout<<"LoopClosing::L182 DetectLoop(): Loop NOT detected!"<<endl;
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    // 步骤5：在候选帧中检测具有连续性的候选帧
    // 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”，vpCandidateKFs --> spCandidateGroup
    // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在 nCurrentConsistency ++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
    // 3、如果 nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
    mvpEnoughConsistentCandidates.clear();// 最终筛选后得到的闭环帧
    // ConsistentGroup 数据类型为pair<set<KeyFrame*>,int>
    // ConsistentGroup:<每个“连续组”中的关键帧，每个“连续组”的序号>
    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);// 子连续组是否连续
    // 步骤5.1： 遍历每一个闭环候选帧
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];// 一个闭环候选帧

        // 步骤5.2： 将自己以及与自己相连的关键帧构成一个 “子候选组”
//        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();// 与自己相连的关键帧
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames_cam1();// 与自己相连的关键帧
        spCandidateGroup.insert(pCandidateKF);// 自己也算进去

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        // 步骤5.3：遍历之前的“子连续组”
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            // 取出一个"之前"的子连续组.每次子连续组mvConsistentGroups都在末尾更新,因此这里的子连续组是上次的
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            // 步骤5.4：	遍历子候选组, 检测候选组中每一个关键帧在“子连续组”中是否存在
            // 如果有一帧共同存在于“子候选组” 与之前的 “子连续组”，那么“子候选组”与该“子连续组”连续
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))// 有一帧共同存在于 “子候选组” 与之前的 “子连续组”
                {
                    bConsistent=true;// 那么“子候选组”与该“子连续组”连续
                    bConsistentForSomeGroup=true;// 该“子候选组”至少与一个”子连续组“相连
                    break;
                }
            }

            // 步骤5.5 ： 连续 (指“子候选组”与该“子连续组”连续)
            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;//与子候选组连续的子连续组序号
                int nCurrentConsistency = nPreviousConsistency + 1;//当前子连续组序号(或者说连续组的数量)
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);//子候选帧, 连续组序号
                    vCurrentConsistentGroups.push_back(cg);
                    // 设置连续组 连续标志
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                //大于等于3
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);// 加入闭环候选帧
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
                //这里添加了个break对不对?-----------------------///////////////////////////////////////////////////////////////////////////////
                break;//这里只是结束 步骤5.3 的for循环// 添加
                // 好像找到一个就返回 ture  这里是不是缺少一个 break;
                // 提前结束 不知道 mvpEnoughConsistentCandidates 在其他地方还有没有用到
                // 在 ComputeSim3() 中会用到 mvpEnoughConsistentCandidates
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        // 步骤6： 如果该“子候选组” 的所有关键帧都不存在于“子连续组”，那么 vCurrentConsistentGroups 将为空，
        // 于是就把“子候选组”全部拷贝到 vCurrentConsistentGroups，并最终用于更新 mvConsistentGroups，
        // 计数器设为0，重新开始
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);//<“子候选组”,编号0>(自己及相连的关键帧构成“子候选组”)
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    // 步骤7：更新连续组
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    // 步骤8：添加当前关键帧 到关键帧数据库

//    mpKeyFrameDB->add(mpCurrentKF);
    mpKeyFrameDB->add_cam1(mpCurrentKF);//只添加cam1的数据

    if(mvpEnoughConsistentCandidates.empty())
    {
//        cout<<"PLC::DetectLoop(): L272 Loop NOT detected!"<<endl;
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
//        cout<<"PLC:DetectLoop(): L278 Loop detected!"<<endl;
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

/**
 * @brief 计算当前帧与闭环帧的Sim3变换等
 *
 * 1. 通过Bow加速描述子的匹配，利用RANSAC粗略地计算出当前帧与闭环帧的Sim3（当前帧---闭环帧）
 * 2. 根据估计的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3（当前帧---闭环帧）
 * 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）
 *
 * 注意以上匹配的结果均都存在成员变量 mvpCurrentMatchedPoints 中，
 * 实际的更新步骤见CorrectLoop()步骤3：Start Loop Fusion
 *
 * 步骤1：遍历每一个闭环候选关键帧  构造 sim3 求解器
 * 步骤2：从筛选的闭环候选帧中取出一帧关键帧pKF
 * 步骤3：将当前帧mpCurrentKF与闭环候选关键帧pKF匹配 得到匹配点对
 *    步骤3.1 跳过匹配点对数少的 候选闭环帧
 *    步骤3.2：  根据匹配点对 构造Sim3求解器
 * 步骤4：对每一个候选闭环关键帧迭代  Sim3利用 相似变换求解器求解 候选闭环关键帧到档期帧的 相似变换
 * 步骤5：通过步骤4求取的Sim3变换，使用sim3变换匹配得到更多的匹配点 弥补步骤3中的漏匹配
 * 步骤6：G2O Sim3优化，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
 * 步骤7：如果没有一个闭环匹配候选帧通过Sim3的求解与优化 清空候选闭环关键帧
 * 步骤8：取出闭环匹配上 关键帧的相连关键帧，得到它们的地图点MapPoints放入mvpLoopMapPoints
 * 步骤9：将闭环匹配上关键帧以及相连关键帧的 地图点 MapPoints 投影到当前关键帧进行投影匹配 为当前帧查找更多的匹配
 * 步骤10：判断当前帧 与检测出的所有闭环关键帧是否有足够多的MapPoints匹配
 * 步骤11：满足匹配点对数>40 寻找成功 清空mvpEnoughConsistentCandidates
 * @return  计算成功 返回true
 */  //检测到闭环后 计算相似变换
bool LoopClosing::ComputeSim3()
{
//    cout<<"LoopClosing.cc::L320 ComputeSim3()"<<endl;
    // For each consistent loop candidate we try to compute a Sim3
    // 闭环候选帧数量
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);//构造函数初始化? TODO 多相机的话这里要改吧

    vector<Sim3Solver*> vpSim3Solvers;//相似变换求解器
    vpSim3Solvers.resize(nInitialCandidates);// 每个候选帧都有一个Sim3Solver

    vector<vector<MapPoint*> > vvpMapPointMatches;//每个候选闭环关键帧和当前帧都会匹配计算 匹配地图
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;// 候选闭环关键帧 好坏
    vbDiscarded.resize(nInitialCandidates);
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    // 步骤1：遍历每一个闭环候选关键帧 构造sim3求解器
    for(int i=0; i<nInitialCandidates; i++)
    {
        // 步骤2：从筛选的闭环候选帧中取出一帧关键帧pKF
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        // 防止在LocalMapping中KeyFrameCulling函数将此关键帧作为冗余帧剔除
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;// 该候选闭环帧 不好
            continue;
        }

        // 步骤3：将当前帧mpCurrentKF与闭环候选关键帧pKF匹配 得到匹配点对
        // 通过bow加速得到mpCurrentKF与pKF之间的匹配特征点，vvpMapPointMatches是匹配特征点对应的MapPoints
        // 通过词包加速匹配，对参考关键帧的地图点进行跟踪
        int nmatches = matcher.SearchByBoW_cam1(mpCurrentKF,pKF,vvpMapPointMatches[i]);//TODO
//        cout<<"LoopClosing.cc::L360 ComputeSim3: SearchByBoW()得到匹配点数: "<<nmatches<<endl;

        //步骤3.1 跳过匹配点对数少的 候选闭环帧
        if(nmatches<15)//跟踪到的点数量过少 匹配效果不好
        {
            cout << "======== NOT ENOUGH MATCHES (" << nmatches << ") ======== " << endl;
            vbDiscarded[i] = true;// 该候选闭环帧 不好
            continue;
        }
        else
        {
            cout << "======== RANSAC (" << nmatches << ") ========" << endl;
            //步骤3.2： 根据匹配点对 构造Sim3求解器
            // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目 多一个尺度缩放）
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mCalibMatrix,mbFixScale);
            //ransac: 从一组包含“局外点”的观测数据集中，通过迭代方式估计数学模型的参数
            pSolver->SetRansacParameters(0.99,15,300); // 设定参数: 至少20个内点inliers  最大300次迭代
            vpSim3Solvers[i] = pSolver;//对每一个闭环候选关键帧都构造一个sim3求解器
        }

        // 参与Sim3计算的候选关键帧数加1
        nCandidates++;
    }

    bool bMatch = false;// 用于标记是否有一个候选帧通过Sim3的求解与优化
    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    // 一直循环所有的候选帧，每个候选帧迭代5次，如果5次迭代后得不到结果，就换下一个候选帧
    // 直到有一个候选帧首次迭代成功bMatch为true，或者某个候选帧总的迭代次数超过限制，直接将它剔除
    // 步骤4：迭代每一个候选闭环关键帧 相似变换求解器求解
    while(nCandidates>0 && !bMatch)
    {
        // 遍历每一个候选闭环关键帧
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])//不好 直接跳过
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];//之前DetectLoop()得到的候选闭环关键帧

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;// 内点数量
            bool bNoMore;// 这是局部变量，在pSolver->iterate(...)内进行初始化

            Sim3Solver* pSolver = vpSim3Solvers[i];//对应的sim3求解器
            // 最多迭代5次，返回的Scm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyfram
            // 经过n次循环，每次迭代5次，总共迭代 n*5 次
            // 总迭代次数达到最大限制 300次 还没有求出合格的Sim3变换，该候选帧剔除
            if(bNoMore) // true即达到最大次数
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            // 得到了相似变换
            if(!Scm.empty())
            {
                //vvpMapPointMatches[i]是第i个候选帧的匹配特征点对应的MapPoints,大小为当前帧的特征点数量
                //vpMapPointMatches: 之前匹配的kf1中的地图点对应kf2有匹配的那部分,后面还需要补充漏匹配
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    // 保存符合sim3变换的内点inlier 的地图点MapPoint
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                // 步骤5：通过步骤4求取的Sim3变换，使用sim3变换匹配得到更多的匹配点 弥补步骤3中的漏匹配
                // [sR t;0 1] 。R、s、t都在iterate()中得到？
                cv::Mat R = pSolver->GetEstimatedRotation();// 候选帧pKF到当前帧mpCurrentKF的R（R12）
                cv::Mat t = pSolver->GetEstimatedTranslation();// 候选帧pKF到当前帧mpCurrentKF的t（t12），当前帧坐标系下，方向由pKF指向当前帧
                // 候选帧pKF到当前帧mpCurrentKF的变换尺度s（s12） 缩放比例, 双目和rgbd尺度固定为1
                const float s = pSolver->GetEstimatedScale();
                // 查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数，之前使用SearchByBoW进行特征点匹配时会有漏匹配）
                // 通过Sim3变换，确定pKF1的特征点在pKF2中的大致区域，同理，确定pKF2的特征点在pKF1中的大致区域
                // 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新匹配vpMapPointMatches
//                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5, mCalibMatrix);// 7.5 搜索半径参数
                matcher.SearchBySim3_cam1(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);// 7.5 搜索半径参数

                // 步骤6：G2O Sim3优化，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                // OpenCV的Mat矩阵转成Eigen的Matrix类型
                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);// 优化初始值
                // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目 多一维 空间尺度）
                // 优化mpCurrentKF与pKF对应的MapPoints间的Sim3，得到优化后的量gScm
                // todo cam2
                const int nInliers = Optimizer::OptimizeSim3_cam1(mpCurrentKF, pKF, vpMapPointMatches, gScm,
                                     10, mbFixScale);

//                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm,
//                        10, mbFixScale, mCalibMatrix);

                // If optimization is succesful stop ransacs and continue
                // 最后 G2O 符合的内点数大于20, 则求解成功(只要找到一个内点数大于20就算成功进入下一步)
                if(nInliers>=20)
                {
                    bMatch = true;
                    // mpMatchedKF 就是最终闭环检测出来与当前帧形成闭环的关键帧
                    mpMatchedKF = pKF;//候选帧中选出的闭环帧
                    // 得到从世界坐标系到该候选帧的Sim3变换，Scale=1
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    // 得到g2o优化后从世界坐标系到当前帧的Sim3变换
                    mg2oScw = gScm*gSmw;// T1W =  T12 * T2W (当前帧的Sim3位姿?)
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;//匹配点(当前帧与第一个通过的闭环帧的匹配点,大小为当前帧地图点大小)
                    // 只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                    break;
                }
            }
        }
    }

    cout<<"LoopClosing.cc::L474 ComputeSim3(): 迭代候选关键帧之后，";
    // 步骤7：如果没有一个闭环匹配候选帧通过Sim3的求解与优化 清空候选闭环关键帧
    if(!bMatch)
    {
        cout<<"没有得到候选关键帧"<<endl;
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
    cout<<"得到候选关键帧,进入下一步=================================================="<<endl;

    // 步骤8：取出闭环匹配上的关键帧的相连关键帧，得到它们的MapPoints放入mvpLoopMapPoints
    // 注意是匹配上的那个关键帧：mpMatchedKF
    // 将mpMatchedKF相连的关键帧全部取出来放入vpLoopConnectedKFs
    // 将vpLoopConnectedKFs的MapPoints取出来放入mvpLoopMapPoints
    // Retrieve MapPoints seen in Loop Keyframe and neighbors
//    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();// 闭环匹配上的关键帧的相连关键帧
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames_cam1();// todo cam2
    vpLoopConnectedKFs.push_back(mpMatchedKF);//连同自己
    mvpLoopMapPoints.clear();
//    vector<int> LoopMPCams;//↑闭环地图点对应的相机编号
    // 迭代每一个闭环关键帧及其相邻帧
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit; //每一个闭环关键帧及其相邻帧
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches_cam1();//对应帧的所有地图点 todo
//        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();//对应帧的所有地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];//每一个地图点
//            int cam = pKF->keypoint_to_cam.find(i)->second;//闭环关键字地图点相机编号
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);// 闭环地图点 加入该点
//                    vLoopMPCams.push_back(cam); //TODO 这个只是闭环地图点所在的相机
                    // 标记该MapPoint被mpCurrentKF闭环时观测到并添加，避免重复添加
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }


    // 步骤9：将闭环匹配上关键帧以及相连关键帧的MapPoints 投影到当前关键帧进行投影匹配, 为当前帧查找更多的匹配
    // 根据投影为当前帧查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数）
    // 根据Sim3变换，将每个mvpLoopMapPoints投影到mpCurrentKF上，并根据尺度确定一个搜索区域，
    // 根据该MapPoint的描述子与该区域内的特征点进行匹配，如果匹配误差小于TH_LOW即匹配成功，更新mvpCurrentMatchedPoints
    // mvpCurrentMatchedPoints将用于SearchAndFuse中检测当前帧MapPoints与匹配的MapPoints是否存在冲突
    // Find more matches projecting with the computed Sim3
    //todo cam1
//    matcher.SearchByProjection(mpCurrentKF, mScw,
//                               mvpLoopMapPoints, vLoopMPCams,
//                               mvpCurrentMatchedPoints,10, mCalibMatrix);// 10匹配距离 阈值
    matcher.SearchByProjection_cam1(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop

    // 步骤10：判断当前帧 与检测出的所有闭环关键帧是否有足够多的MapPoints匹配
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])//当前帧 关键点 找到匹配的地图点
            nTotalMatches++;//匹配点数 +
    }

    // 步骤11：满足匹配点对数>40 寻找成功 清空mvpEnoughConsistentCandidates
    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    // 没找到
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

/**
 * @brief 闭环融合 全局优化
 *
 * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧 mvpCurrentConnectedKFs 位姿
 *      以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧）
 * 2. 用当前帧在闭环地图点mvpLoopMapPoints中匹配的当前帧闭环匹配地图点mvpCurrentMatchedPoints
 *     更新当前帧之前的匹配地图点 mpCurrentKF->GetMapPoint(i)
 * 2. 将闭环帧以及闭环帧相连的关键帧的所有地图点 mvpLoopMapPoints 和当前帧相连的关键帧的点进行匹配
 * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph
 * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整
 * 5. 创建线程进行全局Bundle Adjustment
 *
 * mvpCurrentConnectedKFs    当前帧相关联的关键帧
 * vpLoopConnectedKFs         闭环帧相关联的关键帧        这些关键帧的地图点 闭环地图点 mvpLoopMapPoints
 * mpMatchedKF                与当前帧匹配的  闭环帧
 *
 * mpCurrentKF 当前关键帧    优化的位姿 mg2oScw     原先的地图点 mpCurrentKF->GetMapPoint(i)
 * mvpCurrentMatchedPoints  当前帧在闭环地图点中匹配的地图点  当前帧闭环匹配地图点
 */
void LoopClosing::CorrectLoop()
{
//    cout << "Loop detected!" << endl;
    cout<<"LoopClosing::L588 CorrectLoop() 矫正闭环..."<<endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    // 步骤0：请求局部地图停止，防止局部地图线程中InsertKeyFrame函数插入新的关键帧
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    // 步骤1：停止全局优化
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        // 这个标志位仅用于控制输出提示，可忽略
        mbStopGBA = true;//停止全局优化

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    // 步骤2：等待局部建图线程完全停止
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    // 步骤3：根据共视关系更新当前帧与其它关键帧之间的连接
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    // 步骤4：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
    // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
    // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换
    // todo 19.6.24: 开始修改成多相机++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames_cam1();//当前帧的相连关键帧
//    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();//当前帧的相连关键帧 todo
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);//也加入自己

    // 先将当前帧 mpCurrentKF 的Sim3变换存入，固定不动
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;// 得到闭环g2o优化后各个关键帧的位姿 没有优化的位姿
    CorrectedSim3[mpCurrentKF]=mg2oScw;// 当前帧 对应的 sim3位姿(在computeSim3中得到)
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();//当前帧---> 世界


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        // 步骤4.1：通过位姿传播，得到Sim3调整后其它与当前帧相连关键帧的位姿（只是得到，还没有修正）
        //  遍历与当前帧mpCurrentKF相连的关键帧
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;//与当前帧mpCurrentKF相连的关键帧

            cv::Mat Tiw = pKFi->GetPose();// 帧位姿(Tcw)

            // currentKF在前面已经添加
            if(pKFi!=mpCurrentKF)
            {
                // 得到当前帧到pKFi帧的相对变换
                cv::Mat Tic = Tiw*Twc;//Tciw*Twc=Tcic 即当前帧到帧i的变换
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                // 当前帧到帧i的sim3变换
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                // 当前帧的位姿mg2oScw 固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw; //世界到帧i的sim3变换
                //Pose corrected with the Sim3 of the loop closure
                // 得到闭环g2o优化后各个关键帧i的位姿
                CorrectedSim3[pKFi]=g2oCorrectedSiw; //(当前帧及其相连帧的)帧i矫正sim3后的位姿
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);//即Rciw
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            // 当前帧相连关键帧，没有进行闭环g2o优化的位姿
            NonCorrectedSim3[pKFi]=g2oSiw; //没有矫正尺度,只有位姿变化
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        // 步骤4.2：步骤4.1得到调整相连帧位姿后，修正这些关键帧的MapPoints
        // 遍历每一个相连帧 用优化的位姿修正帧相关的地图点
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;//当前帧mpCurrentKF的相邻帧i
            g2o::Sim3 g2oCorrectedSiw = mit->second;//对应帧的优化后的位姿(世界到帧)
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();//帧到世界

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];//未优化的位姿

            // 遍历帧的每一个地图点
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches(); //todo
//            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches_cam1();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)// 标记 防止重复修正
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                // 将该未校正的 eigP3Dw 先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);//opencv mat格式
                pMPi->SetWorldPos(cvCorrectedP3Dw);//更新地图点坐标值
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;// 标记 防止重复修正
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();//更新地图点观测方向等信息
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // 步骤4.3：将Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();//旋转矩阵 转到 旋转向量 roll pitch yaw
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();//相似变换尺度

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);//更新关键帧 位姿

            // Make sure connections are updated
            // 步骤4.4：根据共视关系更新当前帧与其它关键帧之间的连接
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        // 步骤5：检查当前帧的地图点MapPoints 与闭环检测时匹配的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补
        // 遍历每一个当前帧在闭环匹配时的地图点
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];//当前帧 在闭环检测匹配的地图点
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);//当前帧之前的地图点
                // 如果有重复的MapPoint（当前帧和匹配帧各有一个），则用闭环匹配得到的代替现有的
                // todo plc 所以需要两相机参与闭环检测
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                // 如果当前帧没有该MapPoint，则直接添加
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);//帧添加 关键点对应的 地图点
                    pLoopMP->AddObservation(mpCurrentKF,i);// 地图点 添加帧 和 其上对应的 关键点id
                    pLoopMP->ComputeDistinctiveDescriptors();//更新地图点描述子
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // 步骤6：通过将 闭环时相连关键帧的地图点mvpLoopMapPoints 投影到这些 已矫正的当前帧及相邻关键帧中，进行MapPoints检查与替换
    // 利用校正后的姿态将共视图关键帧所有地图点的位置全部更新，即利用旧姿态投影回像平面，再用新姿态投影到三维空间??
    SearchAndFuse(CorrectedSim3);//CorrectedSim3: 当前帧及相邻帧,及其校正后的位姿


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    // 步骤7：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;//新 一级二级相关联关系

    // 步骤7.1：遍历当前帧相连关键帧（一级相连）
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // 步骤7.2：得到与当前帧相连关键帧的相连关键帧（二级相连） 之前二级相邻关系
//        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames_cam1();

        // Update connections. Detect new links.
        // 步骤7.3：更新一级相连关键帧的连接关系
        pKFi->UpdateConnections();
        // 步骤7.4：取出该帧更新后的连接关系  新二级相邻关系
//        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames_cam1();
        // 步骤7.5：从新连接关系中 去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);// 新二级相邻关系 中删除旧 二级相连关系
        }
        // 步骤7.6：从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    // 步骤8：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    // 步骤9：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    // 这两句话应该放在OptimizeEssentialGraph之前，因为 OptimizeEssentialGraph 的步骤4.2中有优化，（wubo???）
    mpMatchedKF->AddLoopEdge(mpCurrentKF);//添加到mspLoopEdges,有什么用?
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    // 步骤10：新建一个线程用于全局BA优化
    // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
    // 全局优化运算量大所以新建一个线程
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

/**
 * @brief  通过将闭环时相连关键帧上所有的MapPoints投影到这些 关键帧中，进行MapPoints检查与替换
 * @param CorrectedPosesMap  闭环相邻帧及其对应的sim3位姿 (闭环当前帧及其相邻帧?)
 */
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    cout<<"LoopClosing.cc::L807 Searchandfuse()"<<endl;
    // 遍历每一个帧sim3位姿
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;//每一个相邻帧

        g2o::Sim3 g2oScw = mit->second;//sim3位姿
        cv::Mat cvScw = Converter::toCvMat(g2oScw);
        // mvpLoopMapPoints为闭环时相邻关键帧上的所有地图点(闭环地图点)
        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        // 将闭环相连帧的MapPoints坐标变换到pKF帧坐标系，然后投影，检查冲突并融
        //pKF: 当前关键帧及其相邻关键帧
        //mvpLoopMapPoints:筛选后的闭环关键帧及其相邻帧的地图点. vLoopMPCams:vpPoints的各点相机编号
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,vLoopMPCams,4,vpReplacePoints, mCalibMatrix); //todo
//        matcher.Fuse_cam1(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints); //todo
        //对相邻帧匹配的地图点 融合更新 vpReplacePoints是地图点的融合
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];//地图点被关键帧上的点代替的点  关键帧上的点
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);// 用mvpLoopMapPoints替换掉之前的 再替换回来？？
            }
        }
    }
}


void LoopClosing::RequestReset()
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
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

/**
* @brief    请求闭环检测线程重启
* @param nLoopKF 闭环当前帧 id
*/
void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;


    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false, mCalibMatrix); //如何优化cam2的点??

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    // 更新地图点 和关键帧
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();// 停止建图
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            // 步骤1：更新关键帧  地图中所有的关键帧
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();// 地图中的关键帧
                const set<KeyFrame*> sChilds = pKF->GetChilds();// 孩子帧
                cv::Mat Twc = pKF->GetPoseInverse();
                // 遍历每一个孩子帧
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)//跳过 闭环发生事时的当前帧 避免重复
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;// 父亲帧到孩子帧
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;// 标记

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            // 步骤2：更新 地图点
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)// 关键帧优化过 更新地图点
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();//地图点参考帧

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;//todo

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);//todo
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
