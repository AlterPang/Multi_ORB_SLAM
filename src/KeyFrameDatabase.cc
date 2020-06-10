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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
    mvInvertedFile_cam1.resize(voc.size());
}

//用于DetectLoop()
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    //mvInvertedFile里的不同word id可以保存有相同的pKF
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);//关键帧添加到该变量:mvInvertedFile[i]表示包含了word id为i的所有关键帧(倒排索引)
}

//只添加cam1的数据
void KeyFrameDatabase::add_cam1(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec_cam1.begin(), vend=pKF->mBowVec_cam1.end(); vit!=vend; vit++)
        mvInvertedFile_cam1[vit->first].push_back(pKF);//关键帧添加到该变量:mvInvertedFile[i]表示包含了word id为i的所有关键帧(倒排索引)
}

/**
 * @brief 关键帧被删除后，更新数据库的倒排索引
 * @param pKF 关键帧
 */ //仅用于KeyFrame::SetBadFlag()
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
    //将当前帧剔除出数据库的倒排索引(cam1)
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec_cam1.begin(), vend=pKF->mBowVec_cam1.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile_cam1[vit->first]; //包含当前word的所有关键帧

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
    mvInvertedFile_cam1.clear();
    mvInvertedFile_cam1.resize(mpVoc->size());
}

/**
 * @brief 在闭环检测中找到与该关键帧pKF可能闭环的关键帧
 *
 * 1. 找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
 * 2. 只和具有共同单词较多(大于最低得分minScore)的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param pKF      需要闭环的关键帧
 * @param minScore 相似性分数最低要求
 * @return         可能与pKF闭环的关键帧
 * @see III-E Bags of Words Place Recognition
 */
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    // 提出所有与该pKF相连的KeyFrame，这些相连Keyframe都是局部相连，在闭环检测的时候将被剔除
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;//可能与pKF形成回环的候选帧（只要有相同的word，且不属于局部相连帧）

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // 步骤1：找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word (BowVec: <wordid, wordvalue>)todo cam2
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
//        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec_cam1.begin(), vend=pKF->mBowVec_cam1.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            // 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
//            list<KeyFrame*> &lKFs = mvInvertedFile_cam1[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)// pKFi还没有标记为pKF的候选帧
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))// 与pKF局部链接的关键帧不进入闭环候选帧
                    {
                        pKFi->mnLoopQuery=pKF->mnId;// pKFi标记为pKF的候选帧，之后直接跳过判断
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;// 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;//最多的公共单词
    }

    int minCommonWords = maxCommonWords*0.8f;//用最大值得到阈值?

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度(BoW得分)大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // pKF只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到

            //计算bow得分
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));//bow得分大于minCommonWords且单词匹配度大于minScore存入
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将每个候选关键帧与自己相连（共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 步骤4：具体而言：在lScoreAndMatch中,每一个KeyFrame都把与自己共视程度较高的帧归为一组(组数与候选关键帧数相等)
    // 每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;//遍历候选关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10); //该帧的相邻帧,得到一组

        float bestScore = it->first;// 该组最高分数
        float accScore = it->first; // 该组累计得分
        KeyFrame* pBestKF = pKFi;   // 该组最高分数对应的关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;//遍历该候选关键帧的相邻关键帧,但相邻关键帧也要在闭环候选关键帧中
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;//因为pKF2->mnLoopQuery==pKF->mnId，所以只有pKF2也在闭环候选帧中，才能贡献分数
                if(pKF2->mLoopScore>bestScore)// 统计得到组里分数最高的KeyFrame
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));//每组累计得分,该组最高得分的关键帧
        if(accScore>bestAccScore)// 记录所有组中的最高得分
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore; //BOW得分阈值

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // 步骤5：得到组得分大于minScoreToRetain的组，得到组中分数最高的关键帧 0.75*bestScore
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)//组的最高得分要大于0.75*bestScore的组才采用
        {
            KeyFrame* pKFi = it->second;//遍历各组的最高得分的关键帧
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中了
            {
                vpLoopCandidates.push_back(pKFi);//若不在闭环队列,则添加
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

/**
 * @brief 在闭环检测中找到与该关键帧pKF可能闭环的关键帧
 *
 * 1. 找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
 * 2. 只和具有共同单词较多(大于最低得分minScore)的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param pKF      需要闭环的关键帧
 * @param minScore 相似性分数最低要求
 * @return         可能与pKF闭环的关键帧
 * @see III-E Bags of Words Place Recognition
 */
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates_cam1(KeyFrame* pKF, float minScore)
{
    // 提出所有与该pKF相连的KeyFrame，这些相连Keyframe都是局部相连，在闭环检测的时候将被剔除
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames_cam1();
    list<KeyFrame*> lKFsSharingWords;//可能与pKF形成回环的候选帧（只要有相同的word，且不属于局部相连帧）

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // 步骤1：找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word (BowVec: <wordid, wordvalue>)todo cam2
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec_cam1.begin(), vend=pKF->mBowVec_cam1.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            // 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧
            list<KeyFrame*> &lKFs = mvInvertedFile_cam1[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)// pKFi还没有标记为pKF的候选帧
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))// 与pKF局部链接的关键帧不进入闭环候选帧
                    {
                        pKFi->mnLoopQuery=pKF->mnId;// pKFi标记为pKF的候选帧，之后直接跳过判断
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;// 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;//最多的公共单词
    }

    int minCommonWords = maxCommonWords*0.8f;//用最大值得到阈值?

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度(BoW得分)大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // pKF只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到

            //计算bow得分
            float si = mpVoc->score(pKF->mBowVec_cam1,pKFi->mBowVec_cam1);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));//bow得分大于minCommonWords且单词匹配度大于minScore存入
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将每个候选关键帧与自己相连（共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 步骤4：具体而言：在lScoreAndMatch中,每一个KeyFrame都把与自己共视程度较高的帧归为一组(组数与候选关键帧数相等)
    // 每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;//遍历候选关键帧
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames_cam1(10); //该帧的相邻帧,得到一组

        float bestScore = it->first;// 该组最高分数
        float accScore = it->first; // 该组累计得分
        KeyFrame* pBestKF = pKFi;   // 该组最高分数对应的关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;//遍历该候选关键帧的相邻关键帧,但相邻关键帧也要在闭环候选关键帧中
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;//因为pKF2->mnLoopQuery==pKF->mnId，所以只有pKF2也在闭环候选帧中，才能贡献分数
                if(pKF2->mLoopScore>bestScore)// 统计得到组里分数最高的KeyFrame
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));//每组累计得分,该组最高得分的关键帧
        if(accScore>bestAccScore)// 记录所有组中的最高得分
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore; //BOW得分阈值

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // 步骤5：得到组得分大于minScoreToRetain的组，得到组中分数最高的关键帧 0.75*bestScore
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)//组的最高得分要大于0.75*bestScore的组才采用
        {
            KeyFrame* pKFi = it->second;//遍历各组的最高得分的关键帧
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中了
            {
                vpLoopCandidates.push_back(pKFi);//若不在闭环队列,则添加
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

/**
 * @brief 在重定位中找到与该帧相似的关键帧
 *
 * 1. 找出和当前帧具有公共单词的所有关键帧
 * 2. 只和具有共同单词较多的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param F 需要重定位的帧(当前帧)
 * @return   相似的关键帧
 * @see III-E Bags of Words Place Recognition
 */  //用于tracking的检测重定位候选帧 todo only cam1
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    cout<<"KeyFrameDatabase.cc:: L235 DetectRelocalizationCandidates()..."<<endl;
    // 相对于关键帧的闭环检测DetectLoopCandidates，重定位检测中没法获得相连的关键帧
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    // 步骤1：找出和当前帧具有公共单词的所有关键帧
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历当前帧的每一个word.
        // mBowVec是word id 及其权重 //todo cam2 ?
        // 遍历F的word id
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec_cam1.begin(), vend=F->mBowVec_cam1.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            // mvInvertedFile里的不同word id可以保存有相同的pKF
            list<KeyFrame*> &lKFs =   mvInvertedFile_cam1[vit->first];

            //遍历包含该word的KeyFrame
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)//该kf未添加到重定位候选序列中
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;//将该kf添加到重定位候选序列中
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与当前帧F具有共同单词最多的单词数，并以此决定阈值
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    // 步骤3：遍历所有重定位候选帧，挑选出共有单词数大于阈值minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // 当前帧F只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到
            float si = mpVoc->score(F->mBowVec_cam1,pKFi->mBowVec_cam1);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 步骤4：计算候选帧组得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatc
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
//        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);//todo 仅cam1
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames_cam1(10);

        float bestScore = it->first; // 该组最高分数
        float accScore = bestScore;  // 该组累计得分
        KeyFrame* pBestKF = pKFi;    // 该组最高分数对应的关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore; // 只有pKF2也在闭环候选帧中，才能贡献分数
            if(pKF2->mRelocScore>bestScore) // 统计得到组里分数最高的KeyFrame
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore) // 记录所有组中组得分最高的组
            bestAccScore=accScore; // 得到所有组中最高的累计得分
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 步骤5：得到组得分大于阈值的，组内得分最高的关键帧
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        // 只返回累计得分大于minScoreToRetain的组中分数最高的关键帧 0.75*bestScore
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中了
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
