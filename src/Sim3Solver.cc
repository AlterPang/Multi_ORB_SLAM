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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{

//vpMatched12: 两帧中已匹配的地图点
//vpMatched12表示kf1的地图点中有哪些和kf2对应,  vpMatched12[i]=kf2->mp[idx2] ,未对应的i为null
Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12,
                       const cv::Mat CalibMatrix,const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)// Fix scale in 双目/rgbd
{
    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();// vpMatched12表示kf1中有匹配的地图点,大小为kf1的地图点总数量(未匹配点为null)

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;//kf1中匹配的地图点,为匹配点为null.这个变量没有用到?
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    cv::Mat Rcw1 = pKF1->GetRotation();//当前帧kf1的旋转
    cv::Mat tcw1 = pKF1->GetTranslation();//kf1的位移
    cv::Mat Rcw2 = pKF2->GetRotation();//候选帧kf2
    cv::Mat tcw2 = pKF2->GetTranslation();

    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//todo 后面的要不要改???
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
//    const cv::Mat tcam12 = CalibMatrix.row(3).t();
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
//    const cv::Mat Rcam21 = Rcam12.t();
//    const cv::Mat tcam21 = -Rcam21 * tcam12;
    mRcam21 = Rcam12.t();
    mtcam21 = -mRcam21 * tcam12;

    mvAllIndices.reserve(mN1);

    size_t idx=0;
    // mN1为当前帧pKF1特征点的个数
    for(int i1=0; i1<mN1; i1++)
    {
        // 如果该特征点在pKF2中有匹配
        if(vpMatched12[i1])
        {
            // pMP1和pMP2是匹配的MapPoint
            MapPoint* pMP1 = vpKeyFrameMP1[i1];// 匹配的kf1地图点
            MapPoint* pMP2 = vpMatched12[i1];// 匹配的kf1中第i1个地图点对应的kf2的地图点

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            // indexKF1和indexKF2是匹配特征点的索引
            int indexKF1 = pMP1->GetIndexInKeyFrame_cam1(pKF1);//pKF1匹配点mMP1在关键帧pKF1的索引(无则为-1)
            int indexKF2 = pMP2->GetIndexInKeyFrame_cam1(pKF2);//pKF2匹配点mMP2在关键帧pKF2的索引

            if(indexKF1<0 || indexKF2<0)
                continue;

            // kp1和kp2是匹配地图点对应的特征点
//            const cv::KeyPoint &kp1 = pKF1->mvKeysUn_total[indexKF1];
//            const cv::KeyPoint &kp2 = pKF2->mvKeysUn_total[indexKF2];
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            //octave是特征点提取的金字塔层数
            //mvLevelSigma2[i]金字塔第i层尺度因子的平方
            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            // mvpMapPoints1和mvpMapPoints2是匹配的MapPoints容器
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            int cam1 = pKF1->keypoint_to_cam.find(indexKF1)->second;
            cv::Mat X3D1w = pMP1->GetWorldPos();
            cv::Mat x3dc1 = Rcw1*X3D1w+tcw1; // todo 这个变量用来计算变换误差,因此全部都用相机1的坐标就行?
            mvX3Dc1.push_back(x3dc1);//就算是其他相机的点也先用第一个相机的坐标?
            camIdx1.push_back(cam1);//该点在当前关键帧pkf1中属于哪个相机
//            if (indexKF1 < pKF1->N )
//                mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);
//            else if (indexKF1 >= pKF1->N )
//                mvX3Dc1.push_back(Rcam21*Rcw1*X3D1w+Rcam21*tcw1);

            int cam2 = pKF2->keypoint_to_cam.find(indexKF2)->second;
            cv::Mat X3D2w = pMP2->GetWorldPos();
            cv::Mat x3dc2 = Rcw2*X3D2w+tcw2; //todo 同上
            mvX3Dc2.push_back(x3dc2);
            camIdx2.push_back(cam2);//该点在候选帧pkf2中属于哪个相机
//            if (indexKF2 < pKF2->N )
//                mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);
//            else if (indexKF2 >= pKF2->N )
//                mvX3Dc2.push_back(Rcam21*Rcw2*X3D2w+Rcam21*tcw2);
//            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    //各帧的相机坐标投影到自己的像素坐标
//    FromCameraToImage(mvX3Dc1,mvP1im1,mK1); // mvP1im1是匹配地图点的像素坐标, 用来checkinliers
    FromCameraToImage(mvX3Dc1,mvP1im1,mK1,camIdx1); // mvP1im1是匹配地图点的像素坐标, 用来checkinliers
//    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2,camIdx2);

    SetRansacParameters();
}
//参数为0.99 20 30
//最少内点,最大迭代次数
void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    cout<<"Sim3Solver.cc::L156 SetRansacParameters()"<<endl;
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences; 匹配点数量

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    //根据probability, epsilon和 max iterations 设置 RANSAC 迭代次数
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;//总迭代次数?
}

// Ransac求解mvX3Dc1和mvX3Dc2之间Sim3，函数返回mvX3Dc2到mvX3Dc1的Sim3变换
//返回的Scm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    cout<<"Sim3Solver.cc::L186 iterate()"<<endl;
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)//N:匹配点数量?
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    int nCurrentIterations = 0;//迭代次数
    //进行迭代
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;// 这个函数中迭代的次数(最大为5)
        mnIterations++;// 总的迭代次数，默认为最大为300

        vAvailableIndices = mvAllIndices;//储存的是匹配点的序号:0,1,2...(连续序号,不是匹配点在帧的索引)

        // Get min set of points
        // 步骤1：任意取三组点算Sim矩阵
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);//匹配点中随机抽一个

            int idx = vAvailableIndices[randi];

            // P3Dc1i和P3Dc2i中点的排列顺序：
            // x1 x2 x3 ...
            // y1 y2 y3 ...
            // z1 z2 z3 ...
            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));//匹配点在当前帧kf1的相机坐标 // todo mvX3Dc1: kf1中相对相机1的坐标
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));//匹配点在候选帧kf2的相机坐标

            //剔除抽出的这个匹配点(防止下次随机到)
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 步骤2：根据两组匹配的3D点(3对)，计算之间的Sim3变换(变换矩阵T12)
        ComputeSim3(P3Dc1i,P3Dc2i);//两帧的相机坐标 TODO 两个相机之间的匹配怎么办

        // 步骤3：通过投影误差进行inlier检测
        CheckInliers();//即把一个3D点在kf1的像素坐标与其在kf2的坐标通过T21变换到kf1后计算误差.(及kf1投影到kf2)

        if(mnInliersi>=mnBestInliers)// 只要计算得到一次合格的Sim变换，就直接返回
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();//mT12i: 步骤2的 ComputeSim3()得到sim3变换
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

//计算多个点的质心
//P 匹配的点(坐标竖着排列)
//Pr 求得的相对质心坐标
//C 质心
void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);//P 各坐标分别求和存在C(xyz竖着排列)中
    C = C/P.cols;//得到质心

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;//相对质心坐标?
    }
}

// 根据两组匹配的3D点，计算之间的Sim3变换(变换矩阵Ts?) TODO 要是两相机都有匹配怎么办
//匹配点(3对)分别在当前帧和候选帧中的相机坐标
void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // ！！！！！！！这段代码一定要看这篇论文！！！！！！！！！！！
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates //质心和相对坐标

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (相对质心坐标)(set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (相对质心坐标)(set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1 (质心)
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2 (质心)

    // O1和O2分别为P1和P2矩阵中3D点的质心
    // Pr1和Pr2为3D点相对质心坐标
    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix
    //计算M矩阵. 引出M矩阵的目的是为了表示N矩阵
    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix
    // 计算N矩阵
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue
    // N矩阵特征分解
    cv::Mat eval, evec;

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    // N矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3）
    // 将(q1 q2 q3)放入vec行向量，vec就是四元数旋转轴乘以sin(ang/2)
    // 四元数转欧拉角: q = [cos(θ/2), nsin(θ/2)] , 其中n是旋转轴,θ是旋转角
    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));//旋转向量(轴角)中的旋转角θ

    //从旋转向量(轴角)?
    vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

    mR12i.create(3,3,P1.type());

    //从旋转向量(轴角)中计算旋转矩阵R
    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2

    cv::Mat P3 = mR12i*Pr2;

    // Step 6: Scale
    // 计算尺度s
    if(!mbFixScale) //stereo/RGBD的尺度固定为1
    {
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f; //stereo/RGBD 尺度s为1

    // Step 7: Translation
    // 计算平移t
    mt12i.create(1,3,P1.type());
    mt12i = O1 - ms12i*mR12i*O2;//平移

    // Step 8: Transformation
    // 构造相似变换矩阵T12
    // Step 8.1 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;//尺度s*旋转矩阵R

    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));//加上尺度
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));//加上平移

    // Step 8.2 T21
    //得到T21
    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}


//即把一个3D点在kf1的像素坐标与其在kf2的坐标通过T21变换到kf1后计算误差.(及kf1投影到kf2)
void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    // 把kf2中的匹配点经过Sim3变换(mT12i)到kf1中再与kf1中匹配点计算的坐标计算重投影坐标
    Project(mvX3Dc2,vP2im1,mT12i,mK1,camIdx2);
    // 把kf1中的匹配点经过Sim3变换(mT21i)到kf2中再与kf2中匹配点计算的坐标计算重投影坐标
    Project(mvX3Dc1,vP1im2,mT21i,mK2,camIdx1);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)//mvP1im1是帧1的点投影到帧1的像素坐标
    {
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];// 把2系中的3D经过Sim3变换(mT12i)到1系中计算重投影坐标
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];// 把1系中的3D经过Sim3变换(mT21i)到2系中计算重投影坐标

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        //两个误差都小于阈值,则设为内点
        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}

//返回最佳的mR12i
cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}

// stereo/RGBD的尺度固定为1
float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

// 世界坐标投影到像素坐标
// 这里是帧2的相机坐标转换到帧1的相机坐标再投影到像素坐标
void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K,
                         std::vector<int> camIdxs )
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;//转到另一帧的相机坐标上

        if (camIdxs[i] ==1)
        {
            P3Dc = mRcam21* P3Dc +mtcam21; //如果是相机2的点转到相机2坐标系
        }

        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));//像素坐标
    }
}

// 参数: 相机坐标, 像素坐标, 相机参数
// 相机1坐标系投影到各相机像素坐标
void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K,
                                   vector<int> camIdxs)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();//用来储存匹配地图点的像素坐标?
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)//遍历匹配地图点(相机坐标)
    {

//        cv::Mat vP3Dci ;
//        if (camIdxs[i] ==1)
//        {
//            vP3Dci=mRcam21* vP3Dc[i] +mtcam21; //如果是相机2的点先转到相机2坐标系
//        }
//        else
//            vP3Dci=vP3Dc[i];

        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;//得到归一化坐标
//        const float invz = 1/(vP3Dci.at<float>(2));
//        const float x = vP3Dci.at<float>(0)*invz;
//        const float y = vP3Dci.at<float>(1)*invz;//得到归一化坐标

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));//像素坐标
    }
}

} //namespace ORB_SLAM
