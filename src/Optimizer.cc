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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

#include "g2o_sim3_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{

// pMap中所有的MapPoints和关键帧做bundle adjustment优化
// 这个全局BA优化在本程序中有两个地方使用：
// a.单目初始化：CreateInitialMapMonocular函数
// b.闭环优化：RunGlobalBundleAdjustment函数
void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust, cv::Mat mCalibMatrix)
{
//    return; // note plc 不使用优化
    cout<<"Optimizer.cc:: L47 GlobalBundleAdjustemnt()..."<<endl;
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust, mCalibMatrix);
    cout<<"Optimizer.cc:: L51 GlobalBundleAdjustemnt()结束"<<endl;
}

/**
 * @brief bundle adjustment Optimization
 *
 * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n (plc:优化相机位姿的以及地图点???)
 *
 * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
 *            g2o::VertexSBAPointXYZ()，MapPoint的mWorldPos
 * 2. Edge:
 *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
 *         + Vertex：待优化当前帧的Tcw
 *         + Vertex：待优化MapPoint的mWorldPos
 *         + measurement：MapPoint在当前帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 * @param   vpKFs    关键帧
 *          vpMP     MapPoints
 *          nIterations 迭代次数（20次）
 *          pbStopFlag  是否强制暂停
 *          nLoopKF  关键帧的个数
 *          bRobust  是否使用核函数
 */ //全局优化

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust,
                                 cv::Mat CalibMatrix)  //plc 增加标定矩阵(4x3)
{
    cout<<"Optimizer.cc:: L79 BundleAdjustemnt()..."<<endl;
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    // 步骤1：初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // 步骤2：向优化器添加顶点

    // Set KeyFrame vertices
    // 步骤2.1：向优化器添加关键帧位姿顶点
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);//设置(关键帧)顶点id = 关键帧id
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    //设定Tcam21.Tcam11  **************************************************
    cv::Mat Tcam21=cv::Mat::zeros(4,4,CV_32F);
    cv::Mat Rcam21=(CalibMatrix.rowRange(0,3).colRange(0,3)).t();  //Rcam21 = Rcam12.t
//    cv::Mat tcam21;
    cv::Mat tcam21=cv::Mat::zeros(3,1,CV_32F);
    tcam21.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam21.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam21.at<float>(2,0) = CalibMatrix.at<float>(3,2); //这里是tcam12
    tcam21 = -Rcam21 * tcam21; //t21 = -R21 * t12

    Rcam21.copyTo(Tcam21.rowRange(0,3).colRange(0,3));
    tcam21.copyTo(Tcam21.rowRange(0,3).col(3));
    Tcam21.at<float>(3, 3)=1.0;

    cv::Mat Tcam11= cv::Mat::eye(4,4,CV_32F); //T11是单位矩阵

//    vector<g2o::SE3Quat> Tcim = {Converter::toSE3Quat(Tcam11),Converter::toSE3Quat(Tcam21)};
    vector<g2o::SE3Quat,Eigen::aligned_allocator<g2o::SE3Quat> > Tcim(2);
//    vector<g2o::SE3Quat> Tcim(2);
    Tcim[0] = Converter::toSE3Quat(Tcam11);
    Tcim[1] = Converter::toSE3Quat(Tcam21);
    // *********************************************************************

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    // 步骤2.2：向优化器添加MapPoints顶点
    //遍历所有地图点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
//        const int id = pMP->mnId+maxKFid*2+2;//设置(地图点)顶点id = (当前地图点id+最大关键帧id+1)
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();//map<观测到该点的KF,该点在KF上的id>

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            //遍历 观察到该点的关键帧
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            // mit->second: id
//            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];//pKF中对应该地图点的特征点
            const cv::KeyPoint &kpUn = pKF->mvKeysUn_total[mit->second];//该点在观察到该点的关键帧对应的关键点

            int cam = pKF->keypoint_to_cam.find(mit->second)->second; //所在相机

            if(pKF->mvuRight_total[mit->second]<0)//单目
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                // 二元边, 即要优化MapPoints的位置，又要优化相机的位姿
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                // 设置第一个顶点:
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//地图点id
                // 设置第二个顶点:
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));//观察到该点的关键帧id

                // 设置观测值:
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                // 设置信息矩阵: 协方差矩阵之逆. 每条边需设定一个信息矩阵作为不确定性的度量
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)//是否使用核函数
                {
                    // 设置鲁棒核函数
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                // 设置相机内参
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
//                cout<<"cam = "<<cam<<endl;
                e->Tcim_quat = Tcim[cam];

                optimizer.addEdge(e);
            }
            else//双目,rgbd
            {
                Eigen::Matrix<double,3,1> obs;
//                const float kp_ur = pKF->mvuRight[mit->second];
                const float kp_ur = pKF->mvuRight_total[mit->second];
                //todo 相机2的pt.x.y和相机1不一样
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                // BA中的重投影误差，将地图点投影到相机坐标系下的相机平面 todo 所以应该先把cam2的点投到cam1?指kpUn,ur
                //todo cam2的点相机2坐标 -> 相机1坐标, mvuRight同理 kpUn_cam21, mvuRight_cam21
                //地图点位置固定，是二元边
                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                // Edge边 连接 地图点顶点和关键帧顶点
                //添加该地图点id
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //添加观察到该点的关键帧id
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
//                if(mit->second >= pKF->N)
//                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId+maxKFid+1)));
//                else
//                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                //测量是关键点的像素坐标
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];//cam1 cam2的金字塔层数不同?
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);//信息矩阵

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;
                e->Tcim_quat = Tcim[cam];

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
//        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid*2+2));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

//@pFrame 当前帧
//1. 顶点 Vertex: g2o::VertexSE3Expmap()，初始值为当前帧的Tcw
//2. 边 Edge:
//	单目
//	- g2o::EdgeSE3ProjectXYZOnlyPose()，一元边 BaseUnaryEdge
//	+ 顶点 Vertex：待优化当前帧的Tcw
//	+ 测量值 measurement：MapPoint在当前帧中的二维位置(u,v)
//	+ 误差信息矩阵 InfoMatrix: Eigen::Matrix2d::Identity()*invSigma2(与特征点所在的尺度有关)
//	+ 附加信息： 相机内参数： e->fx fy cx cy
//		3d点坐标  ： e->Xw[0] Xw[1] Xw[2] 2d点对应的上一帧的3d点
//	双目
//	- g2o::EdgeStereoSE3ProjectXYZOnlyPose()，一元边 BaseUnaryEdge
//	+ Vertex：待优化当前帧的Tcw
//	+ measurement：MapPoint在当前帧中的二维位置(ul,v, ur) 左相机3d点 右相机横坐标匹配点坐标ur
//	+ InfoMatrix: invSigma2(与特征点所在的尺度有关)
//	+ 附加信息： 相机内参数： e->fx fy cx cy
//		3d点坐标  ： e->Xw[0] Xw[1] Xw[2] 2d点对应的上一帧的3d点

// 通过优化3D-2D的重投影误差来优化位姿
// 仅用于tracking中 todo cam2 先不优化?  todo 重定位的优化暂时只能单相机
int Optimizer::PoseOptimization(Frame *pFrame)
{
//    cout<<"Optimizer.cc::L286 PoseOptimization..."<<endl;
    // 该优化函数主要用于Tracking线程中：运动跟踪、参考帧跟踪、地图跟踪、重定位

    // note 计算局部ba时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 步骤1：构造g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    // 步骤2：添加顶点：待优化当前帧的Tcw
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);//设置当前帧 顶点id=0
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // todo 添加关键帧的相机2位姿?
    //设定Tcam21.Tcam11  **************************************************
//    cv::Mat Tcam21=cv::Mat::zeros(4,4,CV_32F);
//    cv::Mat Rcam21=pFrame->mRcam12.t();
//    cv::Mat tcam21 = -Rcam21 * pFrame->mtcam12;
//    Rcam21.copyTo(Tcam21.rowRange(0,3).colRange(0,3));
//    tcam21.copyTo(Tcam21.rowRange(0,3).col(3));
//    Tcam21.at<float>(3, 3)=1.0;
//    cv::Mat Tcam11= cv::Mat::eye(4,4,CV_32F); //T11是单位矩阵
//    vector<g2o::SE3Quat> Tcim(2);
//    Tcim[0] = Converter::toSE3Quat(Tcam11);
//    Tcim[1] = Converter::toSE3Quat(Tcam21);
    // *********************************************************************

    // Set MapPoint vertices
    // 设置地图点顶点
//    const int N = pFrame->N_total;
    const int N = pFrame->N; //Note

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    // 步骤3：添加一元边：相机投影模型
    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
//            int cam=0;
//            if(i>= pFrame->N){
//                cam=1;
//            }
            // Monocular observation
            // 单目情况, 也有可能在双目下, 当前帧的左兴趣点找不到匹配的右兴趣点
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
//                const cv::KeyPoint &kpUn = pFrame->mvKeysUn_total[i];
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i]; //note
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                // 设置1个顶点: 当前帧位姿
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                // 设置观测值,像素坐标
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                //设置信息矩阵，协方差
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);// 设置阈值，卡方自由度为2，内点概率95%对应的临界值

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
//                e->Tcim_quat = Tcim[cam];
                // 需要在这里设置<不做优化>的MapPoints的位置
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation 双目下
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;  //pFrame: 要优化的当前帧

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;// 这里和单目不同
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i]; //note
//                const cv::KeyPoint &kpUn = pFrame->mvKeysUn_total[i];
                const float &kp_ur = pFrame->mvuRight[i]; //note
//                const float &kp_ur = pFrame->mvuRight_total[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                //e是重投影误差? e = (u,v) - project(Tcw*Pw)，只优化Frame的Tcw，不优化MapPoints的坐标
                // 一元边
                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();// 这里和单目不同

                // 设置1个顶点: 当前帧位姿
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                // 设置观测值,该点像素坐标
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
//                e->Tcim_quat = Tcim[cam];
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // 步骤4：开始优化，总共优化四次，每次优化后，将观测分为outlier和inlier，outlier不参与下次优化
    // 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    // 四次迭代，每次迭代的次数

//    cout<<"Optimizer.cc::L511 PoseOptimization开始优化"<<endl;
    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);// 对level为0的边进行优化
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)//单目
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();// NOTE g2o只会计算active edge的误差
            }

            const float chi2 = e->chi2();//chi2 就是 error*\Omega*error

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);  // 设置为outlier
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);  // 设置为inlier
            }

            if(it==2)
                e->setRobustKernel(0);// 除了前两次优化需要RobustKernel以外, 其余的优化都不需要
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)//双目或rgbd
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];//储存的是地图点编号

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])//误差大于阈值
            {
                pFrame->mvbOutlier[idx]=true;//设为外点
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;//设为内点
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }

//    cout<<"Optimizer.cc::L583 PoseOptimization结束得到优化后位姿..."<<endl;
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    //tracking 优化位姿的结果
    pFrame->SetPose(pose);

//    cout<<"Optimizer.cc::L583 PoseOptimization位姿优化后得到内点(?): "<<nInitialCorrespondences-nBad<<endl;
    // note 计算重定位时间
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tpo= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"位姿优化成功(单相机)，花费时间为： "<<tpo<<endl;

    return nInitialCorrespondences-nBad;
}

int Optimizer::PoseOptimization(Frame *pFrame,bool bAllCams)
{
//    return 100; // note plc 不使用优化

//    cout<<"Optimizer.cc::L611 All Cames PoseOptimization..."<<endl;
    // 该优化函数主要用于Tracking线程中：运动跟踪、参考帧跟踪、地图跟踪、重定位

    // note 计算局部ba时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 步骤1：构造g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    // 步骤2：添加顶点：待优化当前帧的Tcw
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);//设置当前帧 顶点id=0
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // todo 添加当前帧的相机2位姿?
    //设定Tcam21.Tcam11  **************************************************
    cv::Mat Tcam21=cv::Mat::zeros(4,4,CV_32F);
    cv::Mat Rcam21=pFrame->mRcam12.t();
    cv::Mat tcam21 = -Rcam21 * pFrame->mtcam12;
    Rcam21.copyTo(Tcam21.rowRange(0,3).colRange(0,3));
    tcam21.copyTo(Tcam21.rowRange(0,3).col(3));
    Tcam21.at<float>(3, 3)=1.0;
    cv::Mat Tcam11= cv::Mat::eye(4,4,CV_32F); //T11是单位矩阵
//    vector<g2o::SE3Quat> Tcim = {Converter::toSE3Quat(Tcam11),Converter::toSE3Quat(Tcam21)};
//    cout<<"Tcam11 = "<<Tcam11<<endl;
//    cout<<"Tcam21 = "<<Tcam21<<endl;
//    vector<g2o::SE3Quat> Tcim(2);
    vector<g2o::SE3Quat,Eigen::aligned_allocator<g2o::SE3Quat> > Tcim(2);
    Tcim[0] = Converter::toSE3Quat(Tcam11);
    Tcim[1] = Converter::toSE3Quat(Tcam21);
    // *********************************************************************

    // Set MapPoint vertices
    // 设置地图点顶点
    const int N = pFrame->N_total;
//    const int N = pFrame->N; //Note

    vector<g2o::EdgeSE3ProjectXYZOnlyPose_multi*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose_multi*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    // 步骤3：添加一元边：相机投影模型
    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                int cam=0;
                if(i>= pFrame->N)
                    cam=1;
                // Monocular observation
                // 单目情况, 也有可能在双目下, 当前帧的左兴趣点找不到匹配的右兴趣点
//                if(pFrame->mvuRight[i]<0)
                if(pFrame->mvuRight_total[i]<0)
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn_total[i];
//                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i]; //note
                    obs << kpUn.pt.x, kpUn.pt.y;

//                    g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                    g2o::EdgeSE3ProjectXYZOnlyPose_multi* e = new g2o::EdgeSE3ProjectXYZOnlyPose_multi();

                    // 设置1个顶点: 当前帧位姿
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    // 设置观测值,像素坐标
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    //设置信息矩阵，协方差
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);// 设置阈值，卡方自由度为2，内点概率95%对应的临界值

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
//                    cout<<"cam = "<<cam<<endl;
                    e->Tcim_quat = Tcim[cam];
                    // 需要在这里设置<不做优化>的MapPoints的位置
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else  // Stereo observation 双目下
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;  //pFrame: 要优化的当前帧

                    //SET EDGE
                    Eigen::Matrix<double,3,1> obs;// 这里和单目不同
//                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i]; //note
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn_total[i];
//                    const float &kp_ur = pFrame->mvuRight[i]; //note
                    const float &kp_ur = pFrame->mvuRight_total[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    //e是重投影误差? e = (u,v) - project(Tcw*Pw)，只优化Frame的Tcw，不优化MapPoints的坐标
                    // 一元边
                    g2o::EdgeStereoSE3ProjectXYZOnlyPose_multi* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose_multi();// 这里和单目不同

                    // 设置1个顶点: 当前帧位姿
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    // 设置观测值,该点像素坐标
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    e->bf = pFrame->mbf;
//                    cout<<"L763 cam = "<<cam<<endl;
                    e->Tcim_quat = Tcim[cam];
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
            }

        }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // 步骤4：开始优化，总共优化四次，每次优化后，将观测分为outlier和inlier，outlier不参与下次优化
    // 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    // 四次迭代，每次迭代的次数

//    cout<<"Optimizer.cc::L511 PoseOptimization开始优化"<<endl;
    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);// 对level为0的边进行优化
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)//单目
        {
            g2o::EdgeSE3ProjectXYZOnlyPose_multi* e = vpEdgesMono[i];
//            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();// NOTE g2o只会计算active edge的误差
            }

            const float chi2 = e->chi2();//chi2 就是 error*\Omega*error

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);  // 设置为outlier
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);  // 设置为inlier
            }

            if(it==2)
                e->setRobustKernel(0);// 除了前两次优化需要RobustKernel以外, 其余的优化都不需要
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)//双目或rgbd
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose_multi* e = vpEdgesStereo[i];
//            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];//储存的是地图点编号

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])//误差大于阈值
            {
                pFrame->mvbOutlier[idx]=true;//设为外点
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;//设为内点
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }

//    cout<<"Optimizer.cc::L583 PoseOptimization结束得到优化后位姿..."<<endl;
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    //tracking 优化位姿的结果
    pFrame->SetPose(pose);

//    cout<<"Optimizer.cc::L583 PoseOptimization位姿优化后得到内点数(?): "<<nInitialCorrespondences-nBad<<endl;

    // note 计算重定位时间
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tpo= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"位姿优化成功(多相机)，花费时间为： "<<tpo<<endl;

    return nInitialCorrespondences-nBad;
}

/**
 * @brief Local Bundle Adjustment
 *
 * 1. Vertex:
 *     - g2o::VertexSE3Expmap()，LocalKeyFrames，即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
 *     - g2o::VertexSE3Expmap()，FixedCameras，即能观测到LocalMapPoints的关键帧（并且不属于LocalKeyFrames）的位姿，在优化中这些关键帧的位姿不变
 *     - g2o::VertexSBAPointXYZ()，LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的位置
 * 2. Edge:
 *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
 *         + Vertex：关键帧的Tcw，MapPoint的Pw
 *         + measurement：MapPoint在关键帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *     - g2o::EdgeStereoSE3ProjectXYZ()，BaseBinaryEdge
 *         + Vertex：关键帧的Tcw，MapPoint的Pw
 *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 * @param pKF        当前KeyFrame
 * @param pbStopFlag 是否停止优化的标志
 * @param pMap       在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate
 */
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{
//    return; // note plc 不使用优化

//    cout<<"Optimizer.cc::L514 LocalBundleAdjustment(): 局部BA优化..."<<endl;
    // 该优化函数用于LocalMapping线程的局部BA优化
    // note 计算局部ba时间
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    // 步骤1：将当前关键帧加入lLocalKeyFrames
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    // 步骤2：找到关键帧连接的关键帧（一级相连），加入lLocalKeyFrames中
//    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames_cam1();//返回已排序的共视关键帧 (仅cam1)
//    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
//    {
//        KeyFrame* pKFi = vNeighKFs[i];
//        pKFi->mnBALocalForKF = pKF->mnId;
//        if(!pKFi->isBad())
//            lLocalKeyFrames.push_back(pKFi);
//    }
    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();//返回已排序的共视关键帧
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    // 步骤3：遍历lLocalKeyFrames中关键帧，将它们观测的MapPoints加入到lLocalMapPoints
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
//        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches(); //todo 仅相机1
//        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches_cam1();
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;// 防止重复添加
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    // 步骤4：得能到观测到局部MapPoints，但不属于局部关键帧(没有和当前帧相连)的关键帧(lFixedCameras)，这些关键帧在局部BA优化时不优化
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();//todo 相机2
//        map<KeyFrame*,size_t> observations = (*lit)->GetObservations_cam1();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            // pKFi->mnBALocalForKF!=pKF->mnId表示局部关键帧，
            // 其它的关键帧虽然能观测到，但不属于局部关键帧
            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;// 防止重复添加
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);//这些帧不优化
            }
        }
    }

    // Setup optimizer
    // 步骤5：构造g2o优化器
    g2o::SparseOptimizer optimizer;  //优化器是最终要维护的东西
    //每个误差项优化变量(这里是位姿)维度为6, 误差值(这里是3D坐标)维度为3
    //构造线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;//(块求解器包含线性求解器)
    //线性求解器求解的是eigen的方式 ??
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    //矩阵块求解器
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    //梯度下降方法: LM(列文伯格-马夸尔特法)
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);//设置求解器

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    // 步骤6：添加顶点：Pose of Local KeyFrame(这里第一帧fix)
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);//第一帧位置固定
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    // 步骤7：添加顶点：Pose of Fixed KeyFrame，注意这里调用了vSE3->setFixed(true)(这里全部fix)。这里不优化
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);//全部固定,和步骤6不一样
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    //设定Tcam21.Tcam11  **************************************************
    cv::Mat Tcam21=cv::Mat::zeros(4,4,CV_32F);
    cv::Mat Rcam21=pKF->mRcam12.t();
//    cv::Mat tcam21=pKF->mtcam12;
//    tcam21 = -Rcam21 * tcam21;
    cv::Mat tcam21 = -Rcam21 * pKF->mtcam12;
    Rcam21.copyTo(Tcam21.rowRange(0,3).colRange(0,3));
    tcam21.copyTo(Tcam21.rowRange(0,3).col(3));
    Tcam21.at<float>(3, 3)=1.0;
    cv::Mat Tcam11= cv::Mat::eye(4,4,CV_32F); //T11是单位矩阵
//    vector<g2o::SE3Quat> Tcim = {Converter::toSE3Quat(Tcam11),Converter::toSE3Quat(Tcam21)};
//    vector<g2o::SE3Quat> Tcim(2);
    vector<g2o::SE3Quat,Eigen::aligned_allocator<g2o::SE3Quat> > Tcim(2);
    Tcim[0] = Converter::toSE3Quat(Tcam11);
    Tcim[1] = Converter::toSE3Quat(Tcam21);
//    g2o::SE3Quat Tcam11_quat = Converter::toSE3Quat(Tcam11);
//    g2o::SE3Quat Tcam21_quat = Converter::toSE3Quat(Tcam21);
    // *********************************************************************

    // Set MapPoint vertices
    // 步骤8：添加3D顶点  //(顶点是优化变量,边是误差项)
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;// 二元边
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;// 二元边
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    // 添加顶点：MapPoint
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();
//        const map<KeyFrame*,size_t> observations = pMP->GetObservations_cam1();//todo cam1

        //Set edges
        // 步骤9：对每一对关联的MapPoint和KeyFrame构建边
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            int cam = pKFi->keypoint_to_cam.find(mit->second)->second;
//            cout<<"cam = "<<cam<<endl;

            if(!pKFi->isBad())
            {
//                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn_total[mit->second];
//                cout<<"为i点构建优化的边,i= "<<mit->second<<endl;
//                const cv::KeyPoint &kpUn = pKFi->mvKeysUn_total[mit->second];

                // Monocular observation 单目
                if(pKFi->mvuRight_total[mit->second]<0) //_total
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    // 设置第一个顶点
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    // 设置第二个顶点
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    // 信息矩阵: 协方差矩阵之逆. 每条边需设定一个信息矩阵作为不确定性的度量
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    //设置鲁棒核函数
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    // 设置相机内参
                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
//                    cout<<"cam = "<<cam<<endl;
                    e->Tcim_quat = Tcim[cam]; //todo 修改
//                    switch (cam){
//                        case 0:
//                            e->Tcim_quat = Tcam11_quat;
//                            break;
//                        case 1:
//                            e->Tcim_quat = Tcam21_quat;
//                            break;
//                        default:
//                            break;
//                    }


                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation //双目和rgbd
                {  //双目和RGBD
                    Eigen::Matrix<double,3,1> obs;
//                    const float kp_ur = pKFi->mvuRight[mit->second];
                    const float kp_ur = pKFi->mvuRight_total[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    //octave是特征点提取时的金字塔层数
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];//该帧的该特征点所在层的尺度因子的平方的逆
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;//Identity()是单位矩阵
                    e->setInformation(Info); //信息矩阵: 协方差矩阵之逆

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;
                    e->Tcim_quat = Tcim[cam]; //todo 修改

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag) //是否停止优化的标志
        if(*pbStopFlag)
            return;

    // 步骤9：开始优化
    //开始优化，迭代一次优化5次。
    // 然后根据卡方分布筛选阈值（这个阈值见多视图几何），去除野值后再优化。
    // 然后去除误差比较大的KF和地图点的互相观测。最后更新优化后的位姿和地图点。
    optimizer.initializeOptimization();
    optimizer.optimize(5);//迭代一次优化5次?

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    // 步骤10：检测outlier，并设置下次不优化

    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);// 不优化
        }

        e->setRobustKernel(0);// 不使用核函数
    }

    // 优化完成后，进行Edge的检查
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];//遍历边edges
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        // 第二个判断条件，用于检查构成该边的MapPoint在该相机坐标系下的深度是否为正？
        if(e->chi2()>7.815 || !e->isDepthPositive())//chi2卡方检验 : error*\Omega*error
        {
            e->setLevel(1);// 不优化
        }

        // 因为剔除了错误的边，所以下次优化不再使用核函数
        e->setRobustKernel(0);
    }

    // Optimize again without the outliers
    // 步骤11：排除误差较大的outlier后再次优化

//    cout << "Optimizer.cc::L835: 第一次优化并剔除outliers后现在地图中总共有 <" << pMap->MapPointsInMap() << "> 个地图点" << endl;
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);//todo plc :原来是10

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    // 步骤12：在优化后重新计算误差，剔除连接误差比较大的关键帧和MapPoint
    //单目
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    //双目/RGBD
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // 连接偏差比较大，在关键帧中剔除对该MapPoint的观测
    // 连接偏差比较大，在MapPoint中剔除对该关键帧的观测
    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
//    cout << "Optimizer.cc::L894: 第二次优化后现在地图中总共有 <" << pMap->MapPointsInMap() << "> 个地图点" << endl;

    // Recover optimized data
    // 步骤13：优化后更新关键帧位姿以及MapPoints的位置、平均观测方向等属性

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
    // note 计算ba时间
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tlba= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"局部ba成功，花费时间为： "<<tlba<<endl;
//    cout << "Optimizer.cc::L894: 第二次优化并更新位姿及地图点后现在地图中总共有 <" << pMap->MapPointsInMap() << "> 个地图点" << endl;
}

/**
 * @brief 闭环检测后，EssentialGraph优化
 *
 * 1. Vertex:
 *     - g2o::VertexSim3Expmap，Essential graph中关键帧的位姿
 * 2. Edge:
 *     - g2o::EdgeSim3()，BaseBinaryEdge
 *         + Vertex：关键帧的Tcw，MapPoint的Pw
 *         + measurement：经过CorrectLoop函数步骤2，Sim3传播校正后的位姿
 *         + InfoMatrix: 单位矩阵
 *
 * @param pMap               全局地图
 * @param pLoopKF            闭环匹配上的关键帧
 * @param pCurKF             当前关键帧
 * @param NonCorrectedSim3   未经过Sim3传播调整过的关键帧位姿
 * @param CorrectedSim3      经过Sim3传播调整过的关键帧位姿
 * @param LoopConnections    因闭环时MapPoints调整而新生成的边
 */  //闭环
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    cout<<"Optimizer.cc L1042::OptimizeEssentialGraph()优化本质矩阵"<<endl;
    // Setup optimizer
    // 步骤1：构造优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    // 指定线性方程求解器使用Eigen的块求解器
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    // 构造线性求解器
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    // 使用LM算法进行非线性迭代
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    //得到所有关键帧和地图点
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints(); //只用于优化后的地图点调整

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    //创建未矫正和已矫正的相似变换
    // 仅经过Sim3传播调整，未经过优化的keyframe的pose
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    // 经过Sim3传播调整，经过优化的keyframe的pose
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    // 这个变量没有用
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    //最小权重??
    const int minFeat = 100;

    // Set KeyFrame vertices
    // 步骤2：将地图中所有keyframe的pose作为顶点添加到优化器
    // 尽可能使用经过Sim3调整的位姿
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap(); //关键帧sim3顶点, 深度相机顶点尺度固定

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        // 如果该关键帧在闭环时通过Sim3传播调整过，用校正后的位姿
        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else// 如果该关键帧在闭环时没有通过Sim3传播调整过，用自身的位姿
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        // 闭环匹配上的帧不进行位姿优化
        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale; // stereo/rgbd 的尺度固定

        optimizer.addVertex(VSim3);

        // 优化前的pose顶点，后面代码中没有使用
        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    // 步骤3：添加边：LoopConnections是闭环时因为MapPoints调整而出现的新关键帧连接关系（不是当前帧与闭环匹配帧之间的连接关系）
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi]; //没有经过Sim3传播调整过
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight_cam1(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            // 得到两个pose间的Sim3变换
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            //顶点是闭环后出现的新的关键帧的连接关系
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            // 根据两个Pose顶点的位姿算出相对位姿作为边，那还存在误差？优化有用？因为闭环MapPoints调整新形成的边不优化？（wubo???）
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    // 步骤4：添加跟踪时形成的边、闭环匹配成功形成的边
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        // 尽可能得到未经过Sim3传播调整的位姿
        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        /////////////////////
        // 步骤4.1：只添加扩展树的边（有父关键帧）
        /////////////////////
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            // 尽可能得到未经过Sim3传播调整的位姿
            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        /////////////////////
        // 步骤4.2：添加在CorrectLoop函数中AddLoopEdge函数添加的闭环连接边（当前帧与闭环匹配帧之间的连接关系）
        /////////////////////
        // 使用经过Sim3调整前关键帧之间的相对关系作为边
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                // 尽可能得到未经过Sim3传播调整的位姿
                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                // 根据两个Pose顶点的位姿算出相对位姿作为边，那还存在误差？优化有用？（wubo???）
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        /////////////////////
        // 步骤4.3：最有很好共视关系的关键帧也作为边进行优化
        /////////////////////
        // 使用经过Sim3调整前关键帧之间的相对关系作为边
//        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight_cam1(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    // 尽可能得到未经过Sim3传播调整的位姿
                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    // 步骤5：开始g2o优化
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    // 步骤6：设定优化后的位姿
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    // 步骤7：步骤5和步骤6优化得到关键帧的位姿后，MapPoints根据参考帧优化前后的相对关系调整自己的位置
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame(); //todo 这里没问题吗
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

/**
 * @brief 形成闭环时进行Sim3优化
 *
 * 1. Vertex:
 *     - g2o::VertexSim3Expmap()，两个关键帧的位姿
 *     - g2o::VertexSBAPointXYZ()，两个关键帧共有的MapPoints
 * 2. Edge:
 *     - g2o::EdgeSim3ProjectXYZ()，BaseBinaryEdge
 *         + Vertex：关键帧的Sim3，MapPoint的Pw
 *         + measurement：MapPoint在关键帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *     - g2o::EdgeInverseSim3ProjectXYZ()，BaseBinaryEdge
 *         + Vertex：关键帧的Sim3，MapPoint的Pw
 *         + measurement：MapPoint在关键帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 * @param pKF1        KeyFrame 当前关键帧
 * @param pKF2        KeyFrame 一个闭环候选帧 mvpEnoughConsistentCandidates[i]
 * @param vpMatches1  两个关键帧的匹配关系,是帧1的地图点中在帧2有匹配的那部分(无匹配的是null)
 * @param g2oS12      两个关键帧间的Sim3变换
 * @param th2         核函数阈值
 * @param bFixScale   是否优化尺度，弹目进行尺度优化，双目不进行尺度优化
 */
//在用RANSAC求解过Sim3，以及通过Sim3匹配更多的地图点后，对当前关键帧，闭环关键帧，
// 以及匹配的地图点进行优化，获得更准确的Sim3位姿，再去下一步的闭环调整。 优化得到的是sim3位姿
int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            const cv::Mat CalibMatrix) // todo 4x3矩阵
{
    cout<<"Optimizer.cc::L1380 OptimizeSim3()... "<<endl;
    // 步骤1：初始化g2o优化器
    // 先构造求解器
    g2o::SparseOptimizer optimizer;
    // 构造线性方程求解器，Hx = -b的求解器
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    // 使用L-M迭代
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    //当前帧的R t
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    //候选帧的R t
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

//    const cv::Mat Rcam12 = (cv::Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0);
//    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//
    cv::Mat tcam12 = cv::Mat_<float>(3,1);
//    const cv::Mat tcam12 = CalibMatrix.row(3).t();
    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
    const cv::Mat Rcam21 = Rcam12.inv();
    const cv::Mat tcam21 = -Rcam21 * tcam12;

    // Set Sim3 vertex
    // 步骤2.1 添加Sim3顶点 // todo 需要 keypoint_to_cam吗
//    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    VertexSim3Expmap_Multi * vSim3 =
            new VertexSim3Expmap_Multi(pKF1->keypoint_to_cam,pKF2->keypoint_to_cam);
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    vSim3->Rcam21 = Converter::toMatrix3d(Rcam21); //新增
    vSim3->tcam21 = Converter::toVector3d(tcam21);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();//大小为kf1的地图点总数
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
//    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
//    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;//pKF2对应的MapPoints到pKF1的投影
    vector<EdgeSim3ProjectXYZ_Multi*> vpEdges12;//pKF2对应的MapPoints到pKF1的投影
    vector<EdgeInverseSim3ProjectXYZ_Multi*> vpEdges21;//pKF1对应的MapPoints到pKF2的投影
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    //遍历kf1匹配点
    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])//i不是匹配点
            continue;

        // pMP1和pMP2是匹配的MapPoints
        MapPoint* pMP1 = vpMapPoints1[i];//kf1的地图点
        MapPoint* pMP2 = vpMatches1[i];//对应匹配的地图点,和上面应该是同一个点

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        //匹配点在帧2的索引
        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);
//        const int i2 = pMP2->GetIndexInKeyFrame_cam1(pKF2);// todo

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                // 步骤2.2 添加PointXYZ顶点
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                //kf1上的点的相机坐标
                cv::Mat P3D1w = pMP1->GetWorldPos();
                //相机坐标
                cv::Mat P3D1c = R1w*P3D1w + t1w;
//                cv::Mat P3D1c;//相机坐标
//                if (i < pKF1->N)
//                {
//                    P3D1c = R1w*P3D1w + t1w;//相机坐标
//                }
//                else if (i >= pKF1->N)
//                {
//                    P3D1c = Rcam21*R1w*P3D1w + Rcam21*t1w +tcam21;//相机坐标 done
//                }
                // 顶点是帧1相机坐标系
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                vPoint1->SetID(i); // TODO 新增 i是在帧1上匹配点的索引
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                //对应匹配点世界坐标
                cv::Mat P3D2w = pMP2->GetWorldPos();
                //相机坐标
                cv::Mat P3D2c = R2w*P3D2w + t2w;
//                cv::Mat P3D2c;//相机坐标
//                if (i2 < pKF2->N)
//                {
//                    P3D2c = R2w*P3D2w + t2w;
//                }
//                else if (i2 >= pKF2->N)
//                {
//                    P3D2c = Rcam21*R2w*P3D2w + Rcam21*t2w +tcam21;//done
//                }
                // 顶点是帧2相机坐标系
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                vPoint2->SetID(i2); // TODO 新增 i2是匹配点在帧2的索引
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn_total[i];
//        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        // 步骤2.3 添加两个顶点（3D点）到相机投影的边
//        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ(); //todo
        EdgeSim3ProjectXYZ_Multi* e12 = new EdgeSim3ProjectXYZ_Multi();
        //这个顶点是pKF2的点的多相机系统坐标
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        // 这个顶点是两个关键帧间的Sim3变换,一个,id=0
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn_total[i2];
//        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        // 添加边
//        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
        EdgeInverseSim3ProjectXYZ_Multi* e21 = new EdgeInverseSim3ProjectXYZ_Multi();

        // 这个顶点是pKF1的点的多相机系统坐标
        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        // 这个顶点是两个关键帧间的Sim3变换,一个,id=0
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    // 步骤3：g2o开始优化，先迭代5次
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    // 步骤4：剔除一些误差大的边
    // Check inliers
    // 进行卡方检验，大于阈值的边剔除
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
//        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
//        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        // 若误差过大
        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            //删除该匹配点(or该地图点?)
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
//            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges12[i]=static_cast<EdgeSim3ProjectXYZ_Multi*>(NULL);
//            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<EdgeInverseSim3ProjectXYZ_Multi*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers
    // 步骤5：再次g2o优化剔除后剩下的边
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
//        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    // 步骤6：得到优化后的结果 //todo 为什么不用 _multi ??
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

//在用RANSAC求解过Sim3，以及通过Sim3匹配更多的地图点后，对当前关键帧，闭环关键帧，
// 以及匹配的地图点进行优化，获得更准确的Sim3位姿，再去下一步的闭环调整。 优化得到的是sim3位姿
int Optimizer::OptimizeSim3_cam1(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    cout<<"Optimizer.cc::L1380 OptimizeSim3()... "<<endl;
    // 步骤1：初始化g2o优化器
    // 先构造求解器
    g2o::SparseOptimizer optimizer;
    // 构造线性方程求解器，Hx = -b的求解器
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    // 使用L-M迭代
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    //当前帧的R t
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    //候选帧的R t
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();


//    const cv::Mat Rcam12 = CalibMatrix.rowRange(0,3).colRange(0,3);//
//    cv::Mat tcam12 = cv::Mat_<float>(3,1);
//
//    tcam12.at<float>(0,0) = CalibMatrix.at<float>(3,0);
//    tcam12.at<float>(1,0) = CalibMatrix.at<float>(3,1);
//    tcam12.at<float>(2,0) = CalibMatrix.at<float>(3,2);
//    const cv::Mat Rcam21 = Rcam12.inv();
//    const cv::Mat tcam21 = -Rcam21 * tcam12;

    // Set Sim3 vertex
    // 步骤2.1 添加Sim3顶点 // todo 需要 keypoint_to_cam吗
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
//    VertexSim3Expmap_Multi * vSim3 =
//            new VertexSim3Expmap_Multi(pKF1->keypoint_to_cam,pKF2->keypoint_to_cam);
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
//    vSim3->Rcam21 = Converter::toMatrix3d(Rcam21); //新增
//    vSim3->tcam21 = Converter::toVector3d(tcam21);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();//大小为kf1的地图点总数
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
//    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches_cam1();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;//pKF2对应的MapPoints到pKF1的投影
//    vector<EdgeSim3ProjectXYZ_Multi*> vpEdges12;//pKF2对应的MapPoints到pKF1的投影
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;//pKF1对应的MapPoints到pKF2的投影
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    //遍历kf1匹配点
    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])//i不是匹配点
            continue;

        // pMP1和pMP2是匹配的MapPoints
        MapPoint* pMP1 = vpMapPoints1[i];//kf1的地图点
        MapPoint* pMP2 = vpMatches1[i];//对应匹配的地图点,和上面应该是同一个点

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        //匹配点在帧2的索引
        const int i2 = pMP2->GetIndexInKeyFrame_cam1(pKF2);
//        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);// todo

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                // 步骤2.2 添加PointXYZ顶点
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                //kf1上的点的相机坐标
                cv::Mat P3D1w = pMP1->GetWorldPos();
                //相机坐标
                cv::Mat P3D1c = R1w*P3D1w + t1w;

                // 顶点是帧1相机坐标系
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                vPoint1->SetID(i); // TODO 新增 i是在帧1上匹配点的索引
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                //对应匹配点世界坐标
                cv::Mat P3D2w = pMP2->GetWorldPos();
                //相机坐标
                cv::Mat P3D2c = R2w*P3D2w + t2w;

                // 顶点是帧2相机坐标系
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                vPoint2->SetID(i2); // TODO 新增 i2是匹配点在帧2的索引
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
//        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn_total[i];
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        // 步骤2.3 添加两个顶点（3D点）到相机投影的边
        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ(); //todo
//        EdgeSim3ProjectXYZ_Multi* e12 = new EdgeSim3ProjectXYZ_Multi();
        //这个顶点是pKF2的点的多相机系统坐标
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        // 这个顶点是两个关键帧间的Sim3变换,一个,id=0
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
//        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn_total[i2];
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        // 添加边
        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
//        EdgeInverseSim3ProjectXYZ_Multi* e21 = new EdgeInverseSim3ProjectXYZ_Multi();

        // 这个顶点是pKF1的点的多相机系统坐标
        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        // 这个顶点是两个关键帧间的Sim3变换,一个,id=0
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    // 步骤3：g2o开始优化，先迭代5次
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    // 步骤4：剔除一些误差大的边
    // Check inliers
    // 进行卡方检验，大于阈值的边剔除
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
//        EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
//        EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        // 若误差过大
        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            //删除该匹配点(or该地图点?)
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
//            vpEdges12[i]=static_cast<EdgeSim3ProjectXYZ_Multi*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
//            vpEdges21[i]=static_cast<EdgeInverseSim3ProjectXYZ_Multi*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers
    // 步骤5：再次g2o优化剔除后剩下的边
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
//        EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
//        EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    // 步骤6：得到优化后的结果 //todo 为什么不用 _multi ??
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace ORB_SLAM
