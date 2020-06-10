//
// Created by pang on 19-7-3.
//

#ifndef ORB_SLAM2_G2O_SIM3_EXPMAP_H
#define ORB_SLAM2_G2O_SIM3_EXPMAP_H


//#include <opencv2/opencv.hpp>
//#include <Eigen/Dense>
//
//#include "g2o/core/robust_kernel_impl.h"
//#include "g2o/types/types_seven_dof_expmap.h"
//
//#include <Eigen/StdVector>
//
//#include "misc.h"
//#include "cam_model_omni.h"
//#include "cam_system_omni.h"
//#include "cConverter.h"
//#include "g2o_MultiCol_vertices_edges.h"
#include <opencv2/opencv.hpp>
//#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
//#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/types/se3_ops.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <unordered_map>

namespace ORB_SLAM2
{
    using namespace Eigen;
    /**
       * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
       * the parameterization for the increments constructed is a 7d vector
       * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
       */
//class VertexSBAPointXYZ_Multi : public g2o::BaseVertex<3, Vector3d>
//    {
//    public:
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//        VertexSBAPointXYZ_Multi();
//        virtual bool read(std::istream& is);
//        virtual bool write(std::ostream& os) const;
//
//        virtual void setToOriginImpl() {
//            _estimate.fill(0.);
//        }
//
//        virtual void oplusImpl(const double* update)
//        {
//            Eigen::Map<const Vector3d> v(update);
//            _estimate += v;
//        }
//        void SetID(int id) { ptID = id; }
//        int GetID() { return ptID; }
//        int ptID; // 用于 Sim3 optimization
//    };

    // 顶点 用于优化sim3 // 添加了 keypoint_to_cam
    class VertexSim3Expmap_Multi : public g2o::BaseVertex<7, g2o::Sim3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSim3Expmap_Multi() {}
        VertexSim3Expmap_Multi(
                std::unordered_map<size_t, int>& kp_to_cam1,
                std::unordered_map<size_t, int>& kp_to_cam2);

        virtual void setToOriginImpl() {
            _estimate = g2o::Sim3();
        }

        virtual void oplusImpl(const double* update_)
        {
            Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

            if (_fix_scale)
                update[6] = 0;

            g2o::Sim3 s(update);
            setEstimate(s*estimate());
        }

        Vector2d _principle_point1, _principle_point2;
        Vector2d _focal_length1, _focal_length2;

        Matrix3d Rcam21;
        Vector3d tcam21;

        // mcs坐标转到对应相机像素坐标
        Vector2d cam_map1(const Vector3d & v, int ptIdx) const
        {
            Vector2d res;
            int camIdx = keypoint_to_cam1.find(ptIdx)->second;
            Vector3d v3dc = Rcam21 * v + tcam21; // 转到对应相机 todo 要加入标定矩阵
            Vector2d v2 = g2o::project(v3dc); // 投影到归一化平面

            // 投影到像素坐标
            res[0] = v2[0]*_focal_length1[0] + _principle_point1[0];
            res[1] = v2[1]*_focal_length1[1] + _principle_point1[1];
            return res;
        }

        Vector2d cam_map2(const Vector3d & v, int ptIdx) const
        {
            Vector2d res;
            int camIdx = keypoint_to_cam2.find(ptIdx)->second;
            Vector3d v3dc = Rcam21 * v + tcam21; // 转到对应相机 todo 要加入标定矩阵
            Vector2d v2 = g2o::project(v3dc); // 投影到归一化平面

            // 投影到像素坐标
            res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
            res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
            return res;
        }

        bool _fix_scale;

        bool read(std::istream& is)
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }

        bool write(std::ostream& os) const
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }

        std::unordered_map<size_t, int> keypoint_to_cam1;
        std::unordered_map<size_t, int> keypoint_to_cam2;
    };



    // 第一条边 用于优化Sim3
    class EdgeSim3ProjectXYZ_Multi : public g2o::BaseBinaryEdge<2, Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap_Multi>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSim3ProjectXYZ_Multi() {}

        void computeError()
        {
            const VertexSim3Expmap_Multi* v1 = static_cast<const VertexSim3Expmap_Multi*>(_vertices[1]);
//            const VertexSBAPointXYZ_Multi* v2 = static_cast<const VertexSBAPointXYZ_Multi*>(_vertices[0]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

            Vector2d obs(_measurement);
            Vector3d est = v2->estimate();
            Vector2d m = v1->cam_map1(
                    v1->estimate().map(est), //pKF2的点投影到pKF1 mcs坐标系
                    v2->ptID);
            _error = obs - m;
        }
        bool read(std::istream& is)
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }

        bool write(std::ostream& os) const
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }
    };
    /**/
    // 另一条边 用于优化Sim3
    class EdgeInverseSim3ProjectXYZ_Multi :
            public g2o::BaseBinaryEdge<2, Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap_Multi>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeInverseSim3ProjectXYZ_Multi() {}

        void computeError()
        {
            const VertexSim3Expmap_Multi* v1 = static_cast<const VertexSim3Expmap_Multi*>(_vertices[1]);
//            const VertexSBAPointXYZ_Multi* v2 = static_cast<const VertexSBAPointXYZ_Multi*>(_vertices[0]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

            Vector2d obs(_measurement);
            Vector3d est = v2->estimate();
            Vector2d m = v1->cam_map1(
                    v1->estimate().map(est), //pKF1的点投影到pKF2 mcs坐标系
                    v2->ptID);
            _error = obs - m;
        }
        bool read(std::istream& is)
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }

        bool write(std::ostream& os) const
        {
            std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
            return false;
        }
    };

}

#endif //ORB_SLAM2_G2O_SIM3_EXPMAP_H
