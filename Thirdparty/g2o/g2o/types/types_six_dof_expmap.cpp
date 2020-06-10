// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;


Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate()); //Tmw
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci

  //Pm
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
//  double z_2 = z*z;

  //Pci
  double xc = pt3_ci[0];
  double yc = pt3_ci[1];
  double zc = pt3_ci[2];
  double zc_2 = zc*zc;

//  Matrix<double,2,3> tmp;
//  tmp(0,0) = fx;
//  tmp(0,1) = 0;
//  tmp(0,2) = -x/z*fx;
//
//  tmp(1,0) = 0;
//  tmp(1,1) = fy;
//  tmp(1,2) = -y/z*fy;

  const Matrix3d R =  T.rotation().toRotationMatrix(); //Rmw

  double rm00 = R(0,0); //Rmw
  double rm01 = R(0,1);
  double rm02 = R(0,2);
  double rm10 = R(1,0);
  double rm11 = R(1,1);
  double rm12 = R(1,2);
  double rm20 = R(2,0);
  double rm21 = R(2,1);
  double rm22 = R(2,2);

  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
  const Matrix3d Rcim = Tcim.block(0,0,3,3);

  double r00 = Rcim(0,0);
  double r01 = Rcim(0,1);
  double r02 = Rcim(0,2);
  double r10 = Rcim(1,0);
  double r11 = Rcim(1,1);
  double r12 = Rcim(1,2);
  double r20 = Rcim(2,0);
  double r21 = Rcim(2,1);
  double r22 = Rcim(2,2);


  Eigen::Matrix3d tmp1;
  //tmp1 e对相机坐标Pci的偏导 δe/δPc
  tmp1(0,0) = -fx/zc;
  tmp1(0,1) = 0;
  tmp1(0,2) = fx*xc/zc_2;

  tmp1(1,0) = 0;
  tmp1(1,1) = -fy/zc;
  tmp1(1,2) = fy*yc/zc_2;

  Eigen::Matrix3d tmp2;
  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 δe/δPc * Rcim )
  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;

  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;

  //todo 以下6行是第一个雅可比矩阵, ∂e/∂Pw = ∂e/∂Pc * Rcim * Rmw  (即tmp2 * Rmw)
  _jacobianOplusXi(0,0) = tmp2(0,0)*rm00+tmp2(0,1)*rm10+tmp2(0,2)*rm20;
  _jacobianOplusXi(0,1) = tmp2(0,0)*rm01+tmp2(0,1)*rm11+tmp2(0,2)*rm21;
  _jacobianOplusXi(0,2) = tmp2(0,0)*rm02+tmp2(0,1)*rm12+tmp2(0,2)*rm22;

  _jacobianOplusXi(1,0) = tmp2(1,0)*rm00+tmp2(1,1)*rm10+tmp2(1,2)*rm20;
  _jacobianOplusXi(1,1) = tmp2(1,0)*rm01+tmp2(1,1)*rm11+tmp2(1,2)*rm21;
  _jacobianOplusXi(1,2) = tmp2(1,0)*rm02+tmp2(1,1)*rm12+tmp2(1,2)*rm22;

  //todo 以下12行是第二个雅可比矩阵, ∂e/∂δξ = ∂e/∂Pc * Rcim * ∂Pm/∂δξ
  _jacobianOplusXj(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
  _jacobianOplusXj(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
  _jacobianOplusXj(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
  _jacobianOplusXj(0,3) = tmp2(0,0);
  _jacobianOplusXj(0,4) = tmp2(0,1);
  _jacobianOplusXj(0,5) = tmp2(0,2);

  _jacobianOplusXj(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
  _jacobianOplusXj(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
  _jacobianOplusXj(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
  _jacobianOplusXj(1,3) = tmp2(1,0);
  _jacobianOplusXj(1,4) = tmp2(1,1);
  _jacobianOplusXj(1,5) = tmp2(1,2);

//  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();
//
//  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
//  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
//  _jacobianOplusXj(0,2) = y/z *fx;
//  _jacobianOplusXj(0,3) = -1./z *fx;
//  _jacobianOplusXj(0,4) = 0;
//  _jacobianOplusXj(0,5) = x/z_2 *fx;
//
//  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
//  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
//  _jacobianOplusXj(1,2) = -x/z *fy;
//  _jacobianOplusXj(1,3) = 0;
//  _jacobianOplusXj(1,4) = -1./z *fy;
//  _jacobianOplusXj(1,5) = y/z_2 *fy;
}

//相机坐标投影到像素坐标
Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}


//相机坐标投影到像素坐标
Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeStereoSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

//原函数
//void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
//  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
//  SE3Quat T(vj->estimate());
//  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
//  Vector3d xyz = vi->estimate();
//  Vector3d xyz_trans = T.map(xyz);
//
//  const Matrix3d R =  T.rotation().toRotationMatrix();
//
//  double x = xyz_trans[0];
//  double y = xyz_trans[1];
//  double z = xyz_trans[2];
//  double z_2 = z*z;
//
//  _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
//  _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
//  _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;
//
//  _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
//  _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
//  _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;
//
//  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
//  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
//  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;
//
//  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
//  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
//  _jacobianOplusXj(0,2) = y/z *fx;
//  _jacobianOplusXj(0,3) = -1./z *fx;
//  _jacobianOplusXj(0,4) = 0;
//  _jacobianOplusXj(0,5) = x/z_2 *fx;
//
//  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
//  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
//  _jacobianOplusXj(1,2) = -x/z *fy;
//  _jacobianOplusXj(1,3) = 0;
//  _jacobianOplusXj(1,4) = -1./z *fy;
//  _jacobianOplusXj(1,5) = y/z_2 *fy;
//
//  _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
//  _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
//  _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
//  _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
//  _jacobianOplusXj(2,4) = 0;
//  _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
//}
// todo 改写的
void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate()); //Tmw
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz); //Pm
  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci

  const Matrix3d R =  T.rotation().toRotationMatrix(); //Rmw

  double rm00 = R(0,0); //Rmw
  double rm01 = R(0,1);
  double rm02 = R(0,2);
  double rm10 = R(1,0);
  double rm11 = R(1,1);
  double rm12 = R(1,2);
  double rm20 = R(2,0);
  double rm21 = R(2,1);
  double rm22 = R(2,2);

  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
  const Matrix3d Rcim = Tcim.block(0,0,3,3);

  double r00 = Rcim(0,0);
  double r01 = Rcim(0,1);
  double r02 = Rcim(0,2);
  double r10 = Rcim(1,0);
  double r11 = Rcim(1,1);
  double r12 = Rcim(1,2);
  double r20 = Rcim(2,0);
  double r21 = Rcim(2,1);
  double r22 = Rcim(2,2);

  //Pm
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
//  double z_2 = z*z;

  //Pci
  double xc = pt3_ci[0];
  double yc = pt3_ci[1];
  double zc = pt3_ci[2];
  double zc_2 = zc*zc;

  double a1=bf/zc_2;

  Eigen::Matrix3d tmp1;
  //tmp1 e对相机坐标Pci的偏导 ∂e/∂Pc
  //note 双目/rgbd增加了 ur对Pc(x y z)的偏导 (最后三行)
  tmp1(0,0) = -fx/zc;
  tmp1(0,1) = 0;
  tmp1(0,2) = fx*xc/zc_2;

  tmp1(1,0) = 0;
  tmp1(1,1) = -fy/zc;
  tmp1(1,2) = fy*yc/zc_2;

  tmp1(2,0) = tmp1(0,0);
  tmp1(2,1) = 0;
  tmp1(2,2) = tmp1(0,2)-a1;

  Eigen::Matrix3d tmp2;
  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 δe/δPc * Rcim )
  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;

  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;

  tmp2(2,0) = tmp2(0,0) - a1*r20;
  tmp2(2,1) = tmp2(0,1) - a1*r21;
  tmp2(2,2) = tmp2(0,2) - a1*r22;

  //todo 以下9行是第一个雅可比矩阵, ∂e/∂Pw = ∂e/∂Pc * Rcim * Rmw  (即tmp2 * Rmw)
  _jacobianOplusXi(0,0) = tmp2(0,0)*rm00+tmp2(0,1)*rm10+tmp2(0,2)*rm20;
  _jacobianOplusXi(0,1) = tmp2(0,0)*rm01+tmp2(0,1)*rm11+tmp2(0,2)*rm21;
  _jacobianOplusXi(0,2) = tmp2(0,0)*rm02+tmp2(0,1)*rm12+tmp2(0,2)*rm22;

  _jacobianOplusXi(1,0) = tmp2(1,0)*rm00+tmp2(1,1)*rm10+tmp2(1,2)*rm20;
  _jacobianOplusXi(1,1) = tmp2(1,0)*rm01+tmp2(1,1)*rm11+tmp2(1,2)*rm21;
  _jacobianOplusXi(1,2) = tmp2(1,0)*rm02+tmp2(1,1)*rm12+tmp2(1,2)*rm22;

  _jacobianOplusXi(2,0) = tmp2(2,0)*rm00+tmp2(2,1)*rm10+tmp2(2,2)*rm20;
  _jacobianOplusXi(2,1) = tmp2(2,0)*rm01+tmp2(2,1)*rm11+tmp2(2,2)*rm21;
  _jacobianOplusXi(2,2) = tmp2(2,0)*rm02+tmp2(2,1)*rm12+tmp2(2,2)*rm22;

//  _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
//  _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
//  _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;
//
//  _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
//  _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
//  _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;
//
//  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
//  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
//  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

  //todo 以下18行是第二个雅可比矩阵, ∂e/∂δξ = ∂e/∂Pc * Rcim * ∂Pm/∂δξ
  _jacobianOplusXj(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
  _jacobianOplusXj(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
  _jacobianOplusXj(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
  _jacobianOplusXj(0,3) = tmp2(0,0);
  _jacobianOplusXj(0,4) = tmp2(0,1);
  _jacobianOplusXj(0,5) = tmp2(0,2);

  _jacobianOplusXj(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
  _jacobianOplusXj(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
  _jacobianOplusXj(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
  _jacobianOplusXj(1,3) = tmp2(1,0);
  _jacobianOplusXj(1,4) = tmp2(1,1);
  _jacobianOplusXj(1,5) = tmp2(1,2);

  _jacobianOplusXj(2,0) = -tmp2(2,1)*z+tmp2(2,2)*y;
  _jacobianOplusXj(2,1) = tmp2(2,0)*z-tmp2(2,2)*x;
  _jacobianOplusXj(2,2) = -tmp2(2,0)*y+tmp2(2,1)*x;
  _jacobianOplusXj(2,3) = tmp2(2,0);
  _jacobianOplusXj(2,4) = tmp2(2,1);
  _jacobianOplusXj(2,5) = tmp2(2,2);

//  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
//  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
//  _jacobianOplusXj(0,2) = y/z *fx;
//  _jacobianOplusXj(0,3) = -1./z *fx;
//  _jacobianOplusXj(0,4) = 0;
//  _jacobianOplusXj(0,5) = x/z_2 *fx;
//
//  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
//  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
//  _jacobianOplusXj(1,2) = -x/z *fy;
//  _jacobianOplusXj(1,3) = 0;
//  _jacobianOplusXj(1,4) = -1./z *fy;
//  _jacobianOplusXj(1,5) = y/z_2 *fy;
//
//  _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
//  _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
//  _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
//  _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
//  _jacobianOplusXj(2,4) = 0;
//  _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
}

//Only Pose
// 以下仅用于位姿优化？====================================================

//仅位姿，单目
bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


//单相机 单目
void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;
}

// NOTE 修改后的多相机 单目
//void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
//  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//  Vector3d xyz_trans = vi->estimate().map(Xw);
//
//  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci
//
//  //Pm
//  double x = xyz_trans[0];
//  double y = xyz_trans[1];
//  double z = xyz_trans[2];
////    double z_2 = z*z;
////    double invz = 1.0/xyz_trans[2];
////    double invz_2 = invz*invz;
//
//  //Pci
//  double xc = pt3_ci[0];
//  double yc = pt3_ci[1];
//  double zc = pt3_ci[2];
//  double zc_2 = zc*zc;
//
//  // Rcim
//  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
//  const Matrix3d Rcim = Tcim.block(0,0,3,3);
//
//  double r00 = Rcim(0,0);
//  double r01 = Rcim(0,1);
//  double r02 = Rcim(0,2);
//  double r10 = Rcim(1,0);
//  double r11 = Rcim(1,1);
//  double r12 = Rcim(1,2);
//  double r20 = Rcim(2,0);
//  double r21 = Rcim(2,1);
//  double r22 = Rcim(2,2);
//
//  Eigen::Matrix3d tmp1;
//  //tmp1 e对相机坐标Pci的偏导 ∂e/∂Pc
//  tmp1(0,0) = -fx/zc;
//  tmp1(0,1) = 0;
//  tmp1(0,2) = fx*xc/zc_2;
//
//  tmp1(1,0) = 0;
//  tmp1(1,1) = -fy/zc;
//  tmp1(1,2) = fy*yc/zc_2;
//
//  Eigen::Matrix3d tmp2;
//  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 ∂e/∂Pc * Rcim )
//  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
//  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
//  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;
//
//  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
//  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
//  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;
//
//  //todo 雅可比矩阵, ∂e/∂δξ = ∂e/∂Pc * Rcim * ∂Pm/∂δξ
//  _jacobianOplusXi(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
//  _jacobianOplusXi(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
//  _jacobianOplusXi(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
//  _jacobianOplusXi(0,3) = tmp2(0,0);
//  _jacobianOplusXi(0,4) = tmp2(0,1);
//  _jacobianOplusXi(0,5) = tmp2(0,2);
//
//  _jacobianOplusXi(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
//  _jacobianOplusXi(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
//  _jacobianOplusXi(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
//  _jacobianOplusXi(1,3) = tmp2(1,0);
//  _jacobianOplusXi(1,4) = tmp2(1,1);
//  _jacobianOplusXi(1,5) = tmp2(1,2);
//}

//相机坐标投影到像素坐标
Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

// 仅位姿，单目 Note 多相机
bool EdgeSE3ProjectXYZOnlyPose_multi::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZOnlyPose_multi::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

//单相机 单目
void EdgeSE3ProjectXYZOnlyPose_multi::linearizeOplus() {

  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci

  //Pm
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
//    double z_2 = z*z;
//    double invz = 1.0/xyz_trans[2];
//    double invz_2 = invz*invz;

  //Pci
  double xc = pt3_ci[0];
  double yc = pt3_ci[1];
  double zc = pt3_ci[2];
  double zc_2 = zc*zc;

  // Rcim
  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
  const Matrix3d Rcim = Tcim.block(0,0,3,3);

  double r00 = Rcim(0,0);
  double r01 = Rcim(0,1);
  double r02 = Rcim(0,2);
  double r10 = Rcim(1,0);
  double r11 = Rcim(1,1);
  double r12 = Rcim(1,2);
  double r20 = Rcim(2,0);
  double r21 = Rcim(2,1);
  double r22 = Rcim(2,2);

  Eigen::Matrix3d tmp1;
  //tmp1 e对相机坐标Pci的偏导 ∂e/∂Pc
  tmp1(0,0) = -fx/zc;
  tmp1(0,1) = 0;
  tmp1(0,2) = fx*xc/zc_2;

  tmp1(1,0) = 0;
  tmp1(1,1) = -fy/zc;
  tmp1(1,2) = fy*yc/zc_2;

  Eigen::Matrix3d tmp2;
  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 ∂e/∂Pc * Rcim )
  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;

  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;

  //todo 雅可比矩阵, ∂e/∂δξ = ∂e/∂Pc * Rcim * ∂Pm/∂δξ
  _jacobianOplusXi(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
  _jacobianOplusXi(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
  _jacobianOplusXi(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
  _jacobianOplusXi(0,3) = tmp2(0,0);
  _jacobianOplusXi(0,4) = tmp2(0,1);
  _jacobianOplusXi(0,5) = tmp2(0,2);

  _jacobianOplusXi(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
  _jacobianOplusXi(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
  _jacobianOplusXi(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
  _jacobianOplusXi(1,3) = tmp2(1,0);
  _jacobianOplusXi(1,4) = tmp2(1,1);
  _jacobianOplusXi(1,5) = tmp2(1,2);
}
//相机坐标投影到像素坐标
Vector2d EdgeSE3ProjectXYZOnlyPose_multi::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0]*fx + cx;
    res[1] = proj[1]*fy + cy;
    return res;
}

//仅位姿，双目 rgbd　note 单相机
//相机坐标投影到像素坐标
Vector3d EdgeStereoSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}


bool EdgeStereoSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

// note 修改后的 多相机 双目/rgbd
//void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
//  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//  Vector3d xyz_trans = vi->estimate().map(Xw);
//  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci
//
//  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
//  const Matrix3d Rcim = Tcim.block(0,0,3,3);
//  double r00 = Rcim(0,0);
//  double r01 = Rcim(0,1);
//  double r02 = Rcim(0,2);
//  double r10 = Rcim(1,0);
//  double r11 = Rcim(1,1);
//  double r12 = Rcim(1,2);
//  double r20 = Rcim(2,0);
//  double r21 = Rcim(2,1);
//  double r22 = Rcim(2,2);
//
//  //Pm
//  double x = xyz_trans[0];
//  double y = xyz_trans[1];
//  double z = xyz_trans[2];
////  double z_2 = z*z;
////  double invz = 1.0/xyz_trans[2];
////  double invz_2 = invz*invz;
//
//  //Pci
//  double xc = pt3_ci[0];
//  double yc = pt3_ci[1];
//  double zc = pt3_ci[2];
//  double zc_2 = zc*zc;
//
//  double a1=bf/zc_2;
//
//  //tmp1 e对相机坐标Pci的偏导 δe/δPc
//  //note 双目/rgbd增加了 ur对Pc(x y z)的偏导 (最后三行)
//  Eigen::Matrix3d tmp1;
//  tmp1(0,0) = -fx/zc;
//  tmp1(0,1) = 0;
//  tmp1(0,2) = fx*xc/zc_2;
//
//  tmp1(1,0) = 0;
//  tmp1(1,1) = -fy/zc;
//  tmp1(1,2) = fy*yc/zc_2;
//
//  tmp1(2,0) = tmp1(0,0);
//  tmp1(2,1) = 0;
//  tmp1(2,2) = tmp1(0,2)-a1;
//
//  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 δe/δPc * Rcim )
//  Eigen::Matrix3d tmp2;
//  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
//  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
//  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;
//
//  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
//  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
//  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;
//
//  tmp2(2,0) = tmp2(0,0) - a1*r20;
//  tmp2(2,1) = tmp2(0,1) - a1*r21;
//  tmp2(2,2) = tmp2(0,2) - a1*r22;
//
//  //todo 以下18行是第二个雅可比矩阵, δe/δPw = δe/δPc * Rcim * Rmw  (即tmp2 * Rmw)
//  _jacobianOplusXi(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
//  _jacobianOplusXi(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
//  _jacobianOplusXi(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
//  _jacobianOplusXi(0,3) = tmp2(0,0);
//  _jacobianOplusXi(0,4) = tmp2(0,1);
//  _jacobianOplusXi(0,5) = tmp2(0,2);
//
//  _jacobianOplusXi(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
//  _jacobianOplusXi(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
//  _jacobianOplusXi(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
//  _jacobianOplusXi(1,3) = tmp2(1,0);
//  _jacobianOplusXi(1,4) = tmp2(1,1);
//  _jacobianOplusXi(1,5) = tmp2(1,2);
//
//  _jacobianOplusXi(2,0) = -tmp2(2,1)*z+tmp2(2,2)*y;
//  _jacobianOplusXi(2,1) = tmp2(2,0)*z-tmp2(2,2)*x;
//  _jacobianOplusXi(2,2) = -tmp2(2,0)*y+tmp2(2,1)*x;
//  _jacobianOplusXi(2,3) = tmp2(2,0);
//  _jacobianOplusXi(2,4) = tmp2(2,1);
//  _jacobianOplusXi(2,5) = tmp2(2,2);
//
//}

  //原来的
//  bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
//    for (int i=0; i<2; i++){
//      is >> _measurement[i];
//    }
//    for (int i=0; i<2; i++)
//      for (int j=i; j<2; j++) {
//        is >> information()(i,j);
//        if (i!=j)
//          information()(j,i)=information()(i,j);
//      }
//    return true;
//  }
//
//bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {
//
//  for (int i=0; i<2; i++){
//    os << measurement()[i] << " ";
//  }
//
//  for (int i=0; i<2; i++)
//    for (int j=i; j<2; j++){
//      os << " " <<  information()(i,j);
//    }
//  return os.good();
//}

//void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
//  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//  Vector3d xyz_trans = vi->estimate().map(Xw);
//
//  double x = xyz_trans[0];
//  double y = xyz_trans[1];
//  double invz = 1.0/xyz_trans[2];
//  double invz_2 = invz*invz;
//
//  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
//  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
//  _jacobianOplusXi(0,2) = y*invz *fx;
//  _jacobianOplusXi(0,3) = -invz *fx;
//  _jacobianOplusXi(0,4) = 0;
//  _jacobianOplusXi(0,5) = x*invz_2 *fx;
//
//  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
//  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
//  _jacobianOplusXi(1,2) = -x*invz *fy;
//  _jacobianOplusXi(1,3) = 0;
//  _jacobianOplusXi(1,4) = -invz *fy;
//  _jacobianOplusXi(1,5) = y*invz_2 *fy;
//}

//NOTE 原来的一元边 双目 rgbd
void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;

  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
  _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
  _jacobianOplusXi(2,4) = 0;
  _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
}

//仅位姿，双目 rgbd　note 多相机 =================================
//相机坐标投影到像素坐标
Vector3d EdgeStereoSE3ProjectXYZOnlyPose_multi::cam_project(const Vector3d & trans_xyz) const{
    const float invz = 1.0f/trans_xyz[2];
    Vector3d res;
    res[0] = trans_xyz[0]*invz*fx + cx;
    res[1] = trans_xyz[1]*invz*fy + cy;
    res[2] = res[0] - bf*invz;
    return res;
}

bool EdgeStereoSE3ProjectXYZOnlyPose_multi::read(std::istream& is){
    for (int i=0; i<=3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<=2; i++)
        for (int j=i; j<=2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeStereoSE3ProjectXYZOnlyPose_multi::write(std::ostream& os) const {

    for (int i=0; i<=3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<=2; i++)
        for (int j=i; j<=2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

// note 修改后的 多相机 双目/rgbd
void EdgeStereoSE3ProjectXYZOnlyPose_multi::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);
  Vector3d pt3_ci = Tcim_quat.map(xyz_trans); //Pci

  Matrix4d Tcim = Tcim_quat.to_homogeneous_matrix();
  const Matrix3d Rcim = Tcim.block(0,0,3,3);
  double r00 = Rcim(0,0);
  double r01 = Rcim(0,1);
  double r02 = Rcim(0,2);
  double r10 = Rcim(1,0);
  double r11 = Rcim(1,1);
  double r12 = Rcim(1,2);
  double r20 = Rcim(2,0);
  double r21 = Rcim(2,1);
  double r22 = Rcim(2,2);

  //Pm
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
//  double z_2 = z*z;
//  double invz = 1.0/xyz_trans[2];
//  double invz_2 = invz*invz;

  //Pci
  double xc = pt3_ci[0];
  double yc = pt3_ci[1];
  double zc = pt3_ci[2];
  double zc_2 = zc*zc;

  double a1=bf/zc_2;

  //tmp1 e对相机坐标Pci的偏导 δe/δPc
  //note 双目/rgbd增加了 ur对Pc(x y z)的偏导 (最后三行)
  Eigen::Matrix3d tmp1;
  tmp1(0,0) = -fx/zc;
  tmp1(0,1) = 0;
  tmp1(0,2) = fx*xc/zc_2;

  tmp1(1,0) = 0;
  tmp1(1,1) = -fy/zc;
  tmp1(1,2) = fy*yc/zc_2;

  tmp1(2,0) = tmp1(0,0);
  tmp1(2,1) = 0;
  tmp1(2,2) = tmp1(0,2)-a1;

  //tmp2是 e对Pci的偏导 乘 相机坐标点对多相机系统坐标点的偏导 (即 δe/δPc * Rcim )
  Eigen::Matrix3d tmp2;
  tmp2(0,0) = tmp1(0,0)*r00+tmp1(0,2)*r20;
  tmp2(0,1) = tmp1(0,0)*r01+tmp1(0,2)*r21;
  tmp2(0,2) = tmp1(0,0)*r02+tmp1(0,2)*r22;

  tmp2(1,0) = tmp1(1,1)*r10+tmp1(1,2)*r20;
  tmp2(1,1) = tmp1(1,1)*r11+tmp1(1,2)*r21;
  tmp2(1,2) = tmp1(1,1)*r12+tmp1(1,2)*r22;

  tmp2(2,0) = tmp2(0,0) - a1*r20;
  tmp2(2,1) = tmp2(0,1) - a1*r21;
  tmp2(2,2) = tmp2(0,2) - a1*r22;

  //todo 以下18行是第二个雅可比矩阵, δe/δPw = δe/δPc * Rcim * Rmw  (即tmp2 * Rmw)
  _jacobianOplusXi(0,0) = -tmp2(0,1)*z+tmp2(0,2)*y;
  _jacobianOplusXi(0,1) = tmp2(0,0)*z-tmp2(0,2)*x;
  _jacobianOplusXi(0,2) = -tmp2(0,0)*y+tmp2(0,1)*x;
  _jacobianOplusXi(0,3) = tmp2(0,0);
  _jacobianOplusXi(0,4) = tmp2(0,1);
  _jacobianOplusXi(0,5) = tmp2(0,2);

  _jacobianOplusXi(1,0) = -tmp2(1,1)*z+tmp2(1,2)*y;
  _jacobianOplusXi(1,1) = tmp2(1,0)*z-tmp2(1,2)*x;
  _jacobianOplusXi(1,2) = -tmp2(1,0)*y+tmp2(1,1)*x;
  _jacobianOplusXi(1,3) = tmp2(1,0);
  _jacobianOplusXi(1,4) = tmp2(1,1);
  _jacobianOplusXi(1,5) = tmp2(1,2);

  _jacobianOplusXi(2,0) = -tmp2(2,1)*z+tmp2(2,2)*y;
  _jacobianOplusXi(2,1) = tmp2(2,0)*z-tmp2(2,2)*x;
  _jacobianOplusXi(2,2) = -tmp2(2,0)*y+tmp2(2,1)*x;
  _jacobianOplusXi(2,3) = tmp2(2,0);
  _jacobianOplusXi(2,4) = tmp2(2,1);
  _jacobianOplusXi(2,5) = tmp2(2,2);

}


} // end namespace
