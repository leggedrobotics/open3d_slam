//
// Created by peyschen on 10/02/23.
//

#ifndef OPEN3D_SLAM_PLANE_H
#define OPEN3D_SLAM_PLANE_H
#include <Eigen/Core>
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {
class Plane {
 public:
  void initialize(const std::vector<PointWithCov>& pts);
  void update(const std::vector<PointWithCov>& pts);
  bool isPlane = false;
  bool isInitialized = false;
  int id;

 private:
  Matrix6d planeCov_ = Matrix6d::Zero();
  Eigen::Matrix3d cov_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d u_, v_;
  double eigenValues_[3];
  float planeThreshold_;
  int numPoints_;
};
}  // namespace o3d_slam

#endif  // OPEN3D_SLAM_PLANE_H
