/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <Eigen/Eigen>
#include <memory>
// Package
#include "open3d_slam_ros/Trajectory.hpp"
#include "open3d_slam/Transform.hpp"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace o3d_slam {

class TrajectoryAlignment {
 public:
  TrajectoryAlignment();

  // Methods
  void addLidarPose(Eigen::Vector3d position, double time);
  void addGnssPose(Eigen::Vector3d position, double time);
  bool alignTrajectories(Transform& transform);

  // Setters
  void setGnssRate(const double gnssRate) { gnssRate_ = gnssRate; }
  void setLidarRate(const double lidarRate) { lidarRate_ = lidarRate; }
  void setMinDistanceHeadingInit(const double minDistanceHeadingInit) { minDistanceHeadingInit_ = minDistanceHeadingInit; }
  void setNoMovementDistance(const double noMovementDistance) { noMovementDistance_ = noMovementDistance; }
  void setNoMovementTime(const double noMovementTime) { noMovementTime_ = noMovementTime; }

  // Getters
  std::vector<std::pair<double, Eigen::Vector3d>> getLidarTrajectory();
  std::vector<std::pair<double, Eigen::Vector3d>> getGnssTrajectory();

 private:
  // Member methods
  bool associateTrajectories(Trajectory& trajectoryA, Trajectory& trajectoryB, Trajectory& newTrajectoryA, Trajectory& newTrajectoryB);
  bool trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Matrix4d& transform);

  // Member variables
  Trajectory gnssTrajectory_;
  Trajectory lidarTrajectory_;

  // Reference Parameters
  double gnssRate_;
  double lidarRate_;
  double minDistanceHeadingInit_;
  double noMovementDistance_;
  double noMovementTime_;
};

}  // namespace graph_msf