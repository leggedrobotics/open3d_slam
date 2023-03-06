/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <Eigen/Eigen>
// Package
//#include "open3d_slam_ros/Trajectory.hpp"

#include "open3d_slam_ros/TrajectoryAlignment.hpp"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace o3d_slam {

class TrajectoryAlignmentHandler {
 public:
  TrajectoryAlignmentHandler();

  // Methods
  void initHandler();
  void addLidarPose(Eigen::Vector3d position, double time);
  void addGnssPose(Eigen::Vector3d position, double time);
  bool alignTrajectories(Transform& transform);

  // Setters
  void setGnssRate(const double& gnssRate);
  void setLidarRate(const double& lidarRate);
  void setMinDistanceHeadingInit(const double& minDistanceHeadingInit);
  void setNoMovementDistance(const double& noMovementDistance);
  void setNoMovementTime(const double& noMovementTime);

  // Getters
  std::vector<std::pair<double, Eigen::Vector3d>> getLidarTrajectory();
  std::vector<std::pair<double, Eigen::Vector3d>> getGnssTrajectory();

 private:
  // Member variables
  TrajectoryAlignment trajectoryAlignment_;
};

}  // namespace graph_msf
