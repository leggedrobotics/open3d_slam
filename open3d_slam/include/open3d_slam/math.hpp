/*
 * math.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include <vector>
#include "open3d_slam/Transform.hpp"


namespace o3d_slam {


double calcMean(const std::vector<double> &data);
double calcStandardDeviation(const std::vector<double> &data);

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond fromRPY(double roll, double pitch, double yaw);
Eigen::Vector3d toRPY(const Eigen::Quaterniond &q);
Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy);
Transform fromXYZandRPY(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy);
Transform fromXYZandRPY(const Eigen::Vector3d &xyz, double roll, double pitch, double yaw);
Transform fromXYZandRPY(double x, double y, double z, double roll, double pitch, double yaw);
Transform fromXYZandQuaternion(double x, double y, double z, const Eigen::Quaterniond &q);
Transform fromXYZandQuaternion(const Eigen::Vector3d &xyz, const Eigen::Quaterniond &q);

template<typename T>
inline T getRollFromQuat(T w, T x, T y, T z)
{
  return std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

template<typename T>
inline T getPitchFromQuat(T w, T x, T y, T z)
{
  return std::asin(2 * (w * y - x * z));
}

template<typename T>
inline T getYawFromQuat(T w, T x, T y, T z)
{
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

template<typename T>
inline bool isClose(T a, T b, T threshold){
	return std::abs(a-b) < threshold;
}

} /* namespace o3d_slam */
