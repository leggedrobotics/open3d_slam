/*
 * math.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include <vector>


namespace m545_mapping {


double calcMean(const std::vector<double> &data);
double calcStandardDeviation(const std::vector<double> &data);

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond fromRPY(double roll, double pitch, double yaw);
Eigen::Vector3d toRPY(const Eigen::Quaterniond &q);
Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy);

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

} /* namespace m545_mapping */
