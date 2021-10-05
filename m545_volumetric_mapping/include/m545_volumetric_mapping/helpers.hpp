/*
 * helpers.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>
#include "m545_volumetric_mapping/Parameters.hpp"

namespace m545_mapping {

void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl);
void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl);
std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> icpObjectiveFactory(
		const m545_mapping::IcpObjective &obj);

std::string asString (const Eigen::Isometry3d &T);



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

















///////////////////////////////////////////////////////////////////
class Timer {
public:
	Timer(bool isPrintInDestructor = false);
	~Timer();
	double elapsedMsec() const;
	double elapsedSec() const;
private:
	std::chrono::steady_clock::time_point startTime_;
	bool isPrintInDestructor_ = false;
};

} /* namespace m545_mapping */
