/*
 * Transform.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#pragma once
#include <cstdint>
#include <Eigen/Dense>
#include "open3d_slam/time.hpp"

namespace o3d_slam {

using Transform = Eigen::Isometry3d;

struct TimestampedTransform {
	Time time_;
	Transform transform_;
};

TimestampedTransform interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const Time &time);

Transform makeTransform(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);

} //namespace o3d_slam


