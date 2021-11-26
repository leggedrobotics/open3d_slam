/*
 * Transform.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#pragma once
#include <cstdint>
#include <Eigen/Dense>
#include "m545_volumetric_mapping/time.hpp"

namespace m545_mapping {

using Transform = Eigen::Isometry3d;

struct TimestampedTransform {
	Time time_;
	Transform transform_;
};

TimestampedTransform interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const Time &time);

Transform makeTransform(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);

} //namespace m545_mapping


