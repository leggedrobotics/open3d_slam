/*
 * Transform.cpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */


#include "open3d_slam/Transform.hpp"
#include <iostream>
#include <string>
#include "Eigen/Geometry"
#include <glog/logging.h>

namespace o3d_slam {


TimestampedTransform interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const Time &time) {

  if (time > end.time_ || time < start.time_){
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(end.time_) << std::endl;
    std::cout << "Query time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
    throw std::runtime_error("transform interpolate:: query time is not between start and end time");
  }

  if (start.time_ > end.time_){
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << toSecondsSinceFirstMeasurement(start.time_) << std::endl;
    std::cout << "End time: " << toSecondsSinceFirstMeasurement(end.time_) << std::endl;
    throw std::runtime_error("transform interpolate:: start time is greater than end time");
  }

  const double duration = toSeconds(end.time_ - start.time_);
  const double factor = toSeconds(time - start.time_) / (duration + 1e-6); //avoid zero division
  const Eigen::Vector3d origin =
      start.transform_.translation() +
      (end.transform_.translation() - start.transform_.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform_.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform_.rotation()));
  Transform transform(rotation);
  transform.translation() = origin;

  return TimestampedTransform{time,transform};
}

Transform makeTransform(const Eigen::Vector3d &p, const Eigen::Quaterniond &q){
	  Transform transform(q);
	  transform.translation() = p;
	  return transform;
}



} /* namespace o3d_slam */
