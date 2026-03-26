/*
 * DataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/DataProcessorRos.hpp"

#include "open3d_slam/magic.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

DataProcessorRos::DataProcessorRos(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void DataProcessorRos::initCommonRosStuff() {
  cloudTopic_ = tryGetParam<std::string>("cloud_topic", *node_);
  std::cout << "Cloud topic is given as " << cloudTopic_ << std::endl;
  rawCloudPub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", rclcpp::QoS(1).transient_local());
  numAccumulatedRangeDataDesired_ = getParamOr<int>("num_accumulated_range_data", *node_, 1);
  std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired_ << std::endl;
}

void DataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  std::cout << "Warning you have not implemented processMeasurement!!! \n";
}

void DataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp, const Transform& odomToRangeSensor) {
  std::cout << "Warning you have not implemented processMeasurement with externally supplied odometry!!! \n";
}

std::shared_ptr<SlamWrapper> DataProcessorRos::getSlamPtr() {
  return slam_;
}

void DataProcessorRos::resetAccumulatedRangeData() {
  numAccumulatedRangeDataCount_ = 0;
  hasAccumulatedExternalOdometry_ = false;
  accumulatedCloud_.Clear();
}

void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp) {
  const size_t minNumCloudsReceived = magic::skipFirstNPointClouds;
  if (numPointCloudsReceived_ < minNumCloudsReceived) {
    ++numPointCloudsReceived_;
    return;
    // somehow the first cloud can be missing a lot of points when running with ouster os-128 on the robot
    // if we skip that first measurement, it all works okay
    // we skip first five, just to be extra safe
  }

  if (hasAccumulatedExternalOdometry_) {
    resetAccumulatedRangeData();
  }

  accumulatedCloud_ += cloud;
  ++numAccumulatedRangeDataCount_;
  if (numAccumulatedRangeDataCount_ < numAccumulatedRangeDataDesired_) {
    return;
  }

  if (accumulatedCloud_.IsEmpty()) {
    std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
    return;
  }

  processMeasurement(accumulatedCloud_, timestamp);

  resetAccumulatedRangeData();
}

void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp,
                                                     const Transform& odomToRangeSensor) {
  const size_t minNumCloudsReceived = magic::skipFirstNPointClouds;
  if (numPointCloudsReceived_ < minNumCloudsReceived) {
    ++numPointCloudsReceived_;
    return;
  }

  if (numAccumulatedRangeDataCount_ > 0 && !hasAccumulatedExternalOdometry_) {
    resetAccumulatedRangeData();
  }

  accumulatedCloud_ += cloud;
  accumulatedOdomToRangeSensor_ = odomToRangeSensor;
  hasAccumulatedExternalOdometry_ = true;
  ++numAccumulatedRangeDataCount_;
  if (numAccumulatedRangeDataCount_ < numAccumulatedRangeDataDesired_) {
    return;
  }

  if (accumulatedCloud_.IsEmpty()) {
    std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
    return;
  }

  if (!hasAccumulatedExternalOdometry_) {
    std::cout << "Trying to insert a cloud without the required external odometry measurement \n";
    return;
  }

  processMeasurement(accumulatedCloud_, timestamp, accumulatedOdomToRangeSensor_);

  resetAccumulatedRangeData();
}

}  // namespace o3d_slam
