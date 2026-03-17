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

DataProcessorRos::DataProcessorRos(rclcpp::Node::SharedPtr nh) : nh_(std::move(nh)) {}

void DataProcessorRos::initCommonRosStuff() {
  cloudTopic_ = tryGetParam<std::string>("cloud_topic", *nh_);
  std::cout << "Cloud topic is given as " << cloudTopic_ << std::endl;
  const auto latchedQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  rawCloudPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("raw_cloud", latchedQos);
  numAccumulatedRangeDataDesired_ = static_cast<size_t>(tryGetParam<int>("num_accumulated_range_data", *nh_));
  std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired_ << std::endl;
}

void DataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  (void)cloud;
  (void)timestamp;
  std::cout << "Warning you have not implemented processMeasurement!!! \n";
}

std::shared_ptr<SlamWrapper> DataProcessorRos::getSlamPtr() {
  return slam_;
}

void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp) {
  const size_t minNumCloudsReceived = magic::skipFirstNPointClouds;
  if (numPointCloudsReceived_ < minNumCloudsReceived) {
    ++numPointCloudsReceived_;
    return;
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

  numAccumulatedRangeDataCount_ = 0;
  accumulatedCloud_.Clear();
}

}  // namespace o3d_slam
