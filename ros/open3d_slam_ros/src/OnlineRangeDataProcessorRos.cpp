/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr node) : BASE(std::move(node)) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(node_);
  slam_->loadParametersAndInitialize();
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();
  cloudSubscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloudTopic_, rclcpp::QoS(100), std::bind(&OnlineRangeDataProcessorRos::cloudCallback, this, std::placeholders::_1));
  rclcpp::spin(node_);
  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  slam_->addRangeScan(cloud, timestamp);
  const auto slamRos = std::static_pointer_cast<SlamWrapperRos>(slam_);
  o3d_slam::publishCloud(cloud, slamRos->publishedRangeSensorFrame(), toRos(timestamp), rawCloudPub_);
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  std::static_pointer_cast<SlamWrapperRos>(slam_)->setPublishedRangeSensorFrame(msg->header.frame_id);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
