/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor) : BASE(nh, executor) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_, executor_);
  slam_->loadParametersAndInitialize();
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();
  cloudSubscriber_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(cloudTopic_, 100, std::bind(&OnlineRangeDataProcessorRos::cloudCallback, this, std::placeholders::_1));
  executor_->spin();
  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  slam_->addRangeScan(cloud, timestamp);
  o3d_slam::publishCloud(cloud, o3d_slam::frames::rangeSensorFrame, toRos(timestamp), rawCloudPub_);
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
