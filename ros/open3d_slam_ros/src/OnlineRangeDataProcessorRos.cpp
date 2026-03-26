/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr node) : BASE(std::move(node)) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(node_);
  slam_->loadParametersAndInitialize();
  externalPoseFrame_ = getParamOr<std::string>("external_pose_frame", *node_, std::string(""));
  externalPoseLookupTimeoutSec_ = getParamOr<double>("external_pose_lookup_timeout_sec", *node_, 0.1);
  if (!externalPoseFrame_.empty()) {
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  }
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();
  cloudSubscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloudTopic_, rclcpp::QoS(100), std::bind(&OnlineRangeDataProcessorRos::cloudCallback, this, std::placeholders::_1));
  rclcpp::spin(node_);
  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  if (externalPoseFrame_.empty()) {
    slam_->addRangeScan(cloud, timestamp);
  } else {
    slam_->addRangeScanForOdometryOnly(cloud, timestamp);
  }
  o3d_slam::publishCloud(cloud, slam_->getFrames().rangeSensorFrame, toRos(timestamp), rawCloudPub_);
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp,
                                                     const Transform& odomToRangeSensor) {
  slam_->addRangeScan(cloud, timestamp, odomToRangeSensor);
  o3d_slam::publishCloud(cloud, slam_->getFrames().rangeSensorFrame, toRos(timestamp), rawCloudPub_);
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  slam_->setRangeSensorFrame(msg->header.frame_id);
  const Time timestamp = fromRos(msg->header.stamp);
  if (!externalPoseFrame_.empty()) {
    Eigen::Isometry3d externalPose = Eigen::Isometry3d::Identity();
    if (!lookupTransform(externalPoseFrame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp), *tfBuffer_, &externalPose,
                         externalPoseLookupTimeoutSec_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Processing cloud with ICP odometry only because transform %s -> %s was unavailable at the message timestamp.",
                           externalPoseFrame_.c_str(), msg->header.frame_id.c_str());
      accumulateAndProcessRangeData(cloud, timestamp);
      return;
    }
    accumulateAndProcessRangeData(cloud, timestamp, Transform(externalPose));
    return;
  }
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
