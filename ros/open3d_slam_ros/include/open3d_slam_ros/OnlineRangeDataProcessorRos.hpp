/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  explicit OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr node);
  ~OnlineRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp, const Transform& odomToRangeSensor) override;

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::string externalPoseFrame_;
  double externalPoseLookupTimeoutSec_ = 0.1;
};

}  // namespace o3d_slam
