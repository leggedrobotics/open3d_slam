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
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  explicit OnlineRangeDataProcessorRos(rclcpp::Node::SharedPtr nh);
  ~OnlineRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
};

}  // namespace o3d_slam
