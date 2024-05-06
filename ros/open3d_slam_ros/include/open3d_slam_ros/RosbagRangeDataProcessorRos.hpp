/*
 * RosbagRangeDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class RosbagRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  RosbagRangeDataProcessorRos(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor);
  ~RosbagRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void readRosbag(rosbag2_cpp::Reader& bag);
  
  std::string rosbagFilename_;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
};

}  // namespace o3d_slam
