/*
 * RosbagRangeDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class RosbagRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  explicit RosbagRangeDataProcessorRos(rclcpp::Node::SharedPtr nh);
  ~RosbagRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void readRosbag(rosbag2_cpp::Reader& reader);

  std::string rosbagFilename_;
};

}  // namespace o3d_slam
