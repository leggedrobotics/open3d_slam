/*
 * DataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

class DataProcessorRos {
 public:
  DataProcessorRos(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor);
  virtual ~DataProcessorRos() = default;

  virtual void initialize() = 0;
  virtual void startProcessing() = 0;
  virtual void processMeasurement(const PointCloud& cloud, const Time& timestamp);
  void accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp);
  void initCommonRosStuff();
  std::shared_ptr<SlamWrapper> getSlamPtr();

 protected:
  size_t numAccumulatedRangeDataCount_ = 0;
  size_t numPointCloudsReceived_ = 0;
  size_t numAccumulatedRangeDataDesired_ = 1;
  PointCloud accumulatedCloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawCloudPub_;
  std::string cloudTopic_;
  std::shared_ptr<SlamWrapper> slam_;
  rclcpp::Node* nh_;
  rclcpp::executors::SingleThreadedExecutor* executor_;
};

}  // namespace o3d_slam
