/*
 * creators.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor) {
  return std::make_shared<OnlineRangeDataProcessorRos>(nh, executor);
}
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor) {
  return std::make_shared<RosbagRangeDataProcessorRos>(nh, executor);
}

std::shared_ptr<DataProcessorRos> dataProcessorFactory(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor, bool isProcessAsFastAsPossible) {
  if (isProcessAsFastAsPossible) {
    return createRosbagDataProcessor(nh, executor);
  } else {
    return createOnlineDataProcessor(nh, executor);
  }
}

} /* namespace o3d_slam */
