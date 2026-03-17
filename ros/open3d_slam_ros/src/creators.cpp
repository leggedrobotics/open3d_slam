/*
 * creators.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(rclcpp::Node::SharedPtr node) {
  return std::make_shared<OnlineRangeDataProcessorRos>(std::move(node));
}
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(rclcpp::Node::SharedPtr node) {
  return std::make_shared<RosbagRangeDataProcessorRos>(std::move(node));
}

std::shared_ptr<DataProcessorRos> dataProcessorFactory(rclcpp::Node::SharedPtr node, bool isProcessAsFastAsPossible) {
  if (isProcessAsFastAsPossible) {
    return createRosbagDataProcessor(std::move(node));
  }
  return createOnlineDataProcessor(std::move(node));
}

} /* namespace o3d_slam */
