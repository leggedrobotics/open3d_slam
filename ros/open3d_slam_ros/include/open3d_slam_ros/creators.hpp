/*
 * creators.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"

namespace o3d_slam {

std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(rclcpp::Node::SharedPtr node);
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(rclcpp::Node::SharedPtr node);

std::shared_ptr<DataProcessorRos> dataProcessorFactory(rclcpp::Node::SharedPtr node, bool isProcessAsFastAsPossible);

} /* namespace o3d_slam */
