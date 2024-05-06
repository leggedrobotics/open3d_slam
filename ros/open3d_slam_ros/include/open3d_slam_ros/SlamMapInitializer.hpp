/*
 * SlamMapInitializer.hpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <nav_msgs/msg/odometry.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <atomic>
#include <memory>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/SlamWrapper.hpp"

namespace o3d_slam {

class SlamMapInitializer {
 public:
  SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor);
  ~SlamMapInitializer();

  void initialize(const MapInitializingParameters& params);

 private:
  void initInteractiveMarker();
  void setPoseCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr& msg);
  void initMapCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr& msg);
  void initializeWorker();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void initSlamCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  visualization_msgs::msg::InteractiveMarker createInteractiveMarker() const;
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;
  std::shared_ptr<SlamWrapper> slamPtr_;
  std::atomic_bool initialized_;
  MapInitializingParameters mapInitializerParams_;
  rclcpp::Node* nh_;
  std::thread initWorker_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initializeSlamSrv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initPoseSub_;
  std::string interactiveMarkerName_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;

  rclcpp::executors::SingleThreadedExecutor* executor_;
};

}  // namespace o3d_slam
