/*
 * SlamWrapperRos.hpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_msgs/srv/save_map.hpp"
#include "open3d_slam_msgs/srv/save_submaps.hpp"

namespace o3d_slam {

class SlamWrapperRos : public SlamWrapper {
  using BASE = SlamWrapper;

 public:
  explicit SlamWrapperRos(rclcpp::Node::SharedPtr nh);
  ~SlamWrapperRos() override;

  void saveMapCallback(
    const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Response> res);
  void saveSubmapsCallback(
    const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Request> req,
    std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Response> res);
  void loadParametersAndInitialize() override;
  void startWorkers() override;

 private:
  void tfWorker();
  void visualizationWorker();
  void odomPublisherWorker();

  void publishMaps(const Time& time);
  void publishDenseMap(const Time& time);
  void publishMapToOdomTf(const Time& time);

  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odometryInputPub_, mappingInputPub_, assembledMapPub_, denseMapPub_, submapsPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submapOriginsPub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr scan2scanTransformPublisher_, scan2mapTransformPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan2scanOdomPublisher_, scan2mapOdomPublisher_;
  rclcpp::Service<open3d_slam_msgs::srv::SaveMap>::SharedPtr saveMapSrv_;
  rclcpp::Service<open3d_slam_msgs::srv::SaveSubmaps>::SharedPtr saveSubmapsSrv_;
  bool isVisualizationFirstTime_ = true;
  std::thread tfWorker_, visualizationWorker_, odomPublisherWorker_;
  Time prevPublishedTimeScanToScan_, prevPublishedTimeScanToMap_;
  Time prevPublishedTimeScanToScanOdom_, prevPublishedTimeScanToMapOdom_;
};

}  // namespace o3d_slam
