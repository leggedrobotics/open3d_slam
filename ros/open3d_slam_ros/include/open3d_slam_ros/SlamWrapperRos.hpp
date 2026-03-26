/*
 * SlamWrapperRos.hpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
  explicit SlamWrapperRos(rclcpp::Node::SharedPtr node);
  ~SlamWrapperRos() override;

  void loadParametersAndInitialize() override;
  void startWorkers() override;

 private:
  void saveMapCallback(const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Request> req,
                       std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Response> res);
  void saveSubmapsCallback(const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Request> req,
                           std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Response> res);
  void tfWorker();
  void visualizationWorker();
  void odomPublisherWorker();

  void publishMaps(const Time& time);
  void publishDenseMap(const Time& time);
  void publishMapToOdomTf(const Time& time);
  std::string getMapOutputFrame() const;
  Transform getMapOutputFrameToMap(const Time& time) const;
  PointCloud transformMapCloudToOutputFrame(const PointCloud& cloud, const Time& time) const;
  Transform transformMapPoseToOutputFrame(const Transform& mapToPose, const Time& time) const;
  bool saveMapInOutputFrame(const std::string& directory) const;
  bool saveSubmapsInOutputFrame(const std::string& directory) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odometryInputPub_, mappingInputPub_, assembledMapPub_, denseMapPub_,
      submapsPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submapOriginsPub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr scan2scanTransformPublisher_, scan2mapTransformPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan2scanOdomPublisher_, scan2mapOdomPublisher_;
  rclcpp::Service<open3d_slam_msgs::srv::SaveMap>::SharedPtr saveMapSrv_;
  rclcpp::Service<open3d_slam_msgs::srv::SaveSubmaps>::SharedPtr saveSubmapsSrv_;
  bool isVisualizationFirstTime_ = true;
  bool publishTf_ = true;
  std::string externalPoseFrame_;
  std::thread tfWorker_, visualizationWorker_, odomPublisherWorker_;
  Time prevPublishedTimeScanToScan_, prevPublishedTimeScanToMap_;
  Time prevPublishedTimeScanToScanOdom_, prevPublishedTimeScanToMapOdom_;
};

}  // namespace o3d_slam
