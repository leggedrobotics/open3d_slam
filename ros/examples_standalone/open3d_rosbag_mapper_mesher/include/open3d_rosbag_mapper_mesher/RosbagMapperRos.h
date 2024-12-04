/*
 * RosbagMapper.h
 *
 *  Created on: Nov 19, 2024
 *      Author: nubertj
 */

#pragma once

// C++
#include <memory>

// Ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

// Open3D SLAM
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

class RosbagMapperRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  RosbagMapperRos(ros::NodeHandlePtr nh);
  ~RosbagMapperRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp,
                          const std::optional<Transform>& transform = std::nullopt) override;

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const geometry_msgs::TransformStamped& transform);
  void readRosbags(const std::vector<std::shared_ptr<rosbag::Bag>>& pcBagVector, const rosbag::Bag& tfBag, const rosbag::Bag& tfStaticBag);

  // Members
  // Parameters
  SlamParameters params_;

  // Full Names
  std::string tfStaticRosbagFilename_;
  std::string tfRosbagFilename_;
  // Directory name and prefix
  std::string pcRosbagDirectoryName_;
  std::string pcRosbagPrefix_;
  // Frame Names
  std::string worldFrame_;
  std::string lidarFrame_;
  // Submap
  Submap submap_;
  // Publish Rate
  int publishMapEveryNScans_;

  // Saving of the map
  std::string mapSavingFolderPath_;
  std::string mapSavingFilename_;

  // Publisher
  ros::Publisher denseCloudPub_;
};

}  // namespace o3d_slam
