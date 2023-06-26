//
// Created by peyschen on 08/06/23.
//

#ifndef REALTIME_MESHING_REALTIMEMESHING_H
#define REALTIME_MESHING_REALTIMEMESHING_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <utility>
#include "open3d_slam_utils/CircularBuffer.hpp"
#include "open3d_slam_utils/TransformInterpolationBuffer.hpp"
#include "realtime_meshing/Mesher.hpp"
#include "realtime_meshing_ros/types.h"

class RealtimeMeshingRos {
 public:
  explicit RealtimeMeshingRos(ros::NodeHandlePtr nh)
      : nh_(std::move(nh)), pointCloudBuffer_(std::make_unique<o3d_slam::CircularBuffer<StampedCloud>>()) {
    pointCloudBuffer_->set_size_limit(2);
    initializeRos();
  };
  ~RealtimeMeshingRos();

 private:
  void meshingWorker();
  void publishWorker();
  void initializeRos();

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::Subscriber odomSub_;
  ros::Subscriber pointcloudSub_;
  ros::Publisher mapPub_;
  ros::NodeHandlePtr nh_;

  std::unique_ptr<Mesher> mesher_;
  MeshingParameters params_;

  nav_msgs::Odometry lastOdomMsg_;
  o3d_slam::PointCloud map_;
  o3d_slam::TransformInterpolationBuffer tfBuffer_;
  o3d_slam::Transform lastInsertionTf_;
  std::unique_ptr<o3d_slam::CircularBuffer<StampedCloud>> pointCloudBuffer_;
  std::thread meshingThread_;
  std::thread publishThread_;
  bool workersShouldRun_ = true;
};

#endif  // REALTIME_MESHING_REALTIMEMESHING_H
