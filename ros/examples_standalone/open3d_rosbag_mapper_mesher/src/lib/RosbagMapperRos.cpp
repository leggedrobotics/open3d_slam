/*
 * RosbagMapper.cpp
 *
 *  Created on: Nov 19, 2024
 *      Author: nubertj
 */

// Implementation
#include "open3d_rosbag_mapper_mesher/RosbagMapperRos.h"

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// Open3D SLAM
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

// Package
#include "open3d_rosbag_mapper_mesher/file_utils.h"

namespace o3d_slam {

RosbagMapperRos::RosbagMapperRos(ros::NodeHandlePtr nh) : BASE(nh), submap_(0, 0) {
  denseCloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_cloud", 1, true);
}

void RosbagMapperRos::initialize() {
  initCommonRosStuff();

  // Load the rosbag filepaths
  tfStaticRosbagFilename_ = tryGetParam<std::string>("tf_static_rosbag_filepath", *nh_);
  tfRosbagFilename_ = tryGetParam<std::string>("tf_rosbag_filepath", *nh_);
  pcRosbagDirectoryName_ = tryGetParam<std::string>("pc_rosbag_directory_name", *nh_);
  pcRosbagPrefix_ = tryGetParam<std::string>("pc_rosbag_prefix", *nh_);
  std::cout << "TF static rosbag filepath: " << tfStaticRosbagFilename_ << std::endl;
  std::cout << "TF rosbag filepath: " << tfRosbagFilename_ << std::endl;
  std::cout << "PC rosbag directory name: " << pcRosbagDirectoryName_ << std::endl;
  std::cout << "PC rosbag prefix: " << pcRosbagPrefix_ << std::endl;
  // Frame names
  worldFrame_ = tryGetParam<std::string>("world_frame", *nh_);
  lidarFrame_ = tryGetParam<std::string>("lidar_frame", *nh_);
  std::cout << "World frame: " << worldFrame_ << std::endl;
  std::cout << "Lidar frame: " << lidarFrame_ << std::endl;
  // Publish rate
  publishMapEveryNScans_ = tryGetParam<int>("publish_map_every_n_scans", *nh_);
  // Saving of the map
  mapSavingFolderPath_ = tryGetParam<std::string>("map_saving_folder_path", *nh_);
  mapSavingFilename_ = tryGetParam<std::string>("map_saving_filename", *nh_);

  // Load the parameters
  const std::string paramFolderPath = nh_->param<std::string>("parameter_folder_path", "");
  const std::string paramFilename = nh_->param<std::string>("parameter_filename", "");
  io_lua::loadParameters(paramFolderPath, paramFilename, &params_);
  // Set the parameters
  std::cout << "Voxel size before setting parameters: " << submap_.getDenseMap().getVoxelSize() << std::endl;
  submap_.setParameters(params_.mapper_);
  std::cout << "Voxel size: " << submap_.getDenseMap().getVoxelSize() << std::endl;
}

void RosbagMapperRos::startProcessing() {
  // Open the rosbags
  // TF static
  rosbag::Bag tfStaticBag;
  std::cout << "Opening TF static rosbag: " << tfStaticRosbagFilename_ << "\n";
  tfStaticBag.open(tfStaticRosbagFilename_, rosbag::bagmode::Read);
  std::cout << "...done. \n";
  // TF
  rosbag::Bag tfBag;
  std::cout << "Opening TF rosbag: " << tfRosbagFilename_ << "\n";
  tfBag.open(tfRosbagFilename_, rosbag::bagmode::Read);
  std::cout << "...done. \n";
  // Pointcloud
  std::vector<std::shared_ptr<rosbag::Bag>> pointCloudBagPtrVector = {};
  // Get all files in the directory starting with pcRosbagFilename_
  std::vector<std::string> fileNames = getFilesWithPrefix(pcRosbagDirectoryName_, pcRosbagPrefix_);
  for (const auto& fileName : fileNames) {
    std::shared_ptr<rosbag::Bag> bagPtr = std::make_shared<rosbag::Bag>();
    std::cout << "Opening pointcloud rosbag: " << fileName << "\n";
    bagPtr->open(fileName, rosbag::bagmode::Read);
    pointCloudBagPtrVector.push_back(bagPtr);
  }
  std::cout << "...done. \n";
  // Read the rosbag
  readRosbags(pointCloudBagPtrVector, tfBag, tfStaticBag);
  ros::spin();
}

void RosbagMapperRos::readRosbags(const std::vector<std::shared_ptr<rosbag::Bag>>& pcBagVector, const rosbag::Bag& tfBag,
                                  const rosbag::Bag& tfStaticBag) {
  // Rosbag
  rosbag::View tfStaticBagView(tfStaticBag, rosbag::TopicQuery("/tf_static"));
  rosbag::View tfBagView(tfBag, rosbag::TopicQuery("/tf"));
  Timer rosbagTimer;
  ros::Time lastTimestamp;
  // Start
  bool isFirstMessage = true;
  Timer rosbagProcessingTimer;

  // Create a tf2 buffer and listener
  tf2_ros::Buffer tfBuffer(ros::Duration(10000.0));
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Earliest and last timestamp
  ros::Time earliest_time = ros::Time::now();  // Initialize to now (a large value)
  ros::Time latest_time = ros::Time(0);        // Initialize to 0 (a small value)

  // Step 1: Preload static transforms
  for (const rosbag::MessageInstance& msg : tfStaticBagView) {
    if (msg.getTopic() == "/tf_static") {
      auto tfStaticMsg = msg.instantiate<tf2_msgs::TFMessage>();
      if (tfStaticMsg != nullptr) {
        for (const auto& transform : tfStaticMsg->transforms) {
          try {
            tfBuffer.setTransform(transform, "bag_reader", true);  // `true` for static
          } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM("Error setting static transform: " << ex.what());
            throw std::runtime_error("Error setting static transform");
          }
        }
      }
    }
  }

  // Step 2: Loading dynamic transforms
  for (const rosbag::MessageInstance& msg : tfBagView) {
    if (msg.getTopic() == "/tf") {
      auto tfMsg = msg.instantiate<tf2_msgs::TFMessage>();
      if (tfMsg != nullptr) {
        for (const auto& transform : tfMsg->transforms) {
          try {
            tfBuffer.setTransform(transform, "bag_reader");
            if (transform.header.stamp < earliest_time) {
              earliest_time = transform.header.stamp;
            } else if (transform.header.stamp > latest_time) {
              latest_time = transform.header.stamp;
            }
          } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM("Error updating dynamic transform: " << ex.what());
            throw std::runtime_error("Error updating dynamic transform");
          }
        }
      }
    }
  }

  // Print
  std::cout << "TF buffer preloaded with static and dynamic transforms. Earliest time: " << earliest_time << " Latest time: " << latest_time
            << std::endl;

  // Step 2: Main loop for reading the rosbags
  for (const auto& pcBag : pcBagVector) {
    std::cout << "Processing pointcloud rosbag: " << pcBag->getFileName() << "\n";
    rosbag::View pcView(*pcBag, rosbag::TopicQuery(cloudTopic_));
    for (const rosbag::MessageInstance& msg : pcView) {
      // Process Cloud
      if (msg.getTopic() == cloudTopic_ || ("/" + msg.getTopic() == cloudTopic_)) {
        sensor_msgs::PointCloud2::ConstPtr cloud = msg.instantiate<sensor_msgs::PointCloud2>();
        if (cloud != nullptr) {
          if (isFirstMessage) {
            isFirstMessage = false;
            lastTimestamp = cloud->header.stamp;
          }
          // Looking Up the TF
          ros::Time msgTime = cloud->header.stamp;
          geometry_msgs::TransformStamped transform;
          try {
            transform = tfBuffer.lookupTransform(worldFrame_, lidarFrame_, msgTime);
          } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM("Could not get transform: " << ex.what());
            std::cout << "Skipping the cloud message \n";
            continue;
          }
          // Process the cloud
          cloudCallback(cloud, transform);

          // Logging
          const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
          if (elapsedWallTime > 15.0) {
            const double elapsedRosbagTime = (cloud->header.stamp - lastTimestamp).toSec();
            std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: " << 100.0 * elapsedRosbagTime / elapsedWallTime
                      << " % realtime speed \n";
            rosbagProcessingTimer.reset();
            lastTimestamp = cloud->header.stamp;
          }
        }  // end if checking for the null ptr
      }  // end if checking for the right topic
      // Shut Down if ROS is not OK
      if (!ros::ok()) {
        return;
      }
    }  // end foreach
  }  // end through all rosbags

  // Saving the map
  std::cout << "Saving the map to: " << mapSavingFolderPath_ << std::endl;
  createDirectoryOrNoActionIfExists(mapSavingFolderPath_);
  saveToFile(mapSavingFolderPath_ + mapSavingFilename_, submap_.getDenseMap().toPointCloud());
  std::cout << "Map saved. \n";

  // a bit of a hack, this extra thread listens to ros shutdown
  // otherwise we might get stuck in a loop
  bool isProcessingFinished = false;
  std::thread rosSpinner([&]() {
    ros::Rate r(20.0);
    while (true) {
      if (!ros::ok()) {
        break;
      }
      if (isProcessingFinished) {
        break;
      }
      r.sleep();
    }
  });
  isProcessingFinished = true;
  rosSpinner.join();
}

void RosbagMapperRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const geometry_msgs::TransformStamped& transform) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  // Convert transform to Eigen
  Transform T_W_L = tf2::transformToEigen(transform.transform);
  // Transform the cloud to the world frame
  // cloud.Transform(T_W_L.matrix());
  accumulateAndProcessRangeData(cloud, timestamp, T_W_L);
}

void RosbagMapperRos::processMeasurement(const PointCloud& cloud, const Time& timestamp, const std::optional<Transform>& transform) {
  // Counter
  static int publishCounter = 0;
  // Transform
  Transform T_W_L = transform.value_or(Transform::Identity());
  // Add to submap
  bool isPerformCarving = false;
  submap_.insertScanDenseMap(cloud, T_W_L, timestamp, isPerformCarving);
  // Publish the dense cloud
  const bool isCloudEmpty = submap_.getDenseMap().empty();
  if (!isCloudEmpty && !(publishCounter % publishMapEveryNScans_)) {
    std::cout << "Publishing the dense cloud in frame: " << worldFrame_ << std::endl;
    o3d_slam::publishCloud(submap_.getDenseMap().toPointCloud(), worldFrame_, toRos(timestamp), denseCloudPub_);
  }
  publishCounter++;
}

}  // namespace o3d_slam