/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"

#include <chrono>

#include <rclcpp/serialization.hpp>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

namespace {

std::string normalizeTopic(std::string topic) {
  if (topic.empty() || topic.front() == '/') {
    return topic;
  }
  return "/" + topic;
}

bool isBufferFull(size_t currentSize, size_t sizeLimit) {
  assert_gt<size_t>(sizeLimit, 0, "Rosbag processing requires buffer size limits > 0");
  return currentSize >= sizeLimit;
}

}  // namespace

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(rclcpp::Node::SharedPtr node) : BASE(std::move(node)) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(node_);
  slam_->loadParametersAndInitialize();
  rosbagFilename_ = tryGetParam<std::string>("rosbag_filepath", *node_);
  std::cout << "Reading from rosbag: " << rosbagFilename_ << "\n";
}

void RosbagRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  rosbag2_cpp::Reader reader;
  reader.open(rosbagFilename_);
  readRosbag(reader);
  slam_->stopWorkers();
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  slam_->addRangeScan(cloud, timestamp);
  std::pair<PointCloud, Time> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
  if (isTimeValid(cloudTimePair.second) && !isCloudEmpty) {
    o3d_slam::publishCloud(cloudTimePair.first, slam_->getFrames().rangeSensorFrame, toRos(cloudTimePair.second),
                           rawCloudPub_);
  }
}

void RosbagRangeDataProcessorRos::readRosbag(rosbag2_cpp::Reader& reader) {
  Timer rosbagTimer;
  Timer rosbagProcessingTimer;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  const std::string requestedTopic = normalizeTopic(cloudTopic_);
  rclcpp::Time firstTimestamp(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time lastTimestamp(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time progressTimestamp(0, 0, RCL_SYSTEM_TIME);
  bool sawCloud = false;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (normalizeTopic(bag_message->topic_name) != requestedTopic) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    serialization.deserialize_message(&serialized, &cloud);
    const auto cloudPtr = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(cloud));

    if (!sawCloud) {
      firstTimestamp = rclcpp::Time(cloudPtr->header.stamp);
      lastTimestamp = firstTimestamp;
      progressTimestamp = firstTimestamp;
      sawCloud = true;
    }

    while (true) {
      const bool isOdomBufferFull = isBufferFull(slam_->getOdometryBufferSize(), slam_->getOdometryBufferSizeLimit());
      const bool isMappingBufferFull = isBufferFull(slam_->getMappingBufferSize(), slam_->getMappingBufferSizeLimit());
      if (!isOdomBufferFull && !isMappingBufferFull) {
        cloudCallback(cloudPtr);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rclcpp::spin_some(node_);
      if (!rclcpp::ok()) {
        slam_->stopWorkers();
        return;
      }
    }

    const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
    if (elapsedWallTime > 15.0) {
      const double elapsedRosbagTime = (rclcpp::Time(cloudPtr->header.stamp) - progressTimestamp).seconds();
      std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: " << 100.0 * elapsedRosbagTime / elapsedWallTime
                << " % realtime speed \n";
      rosbagProcessingTimer.reset();
      progressTimestamp = rclcpp::Time(cloudPtr->header.stamp);
    }

    lastTimestamp = rclcpp::Time(cloudPtr->header.stamp);

    rclcpp::spin_some(node_);
    if (!rclcpp::ok()) {
      slam_->stopWorkers();
      return;
    }
  }

  if (sawCloud) {
    const double bagDuration = (lastTimestamp - firstTimestamp).seconds();
    std::cout << "Rosbag processing finished. Rosbag duration: " << bagDuration
              << " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
  } else {
    std::cout << "Rosbag processing finished without messages on topic " << requestedTopic << "\n";
  }
  slam_->finishProcessing();
}

void RosbagRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  slam_->setRangeSensorFrame(msg->header.frame_id);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
