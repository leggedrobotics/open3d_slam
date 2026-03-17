/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/serialization.hpp>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(rclcpp::Node::SharedPtr nh) : BASE(std::move(nh)) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  slam_->loadParametersAndInitialize();
  rosbagFilename_ = tryGetParam<std::string>("rosbag_filepath", *nh_);
  std::cout << "Reading from rosbag: " << rosbagFilename_ << "\n";
}

void RosbagRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh_);
  std::thread spinThread([&executor]() { executor.spin(); });

  rosbag2_cpp::Reader reader;
  reader.open(rosbagFilename_);
  readRosbag(reader);

  executor.cancel();
  spinThread.join();
  slam_->stopWorkers();
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  slam_->addRangeScan(cloud, timestamp);
  const auto cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
  if (isTimeValid(cloudTimePair.second) && !isCloudEmpty) {
    o3d_slam::publishCloud(cloudTimePair.first, o3d_slam::frames::rangeSensorFrame, toRos(cloudTimePair.second), rawCloudPub_);
  }
}

void RosbagRangeDataProcessorRos::readRosbag(rosbag2_cpp::Reader& reader) {
  Timer rosbagTimer;
  Timer rosbagProcessingTimer;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  rclcpp::Time firstTimestamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time lastTimestamp(0, 0, RCL_ROS_TIME);
  bool isFirstMessage = true;

  while (reader.has_next()) {
    auto bagMessage = reader.read_next();
    if (bagMessage->topic_name != cloudTopic_ && ("/" + bagMessage->topic_name) != cloudTopic_) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    rclcpp::SerializedMessage serialized(*bagMessage->serialized_data);
    serializer.deserialize_message(&serialized, &cloud);

    if (isFirstMessage) {
      isFirstMessage = false;
      firstTimestamp = rclcpp::Time(cloud.header.stamp);
      lastTimestamp = firstTimestamp;
    }

    while (rclcpp::ok()) {
      const bool isOdomBufferFull = slam_->getOdometryBufferSize() + 1 >= slam_->getOdometryBufferSizeLimit();
      const bool isMappingBufferFull = slam_->getMappingBufferSize() + 1 >= slam_->getMappingBufferSizeLimit();
      if (!isOdomBufferFull && !isMappingBufferFull) {
        cloudCallback(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud));
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!rclcpp::ok()) {
      slam_->stopWorkers();
      return;
    }

    const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
    if (elapsedWallTime > 15.0) {
      const double elapsedRosbagTime = (rclcpp::Time(cloud.header.stamp) - lastTimestamp).seconds();
      std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: " << 100.0 * elapsedRosbagTime / elapsedWallTime
                << " % realtime speed \n";
      rosbagProcessingTimer.reset();
      lastTimestamp = rclcpp::Time(cloud.header.stamp);
    }
  }

  const double bagDurationSec = isFirstMessage ? 0.0 : (lastTimestamp - firstTimestamp).seconds();
  std::cout << "Rosbag processing finished. Rosbag duration: " << bagDurationSec
            << " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
  slam_->finishProcessing();
}

void RosbagRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
