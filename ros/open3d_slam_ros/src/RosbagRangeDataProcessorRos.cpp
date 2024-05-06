/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.h>

#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor) : BASE(nh, executor) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_, executor_);
  slam_->loadParametersAndInitialize();
  nh_->declare_parameter("rosbag_filepath", "");
  rosbagFilename_ = nh_->get_parameter("rosbag_filepath").as_string();
  std::cout << "Reading from rosbag: " << rosbagFilename_ << "\n";
}

void RosbagRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  rosbag2_cpp::Reader reader_;
  reader_.open(rosbagFilename_);
  //readRosbag(reader_);
  reader_.close();
  executor_->spin();
  slam_->stopWorkers();
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  slam_->addRangeScan(cloud, timestamp);
  std::pair<PointCloud, Time> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
  if (isTimeValid(cloudTimePair.second) && !isCloudEmpty) {
    o3d_slam::publishCloud(cloudTimePair.first, o3d_slam::frames::rangeSensorFrame, toRos(cloudTimePair.second), rawCloudPub_);
  }
}

void RosbagRangeDataProcessorRos::readRosbag(rosbag2_cpp::Reader& reader) {
  Timer rosbagTimer;
  rclcpp::Time lastTimestamp;
  bool isFirstMessage = true;
  Timer rosbagProcessingTimer;
  while (reader.has_next()) {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg = reader.read_next();

    if (msg->topic_name == cloudTopic_ || "/" + msg->topic_name == cloudTopic_ ) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      sensor_msgs::msg::PointCloud2::SharedPtr cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

      serialization_.deserialize_message(&serialized_msg, cloud.get());
      if (cloud != nullptr) {
        if (isFirstMessage) {
          isFirstMessage = false;
          lastTimestamp = cloud->header.stamp;
        }
        //      	std::cout << "reading cloud msg with seq: " << cloud->header.seq << std::endl;
        while (true) {
          const bool isOdomBufferFull = slam_->getOdometryBufferSize() + 1 >= slam_->getOdometryBufferSizeLimit();
          const bool isMappingBufferFull = slam_->getMappingBufferSize() + 1 >= slam_->getMappingBufferSizeLimit();

          if (!isOdomBufferFull && !isMappingBufferFull) {
            cloudCallback(cloud);
            break;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          executor_->spin_some();
          if (!rclcpp::ok()) {
            slam_->stopWorkers();
            return;
          }
        }  // end while
        const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
        if (elapsedWallTime > 15.0) {
          const double elapsedRosbagTime = RCL_NS_TO_S((rclcpp::Time(cloud->header.stamp) - lastTimestamp).nanoseconds());
          std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: " << 100.0 * elapsedRosbagTime / elapsedWallTime
                    << " % realtime speed \n";
          rosbagProcessingTimer.reset();
          lastTimestamp = cloud->header.stamp;
        }
        executor_->spin_some();
      }  // end if checking for the null ptr
    }    // end if checking for the right topic
    if (!rclcpp::ok()) {
      slam_->stopWorkers();
      return;
    }
  }  // end foreach

  //const rclcpp::Time bag_begin_time = view.getBeginTime();
  //const rclcpp::Time bag_end_time = view.getEndTime();
  //std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
  //          << " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
  // a bit of a hack, this extra thread listens to ros shutdown
  // otherwise we might get stuck in a loop
  bool isProcessingFinished = false;
  std::thread rosSpinner([&]() {
    rclcpp::Rate r(20.0);
    while (true) {
      if (!rclcpp::ok()) {
        slam_->stopWorkers();
        break;
      }
      if (isProcessingFinished) {
        break;
      }
      r.sleep();
    }
  });
  slam_->finishProcessing();
  isProcessingFinished = true;
  rosSpinner.join();
}

void RosbagRangeDataProcessorRos::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

}  // namespace o3d_slam
