/*
 * RosbagRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */



#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"
#include <rosbag/view.h>

namespace o3d_slam {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh) :
		BASE(nh) {

}

void RosbagRangeDataProcessorRos::initialize() {
	initCommonRosStuff();
	slam_ = std::make_shared<SlamWrapperRos>(nh_);
	slam_->loadParametersAndInitialize();
	rosbagFilename_ = nh_->param<std::string>("rosbag_filepath", "");
				std::cout << "Reading from rosbag: " << rosbagFilename_ << "\n";

}

void RosbagRangeDataProcessorRos::startProcessing() {
	slam_->startWorkers();

	rosbag::Bag bag;
	bag.open(rosbagFilename_, rosbag::bagmode::Read);
	readRosbag(bag);
	bag.close();
	ros::spin();
	slam_->stopWorkers();
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {

	slam_->addRangeScan(cloud, timestamp);
	std::pair<PointCloud, Time> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
	const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
	if (isTimeValid(cloudTimePair.second) && !isCloudEmpty) {
		o3d_slam::publishCloud(cloudTimePair.first, o3d_slam::frames::rangeSensorFrame,
				toRos(cloudTimePair.second), rawCloudPub_);
	}

}

void RosbagRangeDataProcessorRos::readRosbag(const rosbag::Bag &bag) {
	std::vector<std::string> topics;
	topics.push_back(cloudTopic_);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	Timer rosbagTimer;
	ros::Time lastTimestamp;
	bool isFirstMessage = true;
	Timer rosbagProcessingTimer;
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		if (m.getTopic() == cloudTopic_ || ("/" + m.getTopic() == cloudTopic_)) {
			sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
			if (cloud != nullptr) {
				if (isFirstMessage) {
					isFirstMessage = false;
					lastTimestamp = cloud->header.stamp;
				}
				//      	std::cout << "reading cloud msg with seq: " << cloud->header.seq << std::endl;
				while (true) {
					const bool isOdomBufferFull = slam_->getOdometryBufferSize() + 1
							>= slam_->getOdometryBufferSizeLimit();
					const bool isMappingBufferFull = slam_->getMappingBufferSize() + 1
							>= slam_->getMappingBufferSizeLimit();

					if (!isOdomBufferFull && !isMappingBufferFull) {
						cloudCallback(cloud);
						break;
					} else {
						std::this_thread::sleep_for(std::chrono::milliseconds(20));
					}
					ros::spinOnce();
					if (!ros::ok()) {
						slam_->stopWorkers();
						return;
					}
				} // end while
				const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
				if (elapsedWallTime > 15.0) {
					const double elapsedRosbagTime = (cloud->header.stamp - lastTimestamp).toSec();
					std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: "
							<< 100.0 * elapsedRosbagTime / elapsedWallTime << " % realtime speed \n";
					rosbagProcessingTimer.reset();
					lastTimestamp = cloud->header.stamp;
				}
				ros::spinOnce();
			} // end if checking for the null ptr
		} // end if checking for the right topic
		if (!ros::ok()) {
			slam_->stopWorkers();
			return;
		}
	} // end foreach

	const ros::Time bag_begin_time = view.getBeginTime();
	const ros::Time bag_end_time = view.getEndTime();
	std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
			<< " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
	// a bit of a hack, this extra thread listens to ros shutdown
	// otherwise we might get stuck in a loop
	bool isProcessingFinished = false;
	std::thread rosSpinner([&]() {
		ros::Rate r(20.0);
		while (true) {
			if (!ros::ok()) {
				slam_->stopWorkers();
				break;
			}
			if (isProcessingFinished){
				break;
			}
			r.sleep();
		}
	});
	slam_->finishProcessing();
	isProcessingFinished = true;
	rosSpinner.join();
}

void RosbagRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	const Time timestamp = fromRos(msg->header.stamp);
	accumulateAndProcessRangeData(cloud, timestamp);
}


} // namespace o3d_slam


