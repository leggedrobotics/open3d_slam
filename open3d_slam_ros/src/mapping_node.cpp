/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace {
using namespace o3d_slam;
ros::NodeHandlePtr nh;
ros::Publisher rawCloudPub;
std::shared_ptr<SlamWrapper> slam;
size_t numAccumulatedRangeDataCount = 0;
size_t numPointCloudsReceived_ = 0;
size_t numAccumulatedRangeDataDesired = 1;
open3d::geometry::PointCloud accumulatedCloud;
bool isProcessAsFastAsPossible = false;

void processCloud(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp,
		const std::string &frame) {
	//std::cout << "received" << std::endl;

	accumulatedCloud += cloud;
	++numAccumulatedRangeDataCount;
	if (numAccumulatedRangeDataCount < numAccumulatedRangeDataDesired) {
		return;
	}

	if (numPointCloudsReceived_ < 5){
		++numPointCloudsReceived_;
		return;
		// somehow the first cloud can be missing a lot of points when running with ouster os-128 on the robot
		// if we skip that first measurement, it all works okay
		// we skip first five, just to be extra safe
	}

	if (accumulatedCloud.IsEmpty()){
		std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
		return;
	}
	slam->addRangeScan(accumulatedCloud, fromRos(timestamp));


	if (rawCloudPub.getNumSubscribers() > 0) {
		if (isProcessAsFastAsPossible) {
			std::pair<PointCloud, Time> cloudTimePair = slam->getLatestRegisteredCloudTimestampPair();
			const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
			if (isTimeValid(cloudTimePair.second) && !isCloudEmpty) {
				o3d_slam::publishCloud(cloudTimePair.first, o3d_slam::frames::rangeSensorFrame,
						toRos(cloudTimePair.second), rawCloudPub);
			}
		} else {
			open3d::geometry::PointCloud rawCloud= accumulatedCloud;
			o3d_slam::publishCloud(rawCloud, o3d_slam::frames::rangeSensorFrame, timestamp, rawCloudPub);
		} // end isProcessAsFastAsPossible
	} // end rawCloudPub.getNumSubscribers() > 0

	numAccumulatedRangeDataCount = 0;
	accumulatedCloud.Clear();
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	const ros::Time timestamp = msg->header.stamp;
	processCloud(cloud, timestamp, msg->header.frame_id);
}

void readRosbag(const rosbag::Bag &bag, const std::string &cloudTopic) {
	std::vector<std::string> topics;
	topics.push_back(cloudTopic);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	Timer rosbagTimer;
	ros::Time lastTimestamp;
	bool isFirstMessage = true;
	Timer rosbagProcessingTimer;
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		if (m.getTopic() == cloudTopic || ("/" + m.getTopic() == cloudTopic)) {
			sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
			if (cloud != nullptr) {
				if (isFirstMessage) {
					isFirstMessage = false;
					lastTimestamp = cloud->header.stamp;
				}
//      	std::cout << "reading cloud msg with seq: " << cloud->header.seq << std::endl;
				while (true) {
					const bool isOdomBufferFull = slam->getOdometryBufferSize() + 1
							>= slam->getOdometryBufferSizeLimit();
					const bool isMappingBufferFull = slam->getMappingBufferSize() + 1
							>= slam->getMappingBufferSizeLimit();

					if (!isOdomBufferFull && !isMappingBufferFull) {
						cloudCallback(cloud);
						break;
					} else {
						std::this_thread::sleep_for(std::chrono::milliseconds(20));
					}
					ros::spinOnce();
					if (!ros::ok()){
						slam->stopWorkers();
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
		if (!ros::ok()){
			slam->stopWorkers();
			return;
		}
	} // end foreach

	const ros::Time bag_begin_time = view.getBeginTime();
	const ros::Time bag_end_time = view.getEndTime();
	std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
			<< " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";
	slam->finishProcessing();
} // end readRosbag

} // namespace
int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));

	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	std::cout << "Cloud topic is given as " << cloudTopic << std::endl;
	isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
	numAccumulatedRangeDataDesired = nh->param<int>("num_accumulated_range_data", 1);
	std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired << std::endl;

	rawCloudPub = nh->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);

	slam = std::make_shared<SlamWrapperRos>(nh);
	slam->loadParametersAndInitialize();
	slam->startWorkers();

	ros::Subscriber cloudSub;
		if (!isProcessAsFastAsPossible) {
			cloudSub = nh->subscribe(cloudTopic, 100, &cloudCallback);

		} else {
			//handle rosbag
			const std::string rosbagFilename = nh->param<std::string>("rosbag_filepath", "");
			std::cout << "Reading from rosbag: " << rosbagFilename << "\n";
			rosbag::Bag bag;
			bag.open(rosbagFilename, rosbag::bagmode::Read);
			readRosbag(bag, cloudTopic);
			bag.close();
		}


	ros::spin();

	slam->stopWorkers();

	return 0;
}

