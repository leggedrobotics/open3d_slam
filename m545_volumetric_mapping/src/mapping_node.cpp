/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "m545_volumetric_mapping/WrapperRos.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/frames.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace {
using namespace m545_mapping;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher rawCloudPub;
std::shared_ptr<WrapperRos> mapping;
size_t numAccumulatedRangeDataCount = 0;
size_t numAccumulatedRangeDataDesired = 1;
open3d::geometry::PointCloud accumulatedCloud;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	accumulatedCloud += cloud;
	++numAccumulatedRangeDataCount;
	if (numAccumulatedRangeDataCount < numAccumulatedRangeDataDesired) {
		return;
	}
	const ros::Time timestamp = msg->header.stamp;
	mapping->addRangeScan(accumulatedCloud, fromRos(timestamp));
	m545_mapping::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, frames::rangeSensorFrame,
			msg->header.frame_id, tfBroadcaster.get());
	m545_mapping::publishCloud(accumulatedCloud, m545_mapping::frames::rangeSensorFrame, timestamp,
			rawCloudPub);
	numAccumulatedRangeDataCount = 0;
	accumulatedCloud.Clear();
}

void readRosbag(const rosbag::Bag &bag, const std::string &cloudTopic) {
	std::vector<std::string> topics;
	topics.push_back(cloudTopic);
	rosbag::View view(bag, rosbag::TopicQuery(topics));

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
					const bool isOdomBufferFull = mapping->getOdometryBufferSize() + 1
							>= mapping->getOdometryBufferSizeLimit();
					const bool isMappingBufferFull = mapping->getMappingBufferSize() + 1
							>= mapping->getMappingBufferSizeLimit();

					if (!isOdomBufferFull && !isMappingBufferFull) {
						cloudCallback(cloud);
						break;
					} else {
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}
					ros::spinOnce();
				} // end while

				const double elapsedWallTime = rosbagProcessingTimer.elapsedSec();
				if (elapsedWallTime > 15.0) {
					const double elapsedRosbagTime = (cloud->header.stamp - lastTimestamp).toSec();
					std::cout << "ROSBAG PLAYER: Rosbag messages pulsed at: " << 100.0*elapsedRosbagTime / elapsedWallTime << " % realtime speed \n";
					rosbagProcessingTimer.reset();
					lastTimestamp = cloud->header.stamp;
				}

				ros::spinOnce();

			}
		} // end if
	} // end foreach
} // end readRosbag

} // namespace
int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);

	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	numAccumulatedRangeDataDesired = nh->param<int>("num_accumulated_range_data", 1);
	rawCloudPub = nh->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);

	std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired << std::endl;

	mapping = std::make_shared<WrapperRos>(nh);
	mapping->initialize();
	mapping->start();

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

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

