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
#include "m545_volumetric_mapping/ColorProjection.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace {
using namespace m545_mapping;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher rawCloudPub;
std::shared_ptr<WrapperRos> mapping;
size_t numAccumulatedRangeDataCount = 0;
size_t numAccumulatedRangeDataDesired = 1;
open3d::geometry::PointCloud accumulatedCloud;
m545_mapping::ProjectionParameters projectionParams;
std::shared_ptr<m545_mapping::ColorProjection> colorProjectionPtr_;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

void processCloud(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp,
		const std::string &frame) {
	//std::cout << "received" << std::endl;

	accumulatedCloud += cloud;
	++numAccumulatedRangeDataCount;
	if (numAccumulatedRangeDataCount < numAccumulatedRangeDataDesired) {
		return;
	}

	mapping->addRangeScan(accumulatedCloud, fromRos(timestamp));
	m545_mapping::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, frames::rangeSensorFrame, frame,
			tfBroadcaster.get());
	m545_mapping::publishCloud(accumulatedCloud, m545_mapping::frames::rangeSensorFrame, timestamp,
			rawCloudPub);
	numAccumulatedRangeDataCount = 0;
	accumulatedCloud.Clear();
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	const ros::Time timestamp = msg->header.stamp;

	processCloud(cloud, timestamp, msg->header.frame_id);
}

void synchronizeCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
		const sensor_msgs::ImageConstPtr &imageMsg) {

	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(cloudMsg, cloud, true);
	const ros::Time timestamp = cloudMsg->header.stamp;

	open3d::geometry::PointCloud coloredCloud = colorProjectionPtr_->projectionAndColor(cloud, imageMsg,
			projectionParams.K, projectionParams.D, projectionParams.rpy, projectionParams.translation, true);

	processCloud(coloredCloud, timestamp, cloudMsg->header.frame_id);
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
					const bool isOdomBufferFull = mapping->getOdometryBufferSize() + 1
							>= mapping->getOdometryBufferSizeLimit();
					const bool isMappingBufferFull = mapping->getMappingBufferSize() + 1
							>= mapping->getMappingBufferSizeLimit();

					if (!isOdomBufferFull && !isMappingBufferFull) {
						cloudCallback(cloud);
						break;
					} else {
						std::this_thread::sleep_for(std::chrono::milliseconds(20));
					}
					ros::spinOnce();
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

			}
		} // end if
	} // end foreach

	const ros::Time bag_begin_time = view.getBeginTime();
	const ros::Time bag_end_time = view.getEndTime();
	std::cout << "Rosbag processing finished. Rosbag duration: " << (bag_end_time - bag_begin_time).toSec()
			<< " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n";

} // end readRosbag

} // namespace
int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
// Topics
	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	std::cout << "Cloud topic is given as " << cloudTopic << std::endl;
	const std::string cameraTopic = nh->param<std::string>("image_topic", "");
	std::cout << "Camera topic is given as " << cameraTopic << std::endl;

// Subscribers
	const bool useCameraRgbFlag = nh->param<bool>("use_rgb_from_camera", "");
	std::cout << "Use RGB from the camera is set to " << useCameraRgbFlag << std::endl;
	/// if no cameraRgb

	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
	numAccumulatedRangeDataDesired = nh->param<int>("num_accumulated_range_data", 1);
	std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired << std::endl;

	rawCloudPub = nh->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);

	mapping = std::make_shared<WrapperRos>(nh);
	mapping->initialize();
	mapping->start();

	ros::Subscriber cloudSub;
	std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> syncPtr;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(*nh, cloudTopic, 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, cameraTopic, 1);
	if (useCameraRgbFlag) {
		const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
		m545_mapping::loadParameters(paramFile, &projectionParams);
		colorProjectionPtr_ = std::make_shared<m545_mapping::ColorProjection>();
		syncPtr = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(50), cloud_sub,
				image_sub);
		syncPtr->registerCallback(boost::bind(&synchronizeCallback, _1, _2));
	} else {
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
	}

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

