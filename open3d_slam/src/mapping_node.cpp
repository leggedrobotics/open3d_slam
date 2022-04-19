/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/ColorProjection.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace {
using namespace o3d_slam;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher rawCloudPub;
std::shared_ptr<SlamWrapper> slam;
size_t numAccumulatedRangeDataCount = 0;
size_t numPointCloudsReceived_ = 0;
size_t numAccumulatedRangeDataDesired = 1;
open3d::geometry::PointCloud accumulatedCloud;
o3d_slam::ProjectionParameters projectionParams;
std::shared_ptr<o3d_slam::ColorProjection> colorProjectionPtr_;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
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
//	o3d_slam::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, frames::rangeSensorFrame, frame + "_o3d",
//			tfBroadcaster.get());

	if (rawCloudPub.getNumSubscribers() > 0) {
		if (isProcessAsFastAsPossible) {
			std::pair<PointCloud, Time> cloudTimePair = slam->getLatestRegisteredCloudTimestampPair();
			const bool isTimeValid = toUniversal(cloudTimePair.second) > 0;
			const bool isCloudEmpty = cloudTimePair.first.IsEmpty();
			if (isTimeValid && !isCloudEmpty) {
				o3d_slam::publishCloud(cloudTimePair.first, o3d_slam::frames::rangeSensorFrame,
						toRos(cloudTimePair.second), rawCloudPub);
			}
		} else {
			open3d::geometry::PointCloud rawCloud;
			if (colorProjectionPtr_ == nullptr) {
				rawCloud = accumulatedCloud;
			} else {
				rawCloud = colorProjectionPtr_->filterColor(accumulatedCloud);
			}

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

	isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
	numAccumulatedRangeDataDesired = nh->param<int>("num_accumulated_range_data", 1);
	std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired << std::endl;

	rawCloudPub = nh->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);

	slam = std::make_shared<SlamWrapper>(nh);
	slam->initialize();
	slam->startWorkers();

	ros::Subscriber cloudSub;
	std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> syncPtr;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(*nh, cloudTopic, 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, cameraTopic, 1);
	if (useCameraRgbFlag) {
		const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
		o3d_slam::loadParameters(paramFile, &projectionParams);
		colorProjectionPtr_ = std::make_shared<o3d_slam::ColorProjection>();
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

	ros::spin();

	slam->stopWorkers();

	return 0;
}

