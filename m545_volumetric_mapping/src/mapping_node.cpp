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

void processCloud(const open3d::geometry::PointCloud& cloud, const ros::Time& timestamp, const std::string& frame) {
	accumulatedCloud += cloud;
	++numAccumulatedRangeDataCount;
	if (numAccumulatedRangeDataCount < numAccumulatedRangeDataDesired){
		return;
	}

	mapping->addRangeScan(accumulatedCloud, fromRos(timestamp));
	m545_mapping::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, frames::rangeSensorFrame,
			frame, tfBroadcaster.get());
	m545_mapping::publishCloud(accumulatedCloud, m545_mapping::frames::rangeSensorFrame, timestamp, rawCloudPub);
	numAccumulatedRangeDataCount = 0;
	accumulatedCloud.Clear();
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	const ros::Time timestamp = msg->header.stamp;

	processCloud(cloud, timestamp, msg->header.frame_id);
}

void synchronizeCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const sensor_msgs::ImageConstPtr& imageMsg) {

	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(cloudMsg, cloud, true);
	const ros::Time timestamp = cloudMsg->header.stamp;

	open3d::geometry::PointCloud coloredCloud = colorProjectionPtr_->projectionAndColor(cloud, imageMsg, projectionParams.K, projectionParams.D, projectionParams.rpy, projectionParams.translation, true);

	processCloud(coloredCloud, timestamp, cloudMsg->header.frame_id);
}

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
	ros::Subscriber cloudSubscriber;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> syncPtr;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(*nh, cloudTopic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, cameraTopic, 1);
	if (useCameraRgbFlag) {
		const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
		m545_mapping::loadParameters(paramFile, &projectionParams);
		colorProjectionPtr_ = std::make_shared<m545_mapping::ColorProjection>();
		syncPtr = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), cloud_sub, image_sub);
		syncPtr->registerCallback(boost::bind(&synchronizeCallback, _1, _2));
	}
	else {
		cloudSubscriber = nh->subscribe(cloudTopic, 100, &cloudCallback);
	}

	numAccumulatedRangeDataDesired = nh->param<int>("num_accumulated_range_data", 1);
	rawCloudPub = nh->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);

	mapping = std::make_shared<WrapperRos>(nh);
	mapping->initialize();
	mapping->start();
//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

