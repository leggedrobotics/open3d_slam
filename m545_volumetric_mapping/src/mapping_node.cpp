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

using namespace m545_mapping;

ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher rawCloudPub;
std::shared_ptr<WrapperRos> mapping;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	const ros::Time timestamp = msg->header.stamp;
	mapping->addRangeScan(cloud, fromRos(timestamp));
	m545_mapping::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, frames::rangeSensorFrame,
			msg->header.frame_id, tfBroadcaster.get());
	m545_mapping::publishCloud(cloud, m545_mapping::frames::rangeSensorFrame, timestamp, rawCloudPub);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	ros::Subscriber cloudSub = nh->subscribe(cloudTopic, 100, &cloudCallback);

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

