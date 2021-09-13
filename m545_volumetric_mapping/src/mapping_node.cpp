/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

open3d::geometry::PointCloud cloud;
open3d::geometry::PointCloud cloudPrev;
ros::NodeHandlePtr nh;
bool isNewCloudReceived = false;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {

	open3d_conversions::rosToOpen3d(msg, cloud, true);
	isNewCloudReceived = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "m545_mapping_node");
	nh.reset(new ros::NodeHandle("~"));

	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	ros::Subscriber cloudSub = nh->subscribe(cloudTopic, 1, &cloudCallback);

	ros::Rate r(100.0);
	while (ros::ok()) {

		if (isNewCloudReceived) {
			isNewCloudReceived = false;

			if (cloudPrev.IsEmpty()) {
				cloudPrev = cloud;
				continue;
			}
			const double maxCorrespondenceDistance = 10.0;
			auto result = open3d::pipelines::registration::RegistrationICP(cloud, cloudPrev, maxCorrespondenceDistance);
			// source is cloud
			// target is cloudPrev

			cloudPrev = cloud;
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

