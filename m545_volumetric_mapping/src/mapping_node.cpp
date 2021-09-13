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
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

open3d::geometry::PointCloud cloud;
open3d::geometry::PointCloud cloudPrev;
ros::NodeHandlePtr nh;
bool isNewCloudReceived = false;
ros::Time timestamp;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	cloud.Clear();
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	timestamp = msg->header.stamp;
	isNewCloudReceived = true;
}

geometry_msgs::Pose getPose(const Eigen::MatrixXd &T) {
	geometry_msgs::Pose pose;

	// Fill pose
	Eigen::Affine3d eigenTr;
	eigenTr.matrix() = T;
	tf::poseEigenToMsg(eigenTr, pose);

	return pose;
}

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame) {

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = time;
	transformStamped.header.frame_id = frame;
	transformStamped.child_frame_id = childFrame;
	const auto pose = getPose(Mat);
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation = pose.orientation;
	return transformStamped;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "m545_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
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
			const auto startTime = std::chrono::steady_clock::now();
			const Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
			const auto metric = open3d::pipelines::registration::TransformationEstimationPointToPoint(false);
			auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria();
			criteria.max_iteration_ = 50;
			auto result = open3d::pipelines::registration::RegistrationICP(cloud, cloudPrev, maxCorrespondenceDistance,
					init, metric, criteria);
			const auto endTime = std::chrono::steady_clock::now();
			const double nMsec = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()
					/ 1e3;

			std::cout << "Scan matching finished \n";
			std::cout << "Time elapsed: " << nMsec << " msec \n";
			std::cout << "Fitness: " << result.fitness_ << "\n";
			std::cout << "Transform: " << result.transformation_ << "\n";
			std::cout << "\n \n";

			geometry_msgs::TransformStamped transformStamped = toRos(result.transformation_, timestamp, "odom", "range_sensor");
			tfBroadcaster->sendTransform(transformStamped);

			// source is cloud
			// target is cloudPrev
			cloudPrev.Clear();
			cloudPrev = cloud;
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

