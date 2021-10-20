/*
 * helpers_ros.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/helpers_ros.hpp"
// ros stuff
#include "open3d_conversions/open3d_conversions.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {


void publishMesh(const open3d::geometry::MeshBase &mesh, const std::string &frame_id, const ros::Time &timestamp,
		ros::Publisher &pub) {
	if (pub.getNumSubscribers() > 0) {
		m545_volumetric_mapping_msgs::PolygonMesh meshMsg;
		open3d_conversions::open3dToRos(mesh, frame_id, meshMsg);
		meshMsg.header.frame_id = frame_id;
		meshMsg.header.stamp = timestamp;
		pub.publish(meshMsg);
	}
}

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,
		ros::Publisher &pub) {
	if (pub.getNumSubscribers() > 0) {
		sensor_msgs::PointCloud2 msg;
		open3d_conversions::open3dToRos(cloud, msg, frame_id);
		msg.header.stamp = timestamp;
		pub.publish(msg);
	}
}

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster) {
	geometry_msgs::TransformStamped transformStamped = m545_mapping::toRos(Mat, time, frame, childFrame);
	broadcaster->sendTransform(transformStamped);
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

} /* namespace m545_mapping */
