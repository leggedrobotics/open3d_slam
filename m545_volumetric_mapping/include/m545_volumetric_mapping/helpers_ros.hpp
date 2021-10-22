/*
 * helpers_ros.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/MeshBase.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>



namespace m545_mapping {

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);
void publishMesh(const open3d::geometry::MeshBase &mesh, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);

geometry_msgs::Pose getPose(const Eigen::MatrixXd &T);

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame);

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster);
bool lookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time,const tf2_ros::Buffer &tfBuffer,
		Eigen::Isometry3d *transform);

} /* namespace m545_mapping */
