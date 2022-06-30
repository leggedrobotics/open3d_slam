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
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/time.h>
#include "open3d_slam/time.hpp"


namespace o3d_slam {

class SubmapCollection;

void publishSubmapCoordinateAxes(const SubmapCollection &submaps, const std::string &frame_id,
		const ros::Time &timestamp, const ros::Publisher &pub);
geometry_msgs::Point createPoint(double x, double y, double z);
void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker);

void assembleColoredPointCloud(const SubmapCollection &submaps, open3d::geometry::PointCloud *cloud);

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);

geometry_msgs::Pose getPose(const Eigen::MatrixXd &T);

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame);

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster);
bool lookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time,const tf2_ros::Buffer &tfBuffer,
		Eigen::Isometry3d *transform);

ros::Time toRos(Time time);

Time fromRos(const ::ros::Time& time);

template<typename Msg>
void publishIfSubscriberExists(const Msg &msg, const ros::Publisher &pub){
    if (pub.getNumSubscribers() > 0){
        pub.publish(msg);
    }
}

} /* namespace o3d_slam */
