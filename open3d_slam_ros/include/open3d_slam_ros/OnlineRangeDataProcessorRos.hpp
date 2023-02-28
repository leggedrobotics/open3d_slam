/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <memory>
#include "open3d_slam_ros/TrajectoryAlignmentHandler.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos  {

	using BASE = DataProcessorRos;
public:
	OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh);
	~OnlineRangeDataProcessorRos() override = default;

	 void initialize() override;
	 void startProcessing() override;
	 void processMeasurement(const PointCloud &cloud, const Time &timestamp) override;

	// Queires tf to get the odom to range sensor transform.
	bool getAndSetTfPrior(const Time &timestamp);

	void posePublisherTimerCallback(const ros::TimerEvent&);

	void registeredCloudPublisherCallback(const ros::TimerEvent&);

	// Handler for trajectory alingment. Copied from JNs code.
	std::shared_ptr<o3d_slam::TrajectoryAlignmentHandler> trajectoryAlignmentHandlerPtr_;

	// Publishers
	ros::Publisher scan2mapTransformPublisher_, scan2mapOdometryPublisher_, scan2mapOdometryPriorPublisher_ , consolidatedScan2mapOdomPublisher_, registeredCloudPub_;

	void poseStampedPriorCallback(const nav_msgs::Odometry::ConstPtr& odometryPose);

	// Publishers vol_2.
	ros::Publisher lidarPathPublisher_;
	ros::Publisher gnssPathPublisher_;
	ros::Publisher registeredEnuCloud_;
	ros::Publisher lidarPoseInMapPathPublisher_;
	ros::Publisher consolidatedLidarPoseInMapPathPublisher_;

	// Path messages
	nav_msgs::Path lidarPathInMap;
	nav_msgs::Path consolidatedLiDARpathInMap;

	// Container for map to world transform. In this case the world is the ENU frame. ENU frame is defined by the GNSS poses.
	Transform mapToWorld_;

private:
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	// Align with the world. In this case the world is the poses that were defined in ENU frame.
	bool alignWithWorld(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

	// Helper functions to convert between ROS and Open3D types.
	nav_msgs::Odometry getOdomMsg(const geometry_msgs::TransformStamped &transformMsg);
	geometry_msgs::TransformStamped getTransformMsg(const Transform &T, const ros::Time &timestamp, const std::string parent, const std::string child);

	// Helper function to check if the cloud was already published.
	bool isRegisteredCloudAlreadyPublished(const Time &timestamp);

	// Service to align with the world.
	ros::ServiceServer alignWithWorld_;

	// Subscribers
	ros::Subscriber cloudSubscriber_;
	ros::Subscriber priorPoseSubscriber_;

	// Mutex
	mutable std::mutex posePublishingMutex_;

	// Helper flag
	bool scanToMapStarted_ = false;

	// Ros timers
	ros::Timer posePublishingTimer_;
	ros::Timer registeredEnuCloudPublisherTimer_;

	// Tf
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;

	o3d_slam::Time lastPublishedRegisteredCloudTime_ = o3d_slam::Time::max();

};

} // namespace o3d_slam
