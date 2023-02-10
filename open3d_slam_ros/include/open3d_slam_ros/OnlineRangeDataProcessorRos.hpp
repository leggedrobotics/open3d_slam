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

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos  {

	using BASE = DataProcessorRos;
public:
	OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh);
	~OnlineRangeDataProcessorRos() override = default;

	 void initialize() override;
	 void startProcessing() override;
	 void processMeasurement(const PointCloud &cloud, const Time &timestamp) override;

	 void getAndSetTfPrior(const Time &timestamp);

	void posePublisherTimerCallback(const ros::TimerEvent&);

	void registeredCloudPublisherCallback(const ros::TimerEvent&);

	bool alignWithWorld(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

	std::shared_ptr<o3d_slam::TrajectoryAlignmentHandler> trajectoryAlignmentHandlerPtr_;

	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;

	ros::Timer posePublishingTimer_;
	ros::Timer registeredEnuCloudPublisherTimer_;

	ros::Publisher scan2scanTransformPublisher_, scan2scanOdomPublisher_, scan2mapTransformPublisher_, scan2mapOdomPublisher_, scan2mapOdomPriorPublisher_ , consolidatedScan2mapOdomPublisher_;

	nav_msgs::Odometry getOdomMsg(const geometry_msgs::TransformStamped &transformMsg);
	geometry_msgs::TransformStamped getTransformMsg(const Transform &T, const ros::Time &timestamp, const std::string parent, const std::string child);

	void poseStampedPriorCallback(const geometry_msgs::PoseStampedConstPtr& odometryPose);

	Transform mapToWorld_;

  ros::Publisher lidarPath_;
  ros::Publisher gnssPath_;
  ros::Publisher registeredEnuCloud_;

  ros::Publisher before_gnssPath_;
  ros::Publisher before_lidarPath_;

  nav_msgs::PathPtr measMapGnssPathPtr_;
  nav_msgs::PathPtr measMapLidarPathPtr_;
  bool scanToMapStarted_ = false;

private:
	 void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	ros::ServiceServer alignWithWorld_;
	ros::Subscriber cloudSubscriber_;
	ros::Subscriber priorPoseSubscriber_;
	mutable std::mutex posePublishingMutex_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster2_;
};

} // namespace o3d_slam
