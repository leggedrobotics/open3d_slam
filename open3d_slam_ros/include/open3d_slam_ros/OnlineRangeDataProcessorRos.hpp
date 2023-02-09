/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

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

	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;

	ros::Timer posePublishingTimer_;
	ros::Publisher scan2scanTransformPublisher_, scan2scanOdomPublisher_, scan2mapTransformPublisher_, scan2mapOdomPublisher_, scan2mapOdomPriorPublisher_ , consolidatedScan2mapOdomPublisher_;

	nav_msgs::Odometry getOdomMsg(const geometry_msgs::TransformStamped &transformMsg);
	geometry_msgs::TransformStamped getTransformMsg(const Transform &T, const ros::Time &timestamp, const std::string parent, const std::string child);

private:
	 void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	ros::Subscriber cloudSubscriber_;
	mutable std::mutex posePublishingMutex_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

} // namespace o3d_slam
