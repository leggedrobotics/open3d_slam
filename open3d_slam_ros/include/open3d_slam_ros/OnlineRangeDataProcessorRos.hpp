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

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos  {

	using BASE = DataProcessorRos;
public:
	OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh);
	~OnlineRangeDataProcessorRos() override = default;

	 void initialize() override;
	 void startProcessing() override;
	 void processMeasurement(const PointCloud &cloud, const Time &timestamp) override;

private:
	 void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

	ros::Subscriber cloudSubscriber_;

};

} // namespace o3d_slam
