/*
 * RosbagRangeDataProcessorRos.hpp
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
#include <rosbag/bag.h>

namespace o3d_slam {

class RosbagRangeDataProcessorRos : public DataProcessorRos  {

	using BASE = DataProcessorRos;
public:
	RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh);
	~RosbagRangeDataProcessorRos() override = default;

	 void initialize() override;
	 void startProcessing() override;
	 void processMeasurement(const PointCloud &cloud, const Time &timestamp) override;

private:
	 void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
	 void readRosbag(const rosbag::Bag &bag);

	std::string rosbagFilename_;
};

} // namespace o3d_slam
