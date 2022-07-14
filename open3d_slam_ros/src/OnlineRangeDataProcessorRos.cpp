/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) :
		BASE(nh) {

}

void OnlineRangeDataProcessorRos::initialize() {
	initCommonRosStuff();
	slam_ = std::make_shared<SlamWrapperRos>(nh_);
	slam_->loadParametersAndInitialize();
	
}

void OnlineRangeDataProcessorRos::startProcessing() {
	slam_->startWorkers();
	cloudSubscriber_ = nh_->subscribe(cloudTopic_, 100, &OnlineRangeDataProcessorRos::cloudCallback,this);
	ros::spin();
	slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {

	slam_->addRangeScan(cloud, timestamp);
  o3d_slam::publishCloud(cloud, o3d_slam::frames::rangeSensorFrame, toRos(timestamp), rawCloudPub_);

}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	const Time timestamp = fromRos(msg->header.stamp);
	accumulateAndProcessRangeData(cloud, timestamp);
}



} // namespace o3d_slam

