/*
 * DataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace o3d_slam {

DataProcessorRos::DataProcessorRos(ros::NodeHandlePtr nh) :
		nh_(nh) {

}


void DataProcessorRos::initCommonRosStuff() {
	cloudTopic_ = nh_->param<std::string>("cloud_topic", "");
	std::cout << "Cloud topic is given as " << cloudTopic_ << std::endl;
	rawCloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);
	numAccumulatedRangeDataDesired_ = nh_->param<int>("num_accumulated_range_data", 1);
	std::cout << "Num accumulated range data: " << numAccumulatedRangeDataDesired_ << std::endl;
}

void DataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {
	std::cout << "Warning you have not implemented processMeasurement!!! \n";
}

std::shared_ptr<SlamWrapper> DataProcessorRos::getSlamPtr() {
	return slam_;
}

void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud &cloud, const Time &timestamp) {
	const size_t minNumCloudsReceived = magic::skipFirstNPointClouds;
	if (numPointCloudsReceived_ < minNumCloudsReceived) {
		++numPointCloudsReceived_;
		return;
		// somehow the first cloud can be missing a lot of points when running with ouster os-128 on the robot
		// if we skip that first measurement, it all works okay
		// we skip first five, just to be extra safe
	}

	accumulatedCloud_ += cloud;
	++numAccumulatedRangeDataCount_;
	if (numAccumulatedRangeDataCount_ < numAccumulatedRangeDataDesired_) {
		return;
	}

	if (accumulatedCloud_.IsEmpty()) {
		std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
		return;
	}

	processMeasurement(accumulatedCloud_, timestamp);

	numAccumulatedRangeDataCount_ = 0;
	accumulatedCloud_.Clear();
}

} // namespace o3d_slam

