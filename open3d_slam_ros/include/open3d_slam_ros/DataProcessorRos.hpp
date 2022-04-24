/*
 * DataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/time.hpp"
#include <ros/ros.h>

namespace o3d_slam {

class DataProcessorRos {

public:
	DataProcessorRos(ros::NodeHandlePtr nh);
	virtual ~DataProcessorRos() = default;

	virtual void initialize() = 0;
	virtual void startProcessing() = 0;
	virtual void processMeasurement(const PointCloud &cloud, const Time &timestamp);
	void accumulateAndProcessRangeData(const PointCloud &cloud, const Time &timestamp);
	void initCommonRosStuff();


protected:
	size_t numAccumulatedRangeDataCount_ = 0;
	size_t numPointCloudsReceived_ = 0;
	size_t numAccumulatedRangeDataDesired_ = 1;
	PointCloud accumulatedCloud_;
	ros::Publisher rawCloudPub_;
	std::string cloudTopic_;
	ros::NodeHandlePtr nh_;

};

} // namespace o3d_slam
