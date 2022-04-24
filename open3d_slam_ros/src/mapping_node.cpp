/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam_ros/creators.hpp"


int main(int argc, char **argv) {
	using namespace o3d_slam;

	ros::init(argc, argv, "lidar_odometry_mapping_node");
	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";

	std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(nh, isProcessAsFastAsPossible);

	dataProcessor->initialize();

	dataProcessor->startProcessing();

	return 0;
}

