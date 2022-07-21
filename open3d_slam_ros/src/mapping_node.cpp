/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"


int main(int argc, char **argv) {
	using namespace o3d_slam;

	ros::init(argc, argv, "lidar_odometry_mapping_node");
	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	const bool initializeMap = nh->param<bool>("initialize_map", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";

	std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(nh, isProcessAsFastAsPossible);

	dataProcessor->initialize();

	if (initializeMap) {
		std::shared_ptr<SlamWrapper> slam = dataProcessor->getSlamPtr();
		std::shared_ptr<SlamMapInitializer> slamMapInitializer = std::make_shared<SlamMapInitializer>(slam, nh);
		slamMapInitializer->initialize();
	}

	dataProcessor->startProcessing();

	return 0;
}

