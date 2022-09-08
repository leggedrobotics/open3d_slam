/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/Parameters.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"


int main(int argc, char **argv) {
	using namespace o3d_slam;

	ros::init(argc, argv, "lidar_odometry_mapping_node");
	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

	const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
	NodeParameters params;
	loadParameters(paramFile, &params);

	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
	std::cout << "Initialize map: " << std::boolalpha << params.isInitializeMap_ << "\n";

	std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(nh, isProcessAsFastAsPossible);

	dataProcessor->initialize();

	if (params.isInitializeMap_) {
		std::shared_ptr<SlamWrapper> slam = dataProcessor->getSlamPtr();
		std::shared_ptr<SlamMapInitializer> slamMapInitializer = std::make_shared<SlamMapInitializer>(slam, nh);
		slamMapInitializer->initialize(params.MapInitializing_);
		std::cout << "Finished setting initial map! \n";
	}

	dataProcessor->startProcessing();

	return 0;
}

