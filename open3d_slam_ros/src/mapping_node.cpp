/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"


int main(int argc, char **argv) {
	using namespace o3d_slam;

	ros::init(argc, argv, "lidar_odometry_mapping_node");
	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

	const std::string paramFolderPath = nh->param<std::string>("parameter_folder_path", "");
	const std::string paramFilename = nh->param<std::string>("parameter_filename", "");

	SlamParameters params;
	io_lua::loadParameters(paramFolderPath, paramFilename, &params);

	const bool isProcessAsFastAsPossible = nh->param<bool>("is_read_from_rosbag", false);
	std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
	std::cout << "Is use a map for initialization: " << std::boolalpha << params.mapper_.isUseInitialMap_ << "\n";

	std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(nh, isProcessAsFastAsPossible);
	dataProcessor->initialize();

	std::shared_ptr<SlamMapInitializer> slamMapInitializer;
	if (params.mapper_.isUseInitialMap_) {
		std::shared_ptr<SlamWrapper> slam = dataProcessor->getSlamPtr();
		slamMapInitializer = std::make_shared<SlamMapInitializer>(slam, nh);
		slamMapInitializer->initialize(params.mapper_.mapInit_);
	}

	dataProcessor->startProcessing();

	return 0;
}

