/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam_ros/creators.hpp"

int main(int argc, char** argv) {
  using namespace o3d_slam;

  ros::init(argc, argv, "open3d_slam");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

	const std::string paramFolderPath = tryGetParam<std::string>("parameter_folder_path", *nh);
	const std::string paramFilename = o3d_slam::tryGetParam<std::string>("parameter_filename", *nh);
  const std::string mapLoadPath = nh->param<std::string>("pcd_file_path", "");

  // The LUA parameters are loaded twice. This is the first time. Soley because we need to know if we are using a map for initialization.
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params);

  const bool isProcessAsFastAsPossible = o3d_slam::tryGetParam<bool>("is_read_from_rosbag", *nh);
  std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
  std::cout << "Is use a map for initialization: " << std::boolalpha << params.mapper_.isUseInitialMap_ << "\n";
  if (!mapLoadPath.empty()) {
		params.mapper_.mapInit_.pcdFilePath_ = mapLoadPath;
	}
	if (params.mapper_.isUseInitialMap_) {
		std::cout << "Loading Map from: " <<  params.mapper_.mapInit_.pcdFilePath_ << "\n";
	}
  
  // This is where the initial class is constructed and passed on.
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
