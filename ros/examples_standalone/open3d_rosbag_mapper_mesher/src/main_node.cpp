/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */

// Open3D
#include <open3d/Open3D.h>

// Open3D SLAM
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

// Package
#include "open3d_rosbag_mapper_mesher/RosbagMapperRos.h"

int main(int argc, char** argv) {
  using namespace o3d_slam;

  // ROS
  ros::init(argc, argv, "open3d_slam");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // ROS configuration
  // Paths
  const std::string paramFolderPath = tryGetParam<std::string>("parameter_folder_path", *nh);
  const std::string paramFilename = o3d_slam::tryGetParam<std::string>("parameter_filename", *nh);
  const std::string mapSavingFolderPath = tryGetParam<std::string>("map_saving_folder_path", *nh);
  const std::string mapSavingFilename = tryGetParam<std::string>("map_saving_filename", *nh);
  // Bools
  const bool isProcessAsFastAsPossible = o3d_slam::tryGetParam<bool>("is_read_from_rosbag", *nh);

  // The LUA parameters are loaded twice. This is the first time. Solely because we need to know if we are using a map for initialization.
  //  SlamParameters params;
  //  io_lua::loadParameters(paramFolderPath, paramFilename, &params);

  // This is where the initial class is constructed and passed on
  std::shared_ptr<DataProcessorRos> dataProcessorPtr;
  if (isProcessAsFastAsPossible) {
    dataProcessorPtr = std::make_shared<RosbagMapperRos>(nh);
  } else {
    throw std::logic_error("Not implemented yet. Can only read from rosbag.");
  }
  dataProcessorPtr->initialize();

  // Start processing
  dataProcessorPtr->startProcessing();

  // Wrap up
  std::cout << "Finished processing. Exiting... \n";
  return 0;
}
