/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <rclcpp/rclcpp.hpp>
#include <open3d/Open3D.h>
#include <filesystem>

#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_yaml_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam_ros/creators.hpp"

int main(int argc, char** argv) {
  using namespace o3d_slam;

  rclcpp::init(argc, argv);
  {
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("open3d_slam", options);

    const std::string paramFolderPath = tryGetParam<std::string>("parameter_folder_path", *node);
    const std::string paramFilename = o3d_slam::tryGetParam<std::string>("parameter_filename", *node);
    const std::string parameterFilePath = (std::filesystem::path(paramFolderPath) / paramFilename).string();

    // Parameters are loaded twice because we need the map-initialization settings before constructing the processing pipeline.
    SlamParameters params;
    io_yaml::loadParameters(parameterFilePath, &params);

    const bool isProcessAsFastAsPossible = o3d_slam::tryGetParam<bool>("is_read_from_rosbag", *node);
    std::cout << "Is process as fast as possible: " << std::boolalpha << isProcessAsFastAsPossible << "\n";
    std::cout << "Is use a map for initialization: " << std::boolalpha << params.mapper_.isUseInitialMap_ << "\n";

    // This is where the initial class is constructed and passed on.
    std::shared_ptr<DataProcessorRos> dataProcessor = dataProcessorFactory(node, isProcessAsFastAsPossible);
    dataProcessor->initialize();

    std::shared_ptr<SlamWrapper> slam = dataProcessor->getSlamPtr();
    std::shared_ptr<SlamMapInitializer> slamMapInitializer;
    if (params.mapper_.isUseInitialMap_) {
      params.mapper_.mapInit_.frameId_ = slam->getFrames().mapFrame;
      slamMapInitializer = std::make_shared<SlamMapInitializer>(slam, node);
      slamMapInitializer->initialize(params.mapper_.mapInit_);
    }

    dataProcessor->startProcessing();
  }
  rclcpp::shutdown();

  return 0;
}
