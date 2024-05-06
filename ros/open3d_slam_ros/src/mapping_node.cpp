/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "open3d/Open3D.h"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam_ros/creators.hpp"

class Open3DSlamNode : public rclcpp::Node {
public:
  Open3DSlamNode() : Node("open3d_slam") {
    auto paramFolderPath = this->declare_parameter<std::string>("parameter_folder_path");
    auto paramFilename = this->declare_parameter<std::string>("parameter_filename");

    // The LUA parameters are loaded twice. This is the first time. Solely because we need to know if we are using a map for initialization.
    o3d_slam::SlamParameters params;
    o3d_slam::io_lua::loadParameters(paramFolderPath, paramFilename, &params);

    auto isProcessAsFastAsPossible = this->declare_parameter<bool>("is_read_from_rosbag");
    RCLCPP_INFO(this->get_logger(), "Is process as fast as possible: %s", isProcessAsFastAsPossible ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Is use a map for initialization: %s", params.mapper_.isUseInitialMap_ ? "true" : "false");

    executor_.add_node(this->get_node_base_interface());

    // This is where the initial class is constructed and passed on.
    dataProcessor_ = o3d_slam::dataProcessorFactory(this, &executor_, isProcessAsFastAsPossible);
    dataProcessor_->initialize();

    if (params.mapper_.isUseInitialMap_) {
      auto slam = dataProcessor_->getSlamPtr();
      slamMapInitializer_ = std::make_shared<o3d_slam::SlamMapInitializer>(slam, this, &executor_);
      slamMapInitializer_->initialize(params.mapper_.mapInit_);
    }

    dataProcessor_->startProcessing();
  }

private:
  std::shared_ptr<o3d_slam::DataProcessorRos> dataProcessor_;
  std::shared_ptr<o3d_slam::SlamMapInitializer> slamMapInitializer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Open3DSlamNode>();
  rclcpp::shutdown();
  return 0;
}
