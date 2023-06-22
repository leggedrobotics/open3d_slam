/*
 * minimal_example.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */



#include <ros/package.h>
#include <iostream>
#include <memory>
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#define watch(x) std::cout << "variable: \033[1;32m" << (#x) << "\033[0m is " << (x) << std::endl


using namespace o3d_slam;
using namespace io_lua;

int main(int argc, char** argv) {

  const std::string folderPath = ros::package::getPath("open3d_slam_lua_io") + "/example_param";

  SlamParameters param;
  io_lua::loadParameters(folderPath, "configuration.lua", &param);

  watch(param.odometry_.scanProcessing_.voxelSize_);
  watch(param.odometry_.scanProcessing_.downSamplingRatio_);
  watch(param.odometry_.scanProcessing_.cropper_.croppingMaxRadius_);
  watch(param.odometry_.scanProcessing_.cropper_.croppingMinRadius_);
  watch(param.odometry_.scanMatcher_.icp_.maxCorrespondenceDistance_);
  std::cout << "\n";
  watch(param.mapper_.scanMatcher_.icp_.maxCorrespondenceDistance_);
  watch(param.mapper_.scanProcessing_.voxelSize_);
  watch(param.mapper_.scanProcessing_.downSamplingRatio_);
  watch(param.mapper_.scanProcessing_.cropper_.croppingMaxRadius_);
  watch(param.mapper_.scanProcessing_.cropper_.croppingMinRadius_);

  std::cout << "All done" << std::endl;
  return 0;
}


