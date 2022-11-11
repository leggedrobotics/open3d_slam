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

int main(int argc, char** argv) {
  using namespace o3d_slam;
  const std::string folderPath = ros::package::getPath("open3d_slam_lua_io") + "/example_param/";
  SlamParameters param;
  io_lua::loadParameters(folderPath, "full_parameters.lua", &param);



  std::cout << "All done" << std::endl;
  return 0;
}


