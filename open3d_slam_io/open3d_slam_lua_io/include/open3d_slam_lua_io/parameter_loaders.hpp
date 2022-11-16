/*
 * parameter_loaders.hpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */

#pragma once
#include "open3d_slam/Parameters.hpp"


namespace o3d_slam {
namespace io_lua{

void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p);

} // namespace io_lua
} // namespace o3d_slam
