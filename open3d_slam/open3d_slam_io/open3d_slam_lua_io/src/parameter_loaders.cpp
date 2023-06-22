/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */



#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_lua_io/LuaLoader.hpp"
#include "open3d_slam_lua_io/helpers.hpp"

#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include <vector>

namespace o3d_slam {
namespace io_lua {



void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p){
	LuaLoader loader;
	loader.setupDictionary(topLevelFileName, folderpath);
	loader.loadParameters(folderpath, topLevelFileName, p);
	if (!loader.isLoadingOkay()){
		std::cout << "\n	Some parameters could not be loaded or loaded multiple times \n";
	}
}



} // namespace io_lua
} // namespace o3d_slam
