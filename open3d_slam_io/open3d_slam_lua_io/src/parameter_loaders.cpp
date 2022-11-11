/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */



#include "open3d_slam_lua_io/parameter_loaders.hpp"

#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"


namespace o3d_slam {
namespace io_lua {

namespace {


const double kDegToRad = M_PI / 180.0;

} //namespace




void loadParameters(const std::string &filename, SlamParameters *p){
//	YAML::Node basenode = YAML::LoadFile(filename);
//	if (basenode.IsNull()) {
//		throw std::runtime_error("SlamParameters::loadParameters loading failed");
//	}
//	loadParameters(basenode, p);
}

void loadParameters(const std::string &filename, MapperParameters *p){
//	YAML::Node basenode = YAML::LoadFile(filename);
//	if (basenode.IsNull()) {
//		throw std::runtime_error("MapperParameters::loadParameters loading failed");
//	}
//	if (!basenode["mapping"].IsDefined()){
//		std::cout << "mapping not defined \n";
//		return;
//	}
//	loadParameters(basenode["mapping"], p);
}


} // namespace io_lua
} // namespace o3d_slam
