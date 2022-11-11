/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */



#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_lua_io/LuaLoader.hpp"

#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"


namespace o3d_slam {
namespace io_lua {

namespace {


const double kDegToRad = M_PI / 180.0;

} //namespace




void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p){
	LuaLoader loader;
	loader.setupDictionary(topLevelFileName, folderpath);
	DictSharedPtr dict = loader.getDict();
	if (dict->HasKey("saving_parameters")){
		DictPtr subDict = dict->GetDictionary("saving_parameters");
		loadParameters(std::move(subDict), &p->saving_);
	} else {
		std::cout << "saving_parameters not defined \n";
	}
}

void loadParameters(const DictPtr dict, SavingParameters *p){
	p->isSaveAtMissionEnd_ = dict->GetBool("save_at_mission_end");
	p->isSaveMap_ = dict->GetBool("save_map");
	p->isSaveSubmaps_ = dict->GetBool("save_submaps");
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
