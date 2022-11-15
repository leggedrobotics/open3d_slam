/*
 * minimal_example.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */



#include <ros/package.h>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <set>
#include <stack>
#include "open3d_slam_lua_io/helpers.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#define watch(x) std::cout << "variable: \033[1;31m" << (#x) << "\033[0m is " << (x) << std::endl


using namespace o3d_slam;
using namespace io_lua;

int main(int argc, char** argv) {

  const std::string folderPath = ros::package::getPath("open3d_slam_lua_io") + "/example_param/";

  io_lua::LuaLoader loader;
	loader.setupDictionary("configuration.lua",folderPath);
	const DictPtr &dict = loader.getDict();
	loader.buildLuaParamList();

//	auto dict2 = dict->GetDictionary("saving");
//	std::cout << "isNullptr: " << (dict == nullptr) << std::endl;
//	std::cout << "isNullptr2: " << (dict2 == nullptr) << std::endl;
//	std::cout << dict2->ToString() << std::endl;
//	std::cout << "has key: " << dict2->HasKey("save_map") << std::endl;
//	std::cout << "value " << dict2->GetBool("save_map") << "\n";
//
//	std::cout << "is table " << dict2->IsTable("save_map")<< std::endl;
//	std::cout << "is table " << dict->IsTable("saving")<< std::endl;
//	std::cout << "value " << dict2->GetBool("save_map") << "\n";
//	std::cout << "value " << dict2->GetBool("save_at_mission_end") << "\n";
//	std::cout << "value " << dict2->GetBool("save_submaps") << "\n";
//	std::cout << "is table " << dict2->IsTable("save_at_mission_end")<< std::endl;
//	auto dict3 = dict->GetDictionary("saving");

  SlamParameters param;
  io_lua::loadParameters(folderPath, "configuration.lua", &param);


  watch(param.odometry_.scanProcessing_.voxelSize_);
  watch(param.odometry_.scanProcessing_.downSamplingRatio_);
  watch(param.odometry_.scanProcessing_.cropper_.cropperName_);
  watch(param.odometry_.scanProcessing_.cropper_.croppingMaxRadius_);

//  watch(param.saving_.isSaveAtMissionEnd_);
//  watch(param.saving_.isSaveMap_);
//  watch(param.saving_.isSaveSubmaps_);
//
//  watch(param.motionCompensation_.isSpinningClockwise_);
//  watch(param.motionCompensation_.isUndistortInputCloud_);
//	watch(param.motionCompensation_.numPosesVelocityEstimation_);
//	watch(param.motionCompensation_.scanDuration_);

  std::cout << "All done" << std::endl;
  return 0;
}


