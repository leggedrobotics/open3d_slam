/*
 * parameter_loaders.hpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */

#pragma once
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam_lua_io/LuaLoader.hpp"


namespace o3d_slam {
namespace io_lua{

void loadParameters(const DictPtr dict, ConstantVelocityMotionCompensationParameters *p);
void loadParameters(const DictPtr dict, SavingParameters *p);
void loadParameters(const DictPtr dict, PlaceRecognitionConsistencyCheckParameters *p);
void loadParameters(const DictPtr dict, PlaceRecognitionParameters *p);
void loadParameters(const DictPtr dict, GlobalOptimizationParameters *p);
void loadParameters(const DictPtr dict, VisualizationParameters *p);
void loadParameters(const DictPtr dict, SubmapParameters *p);
void loadParameters(const DictPtr dict, ScanProcessingParameters *p);
void loadParameters(const DictPtr dict, IcpParameters *p);
void loadParameters(const DictPtr dict, CloudRegistrationParameters *p);
void loadParameters(const DictPtr dict, MapperParameters *p);
void loadParameters(const DictPtr dict, MapBuilderParameters *p);
void loadParameters(const DictPtr dict, OdometryParameters *p);
void loadParameters(const DictPtr dict, SpaceCarvingParameters *p);
void loadParameters(const DictPtr dict, ScanCroppingParameters *p);
void loadParameters(const DictPtr dict, ScanToMapRegistrationParameters *p);
void loadParameters(const DictPtr dict, MapInitializingParameters *p);
void loadParameters(const DictPtr dict, Eigen::Isometry3d* T);


void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p);

} // namespace io_lua
} // namespace o3d_slam
