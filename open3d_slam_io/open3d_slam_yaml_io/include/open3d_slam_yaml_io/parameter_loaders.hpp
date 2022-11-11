/*
 * parameter_loaders.hpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */

#pragma once
#include <yaml-cpp/yaml.h>
#include "open3d_slam/Parameters.hpp"


namespace o3d_slam {
namespace io_yaml{

void loadParameters(const YAML::Node &node, ConstantVelocityMotionCompensationParameters *p);
void loadParameters(const YAML::Node &node, SavingParameters *p);
void loadParameters(const YAML::Node &node, PlaceRecognitionConsistencyCheckParameters *p);
void loadParameters(const YAML::Node &node, PlaceRecognitionParameters *p);
void loadParameters(const YAML::Node &node, GlobalOptimizationParameters *p);
void loadParameters(const YAML::Node &node, VisualizationParameters *p);
void loadParameters(const YAML::Node &node, SubmapParameters *p);
void loadParameters(const YAML::Node &node, ScanProcessingParameters *p);
void loadParameters(const YAML::Node &node, IcpParameters *p);
void loadParameters(const YAML::Node &node, CloudRegistrationParameters *p);
void loadParameters(const YAML::Node &node, MapperParameters *p);
void loadParameters(const YAML::Node &node, MapBuilderParameters *p);
void loadParameters(const YAML::Node &node, OdometryParameters *p);
void loadParameters(const YAML::Node &node, SpaceCarvingParameters *p);
void loadParameters(const YAML::Node &node, ScanCroppingParameters *p);
void loadParameters(const YAML::Node &node, ScanToMapRegistrationParameters *p);
void loadParameters(const YAML::Node &node, SlamParameters *p);
void loadParameters(const YAML::Node &node, MapInitializingParameters *p);
void loadParameters(const YAML::Node& node, Eigen::Isometry3d* T);


void loadParameters(const std::string &filename, MapperParameters *p);
void loadParameters(const std::string &filename, SlamParameters *p);

} // namespace io_yaml
} // namespace o3d_slam
