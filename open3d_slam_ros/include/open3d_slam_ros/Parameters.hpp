/*
 * Parameters.hpp
 *
 *  Created on: Aug 24, 2022
 *      Author: Lukasz Pietrasik
 */

#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>


namespace o3d_slam {


struct MapInitializingParameters {
	std::string frameId_ = "";
	std::string meshFilePath_ = "";
	std::string pcdFilePath_ = "";
};

struct NodeParameters {
  // TODO(lukaszpi): Add rest of parameters
  bool isInitializeMap_ = false;
  MapInitializingParameters MapInitializing_;
};

// TODO(lukaszpi): Use template from open3d_slam
void loadParameters(const std::string& filename, MapInitializingParameters* p);
void loadParameters(const YAML::Node &node, MapInitializingParameters *p);
void loadParameters(const std::string& filename, NodeParameters* p);
void loadParameters(const YAML::Node &node, NodeParameters *p);

} // namespace o3d_slam
