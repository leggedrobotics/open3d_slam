/*
 * Parameters.hpp
 *
 *  Created on: Aug 24, 2022
 *      Author: lukaszpi
 */

#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include "open3d_slam/Parameters.hpp"

namespace o3d_slam {



struct MapInitializingParameters {
  std::string frameId_ = "";
  geometry_msgs::Pose initialMarkerPose_;
  std::string meshFilePath_ = "";
	std::string pcdFilePath_ = "";
	bool isUseInteractiveMarker_= false;
};

//todo move these guys
struct MapperParametersWithInitialization : public MapperParameters  {
	MapInitializingParameters mapInitParameters_;
};



// TODO(lukaszpi): Use template from open3d_slam
void loadParameters(const std::string& filename, MapperParametersWithInitialization* p);
void loadParameters(const YAML::Node &node, MapperParametersWithInitialization *p);
void loadParameters(const YAML::Node &node, MapInitializingParameters *p);
void loadParameters(const YAML::Node &node, geometry_msgs::Pose *p);
void loadParameters(const YAML::Node &node, geometry_msgs::Point *p);
void loadParameters(const YAML::Node &node, geometry_msgs::Quaternion *p);

} // namespace o3d_slam
