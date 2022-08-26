/*
 * Parameters.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam_ros/Parameters.hpp"

namespace o3d_slam {


void loadParameters(const std::string& filename, MapInitializingParameters *p)
{
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		std::string error_msg = typeid(p).name();
		error_msg.append(" params::loadParameters loading failed");
		throw std::runtime_error(error_msg);
	}

	if (basenode["map_intializer"].IsDefined()) {
		loadParameters(basenode["map_intializer"], p);
	}
}

void loadParameters(const YAML::Node& node, MapInitializingParameters* p) {
	p->frameId_ = node["frame_id"].as<std::string>();
	p->meshFilePath_ = node["mesh_file_path"].as<std::string>();
	p->pcdFilePath_ = node["pcd_file_path"].as<std::string>();
	if (node["marker_pose"].IsDefined()) {
		loadParameters(node["marker_pose"], &(p->initialMarkerPose_));
	}
}


void loadParameters(const std::string& filename, NodeParameters *p)
{
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		std::string error_msg = typeid(p).name();
		error_msg.append(" params::loadParameters loading failed");
		throw std::runtime_error(error_msg);
	}

	if (basenode["ros_node"].IsDefined()) {
		loadParameters(basenode["ros_node"], p);
	}
}

void loadParameters(const YAML::Node& node, NodeParameters* p) {
	p->isInitializeMap_ = node["is_initialize_map"].as<bool>();
	if (node["map_intializer"].IsDefined()){
	  loadParameters(node["map_intializer"], &(p->MapInitializing_) );
	}
}

void loadParameters(const YAML::Node& node, geometry_msgs::Pose* p) {
	if (node["position"].IsDefined()){
	  loadParameters(node["position"], &(p->position) );
	}
	if (node["orientation"].IsDefined()){
	  loadParameters(node["orientation"], &(p->orientation) );
	}
}

void loadParameters(const YAML::Node& node, geometry_msgs::Point* p) {
	if (node["x"].IsDefined()){
	  p->x = node["x"].as<float>();
	}
	if (node["y"].IsDefined()) {
		p->y = node["y"].as<float>();
	}
	if (node["z"].IsDefined()){
	  p->z = node["z"].as<float>();
	}
}

void loadParameters(const YAML::Node& node, geometry_msgs::Quaternion* p) {
	if (node["x"].IsDefined()){
	  p->x = node["x"].as<float>();
	}
	if (node["y"].IsDefined()) {
		p->y = node["y"].as<float>();
	}
	if (node["z"].IsDefined()){
	  p->z = node["z"].as<float>();
	}
	if (node["w"].IsDefined()){
	  p->w = node["w"].as<float>();
	}
}

} // namespace o3d_slam

