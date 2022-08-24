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

} // namespace o3d_slam

