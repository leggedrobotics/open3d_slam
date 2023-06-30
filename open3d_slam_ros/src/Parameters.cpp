/*
 * Parameters.hpp
 *
 *  Created on: Aug 24, 2022
 *      Author: lukaszpi
 */

#include "open3d_slam_ros/Parameters.hpp"
#include "open3d_slam/math.hpp"

namespace o3d_slam {
namespace{
 const double kDegToRad = M_PI / 180.0;
}

void loadParameters(const std::string& filename, MapperParametersWithInitialization *p)
{
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		std::string error_msg = typeid(p).name();
		error_msg.append(" params::loadParameters loading failed");
		throw std::runtime_error(error_msg);
	}

	loadParameters(filename,static_cast<MapperParameters*>(p));

	if (basenode["mapping"].IsDefined()) {
		loadParameters(basenode["mapping"], p);
	}
}

void loadParameters(const YAML::Node& node, MapperParametersWithInitialization* p) {
	if (node["map_intializer"].IsDefined()){
	  loadParameters(node["map_intializer"], &(p->mapInitParameters_) );
	}
}

void loadParameters(const YAML::Node& node, MapInitializingParameters* p) {
	p->frameId_ = node["frame_id"].as<std::string>();
	p->pcdFilePath_ = ros::package::getPath("impact_printing_common") + node["pcd_file_path"].as<std::string>();
	p->isInitializeInteractively_ = node["is_initialize_interactively"].as<bool>();
	if (node["init_pose"].IsDefined()) {
		loadParameters(node["init_pose"], &(p->initialMarkerPose_));
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
	double roll(0.0),pitch(0.0),yaw(0.0);
	if (node["roll"].IsDefined()){
	  roll = node["roll"].as<double>() * kDegToRad;
	}
	if (node["pitch"].IsDefined()) {
		pitch = node["pitch"].as<double>() * kDegToRad;
	}
	if (node["yaw"].IsDefined()){
	  yaw = node["yaw"].as<double>() * kDegToRad;
	}
	const Eigen::Quaterniond q = fromRPY(roll,pitch,yaw).normalized();
	p->x = q.x();
	p->y = q.y();
	p->z = q.z();
	p->w = q.w();

}

} // namespace o3d_slam

