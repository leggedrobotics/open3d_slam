/*
 * Parameters.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Parameters.hpp"

namespace m545_mapping {

void loadParameters(const std::string &filename, IcpParameters *p) {

	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("IcpParameters::loadParameters loading failed");
	}

	loadParameters(basenode["icp_odometry"], p);
}

void loadParameters(const YAML::Node &n, IcpParameters *p) {

	p->icpObjective_ = IcpObjectiveNames.at(n["icp_objective"].as<std::string>());
	p->kNNnormalEstimation_ = n["knn_normal_estimation"].as<int>();
	p->maxCorrespondenceDistance_ = n["max_correspondence_dist"].as<double>();
	p->maxNumIter_ = n["max_n_iter"].as<int>();
	p->downSamplingRatio_ = n["downsampling_ratio"].as<double>();
	p->cropBoxLowBound_.x() = n["crop_box_min_x"].as<double>();
	p->cropBoxLowBound_.y() = n["crop_box_min_y"].as<double>();
	p->cropBoxLowBound_.z() = n["crop_box_min_z"].as<double>();
	p->cropBoxHighBound_.x() = n["crop_box_max_x"].as<double>();
	p->cropBoxHighBound_.y() = n["crop_box_max_y"].as<double>();
	p->cropBoxHighBound_.z() = n["crop_box_max_z"].as<double>();
	p->voxelSize_=n["voxel_filter_size"].as<double>();


}

void loadParameters(const std::string &filename, MapperParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("MapperParameters::loadParameters loading failed");
	}

	loadParameters(basenode["icp_mapping"], p);
}
void loadParameters(const YAML::Node &node, MapperParameters *p) {
	loadParameters(node["scan_to_map_refinement"], static_cast<IcpParameters*>(p));
	const auto n = node["map_builder"];
	p->mapBuilderCropBoxLowBound_.x() = n["crop_box_min_x"].as<double>();
	p->mapBuilderCropBoxLowBound_.y() = n["crop_box_min_y"].as<double>();
	p->mapBuilderCropBoxLowBound_.z() = n["crop_box_min_z"].as<double>();
	p->mapBuilderCropBoxHighBound_.x() = n["crop_box_max_x"].as<double>();
	p->mapBuilderCropBoxHighBound_.y() = n["crop_box_max_y"].as<double>();
	p->mapBuilderCropBoxHighBound_.z() = n["crop_box_max_z"].as<double>();

	p->mapVoxelSize_ = node["map_voxel_size"].as<double>();
	p->minRefinementFitness_ = node["min_refinement_fitness"].as<double>();
	p->minMovementBetweenMappingSteps_ = node["min_movement_between_mapping_steps"].as<double>();

}

void loadParameters(const std::string &filename, LocalMapParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("Local map::loadParameters loading failed");
	}

	loadParameters(basenode["local_map"], p);
}
void loadParameters(const YAML::Node &n, LocalMapParameters *p){
	p->voxelSize_ = n["voxel_size"].as<double>();
	p->cropBoxLowBound_.x() = n["crop_box_min_x"].as<double>();
	p->cropBoxLowBound_.y() = n["crop_box_min_y"].as<double>();
	p->cropBoxLowBound_.z() = n["crop_box_min_z"].as<double>();
	p->cropBoxHighBound_.x() = n["crop_box_max_x"].as<double>();
	p->cropBoxHighBound_.y() = n["crop_box_max_y"].as<double>();
	p->cropBoxHighBound_.z() = n["crop_box_max_z"].as<double>();
}

void loadParameters(const std::string &filename, OdometryParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("Odometry::loadParameters loading failed");
	}

	loadParameters(basenode["icp_odometry"], p);
}
void loadParameters(const YAML::Node &node, OdometryParameters *p){
	loadParameters(node, static_cast<IcpParameters*>(p));
	p->croppingRadius_ = node["cropping_radius"].as<double>();
}


void loadParameters(const YAML::Node &n, MesherParameters *p) {

	p->strategy_ = mesherStrategyNames.at(n["strategy"].as<std::string>());
	p->knnNormalEstimation_ = n["knn_normal_estimation"].as<int>();
	p->voxelSize_=n["voxel_size"].as<double>();
	p->alphaShapeAlpha_ = n["alpha_shape_alpha"].as<double>();
	p->poissonDepth_ = n["poisson_depth"].as<int>();
	p->poissonMinDensity_ = n["poisson_min_density"].as<double>();
	p->poissonScale_ = n["poisson_scale"].as<double>();
	p->ballPivotRadii_ = n["ball_pivot_radii"].as<std::vector<double>>();
	p->isComputeMesh_ = n["is_compute_mesh"].as<bool>();

}

void loadParameters(const std::string &filename, MesherParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("MesherParameters::loadParameters loading failed");
	}
	loadParameters(basenode["mesher"], p);
}

void loadParameters(const YAML::Node &n, SpaceCarvingParameters *p){
	p->voxelSize_ = n["voxel_size"].as<double>();
	p->maxRaytracingLength_ = n["max_raytracing_length"].as<double>();
	p->truncationDistance_ = n["truncation_distance"].as<double>();
	p->carveSpaceEveryNsec_ = n["carve_space_every_n_sec"].as<double>();
}
void loadParameters(const std::string &filename, SpaceCarvingParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("SpaceCarving::loadParameters loading failed");
	}
	loadParameters(basenode["space_carving"], p);
}

} // namespace m545_mapping

