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

} // namespace m545_mapping

