/*
 * Parameters.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */


#include "m545_volumetric_mapping/Parameters.hpp"


namespace m545_mapping {

void loadParameters(const std::string &filename, IcpOdometryParameters *p){

	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("IcpOdometryParameters::loadParameters loading failed");
	}

	loadParameters(basenode["icp_odometry"],p);
}


void loadParameters(const YAML::Node &n, IcpOdometryParameters *p){

	p->icpObjective_ = IcpObjectiveNames.at(n["icp_objective"].as<std::string>());
	p->kNNnormalEstimation_ = n["knn_normal_estimation"].as<int>();
	p->maxCorrespondenceDistance_ = n["max_correspondence_dist"].as<double>();
	p->maxNumIter_ = n["max_n_iter"].as<int>();
	p->downSamplingRatio_ =n["downsampling_ratio"].as<double>();
	p->cropBoxLowBound_.x() = n["crop_box_min_x"].as<double>();
	p->cropBoxLowBound_.y() = n["crop_box_min_y"].as<double>();
	p->cropBoxLowBound_.z() = n["crop_box_min_z"].as<double>();
	p->cropBoxHighBound_.x() = n["crop_box_max_x"].as<double>();
	p->cropBoxHighBound_.y() = n["crop_box_max_y"].as<double>();
	p->cropBoxHighBound_.z() = n["crop_box_max_z"].as<double>();

}


} // namespace m545_mapping

