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
}


} // namespace m545_mapping

