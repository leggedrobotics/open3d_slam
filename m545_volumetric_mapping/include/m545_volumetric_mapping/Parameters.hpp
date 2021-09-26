/*
 * Parameters.hpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <Eigen/Dense>


namespace m545_mapping {

enum class IcpObjective : int {
	PointToPoint,
	PointToPlane
};

static const std::map<std::string, IcpObjective> IcpObjectiveNames {
	{"PointToPoint",IcpObjective::PointToPoint},
	{"PointToPlane",IcpObjective::PointToPlane}
};

struct IcpParameters {
	int kNNnormalEstimation_ = 5;
	int maxNumIter_ = 50;
	double maxCorrespondenceDistance_ = 0.2;
	IcpObjective icpObjective_ = IcpObjective::PointToPoint;
	double downSamplingRatio_ = 1.0;
	Eigen::Vector3d cropBoxLowBound_ = Eigen::Vector3d(-30.0,-30.0,-1e3);
	Eigen::Vector3d cropBoxHighBound_ = Eigen::Vector3d(30.0,30.0,1e3);

};

struct MapperParameters : public IcpParameters {

};

void loadParameters(const std::string &filename, IcpParameters *p);
void loadParameters(const YAML::Node &node, IcpParameters *p);
void loadParameters(const std::string &filename, MapperParameters *p);
void loadParameters(const YAML::Node &node, MapperParameters *p);



} // namespace m545_mapping
