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
#include <Eigen/Geometry>

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
	double voxelSize_ = 0.03;
	Eigen::Vector3d cropBoxLowBound_ = Eigen::Vector3d(-30.0,-30.0,-1e3);
	Eigen::Vector3d cropBoxHighBound_ = Eigen::Vector3d(30.0,30.0,1e3);

};

struct OdometryParameters : public IcpParameters {
	int everyKpoints_ = 1;
};


struct MapperParameters : public IcpParameters {
	double mapVoxelSize_ = 0.03;
	double minMovementBetweenMappingSteps_ = 0.0;
	double minRefinementFitness_ = 0.7;
	Eigen::Vector3d mapBuilderCropBoxLowBound_ = Eigen::Vector3d(-50.0,-50.0,-1e3);
	Eigen::Vector3d mapBuilderCropBoxHighBound_ = Eigen::Vector3d(50.0,50.0,1e3);
};

struct LocalMapParameters {
	double voxelSize_ = 0.1;
	Eigen::Vector3d cropBoxLowBound_ = Eigen::Vector3d(-50.0,-50.0,-1e3);
	Eigen::Vector3d cropBoxHighBound_ = Eigen::Vector3d(50.0,50.0,1e3);
};

struct ProjectionParameters {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K;
    Eigen::Matrix<double, 5, 1> D;
    Eigen::Quaternion<double> quaternion;
    Eigen::Vector3d translation;
//    Eigen::Matrix3d rotation;
//    Eigen::MatrixXd RT;
};

void loadParameters(const std::string &filename, IcpParameters *p);
void loadParameters(const YAML::Node &node, IcpParameters *p);
void loadParameters(const std::string &filename, MapperParameters *p);
void loadParameters(const YAML::Node &node, MapperParameters *p);
void loadParameters(const std::string &filename, LocalMapParameters *p);
void loadParameters(const YAML::Node &node, LocalMapParameters *p);
void loadParameters(const std::string &filename, OdometryParameters *p);
void loadParameters(const YAML::Node &node, OdometryParameters *p);
void loadParameters(const std::string &filename, ProjectionParameters *p);
void loadParameters(const YAML::Node &node, ProjectionParameters *p);

} // namespace m545_mapping
