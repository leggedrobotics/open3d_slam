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

enum class MesherStrategy : int {
	AlphaShape,
	BallPivot,
	Poisson
};


static const std::map<std::string, MesherStrategy> mesherStrategyNames {
	{"AlphaShape",MesherStrategy::AlphaShape},
	{"BallPivot",MesherStrategy::BallPivot},
	{"Poisson",MesherStrategy::Poisson}
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
    Eigen::Vector3d translation;
    Eigen::Vector3d rpy;
};

struct MesherParameters{
	bool isComputeMesh_ = true;
	double voxelSize_ = 0.01;
	MesherStrategy strategy_ = MesherStrategy::AlphaShape;
	double alphaShapeAlpha_ = 0.5;
	int poissonDepth_ = 10;
	double poissonMinDensity_ = 5.0;
	float poissonScale_=1.1;
	std::vector<double> ballPivotRadii_ { 0.3, 1.0 };
	int knnNormalEstimation_ = 4;
};

struct MesherParamsInMesher{
    double overlapVoxelSize = 0.5;
    double overlapMinPoints = 10;
    double radiusOutlierNbPoints = 15;
    double radiusOutlierRadius = 0.5;
    double statisticalOutlierNbPoints = 15;
    double statisticalOutlierRatio = 1.0;
    double densityThreshold = 2.0;
    double computeOverlappingThreshold = 2.0;
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
void loadParameters(const YAML::Node &n, MesherParameters *p, MesherParamsInMesher *p2);
void loadParameters(const std::string &filename, MesherParameters *p, MesherParamsInMesher *p2);

} // namespace m545_mapping
