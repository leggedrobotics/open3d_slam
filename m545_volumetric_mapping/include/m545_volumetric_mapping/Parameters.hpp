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
};

struct OdometryParameters : public IcpParameters {
	int everyKpoints_ = 1;
	double croppingRadius_=20.0;
};

struct MapInconsistencyRemoval {
	Eigen::Vector3d voxelSize_{0.5,0.5,0.5};
	int minPointsPerVoxel_ = 2;
	int numPointsWithHighestErrorToRemove_ = 50;
	double minErrorThresholdForRemoval_ = 1.0;
};

struct SpaceCarvingParameters{
	double voxelSize_=0.1;
	double maxRaytracingLength_ = 20.0;
	double truncationDistance_ = 0.1;
	double carveSpaceEveryNsec_ = 1.0;
};

struct MapperParameters : public IcpParameters {
	double mapVoxelSize_ = 0.03;
	double minMovementBetweenMappingSteps_ = 0.0;
	double minRefinementFitness_ = 0.7;
	double mapBuilderCroppingRadius_=40.0;
	double scanMatcherCroppingRadius_=30.0;

};

struct LocalMapParameters {
	double voxelSize_ = 0.1;
	double croppingRadius_ = 10.0;
};

struct MesherParameters{
	bool isComputeMesh_ = false;
	double voxelSize_ = 0.05;
	MesherStrategy strategy_ = MesherStrategy::AlphaShape;
	double alphaShapeAlpha_ = 0.5;
	int poissonDepth_ = 8;
	double poissonMinDensity_ = 5.0;
	float poissonScale_=1.1;
	std::vector<double> ballPivotRadii_ { 0.3, 1.0 };
	int knnNormalEstimation_ = 4;
};



void loadParameters(const std::string &filename, IcpParameters *p);
void loadParameters(const YAML::Node &node, IcpParameters *p);
void loadParameters(const std::string &filename, MapperParameters *p);
void loadParameters(const YAML::Node &node, MapperParameters *p);
void loadParameters(const std::string &filename, LocalMapParameters *p);
void loadParameters(const YAML::Node &node, LocalMapParameters *p);
void loadParameters(const std::string &filename, OdometryParameters *p);
void loadParameters(const YAML::Node &node, OdometryParameters *p);
void loadParameters(const YAML::Node &n, MesherParameters *p);
void loadParameters(const std::string &filename, MesherParameters *p);
void loadParameters(const YAML::Node &n, SpaceCarvingParameters *p);
void loadParameters(const std::string &filename, SpaceCarvingParameters *p);


} // namespace m545_mapping
