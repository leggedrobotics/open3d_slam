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

struct ScanCroppingParameters {
	double croppingRadius_=20.0;
	double croppingMinZ_ = -10.0;
	double croppingMaxZ_ = 10.0;
	std::string cropperName_ = "MaxRadius";
};

struct ScanProcessingParameters{
	double downSamplingRatio_ = 1.0;
	double voxelSize_ = 0.03;
	ScanCroppingParameters cropper_;
};

struct IcpParameters {
	int kNNnormalEstimation_ = 5;
	int maxNumIter_ = 50;
	double maxCorrespondenceDistance_ = 0.2;
	IcpObjective icpObjective_ = IcpObjective::PointToPoint;
};

struct OdometryParameters {
	IcpParameters scanMatcher_;
	ScanProcessingParameters scanProcessing_;
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
	double minDotProductWithNormal_ = 0.5;
};

struct MapBuilderParameters{
	double mapVoxelSize_ = 0.03;
	ScanCroppingParameters cropper_;
	SpaceCarvingParameters carving_;
};

struct SubmapParameters{
	double radius_=20.0;
	int minNumRangeData_ = 5;
	double minSecondsBetweenFeatureComputation_=5.0;
};

struct PlaceRecognitionParameters{
	double normalEstimationRadius_=1.0;
	double featureVoxelSize_ = 0.5;
	double featureRadius_ = 2.5;
	size_t featureKnn_=100;
	size_t normalKnn_=10;
	size_t ransacNumIter_ = 1000000;
	double ransacProbability_ = 0.99;
	size_t ransacModelSize_=3;
	double ransacMaxCorrespondenceDistance_= 0.75;
	double correspondenceCheckerDistance_=0.75;
	double correspondenceCheckerEdgeLength_=0.5;
	size_t ransacMinCorrespondenceSetSize_ = 25;
	double maxIcpCorrespondenceDistance_ = 0.3;
	double minRefinementFitness_ = 0.7;
};

struct GlobalOptimizationParameters {
	double maxCorrespondenceDistance_ = 10.0;
	double loopClosurePreference_ = 2.0;
	double edgePruneThreshold_ = 0.2;
	int referenceNode_ = 0;
};


struct MapperParameters {
	IcpParameters scanMatcher_;
	ScanProcessingParameters scanProcessing_;
	double minMovementBetweenMappingSteps_ = 0.0;
	double minRefinementFitness_ = 0.7;
	MapBuilderParameters mapBuilder_;
	MapBuilderParameters denseMapBuilder_;
	size_t numScansOverlap_ = 3;
	bool isBuildDenseMap_ = true;
	SubmapParameters submaps_;
	PlaceRecognitionParameters placeRecognition_;
	GlobalOptimizationParameters globalOptimization_;
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

struct VisualizationParameters {
	double assembledMapVoxelSize_ = 0.1;
	double submapVoxelSize_ = 0.1;
	double visualizeEveryNmsec_ = 250.0;
};

void loadParameters(const std::string &filename, PlaceRecognitionParameters *p);
void loadParameters(const YAML::Node &node, PlaceRecognitionParameters *p);
void loadParameters(const std::string &filename, GlobalOptimizationParameters *p);
void loadParameters(const YAML::Node &node, GlobalOptimizationParameters *p);
void loadParameters(const std::string &filename, VisualizationParameters *p);
void loadParameters(const YAML::Node &node, VisualizationParameters *p);
void loadParameters(const std::string &filename, SubmapParameters *p);
void loadParameters(const YAML::Node &node, SubmapParameters *p);
void loadParameters(const std::string &filename, ScanProcessingParameters *p);
void loadParameters(const YAML::Node &node, ScanProcessingParameters *p);
void loadParameters(const std::string &filename, IcpParameters *p);
void loadParameters(const YAML::Node &node, IcpParameters *p);
void loadParameters(const std::string &filename, MapperParameters *p);
void loadParameters(const YAML::Node &node, MapperParameters *p);
void loadParameters(const std::string &filename, MapBuilderParameters *p);
void loadParameters(const YAML::Node &node, MapBuilderParameters *p);
void loadParameters(const std::string &filename, LocalMapParameters *p);
void loadParameters(const YAML::Node &node, LocalMapParameters *p);
void loadParameters(const std::string &filename, OdometryParameters *p);
void loadParameters(const YAML::Node &node, OdometryParameters *p);
void loadParameters(const YAML::Node &n, MesherParameters *p);
void loadParameters(const std::string &filename, MesherParameters *p);
void loadParameters(const YAML::Node &n, SpaceCarvingParameters *p);
void loadParameters(const std::string &filename, SpaceCarvingParameters *p);
void loadParameters(const std::string &filename, ScanCroppingParameters *p);
void loadParameters(const YAML::Node &node, ScanCroppingParameters *p);

} // namespace m545_mapping
