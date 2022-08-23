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


namespace o3d_slam {

namespace params_internal {
	const double kDegToRad =  M_PI / 180.0;
}

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
	double croppingMinZ_ = -10.0;
	double croppingMaxZ_ = 10.0;
	double croppingMinRadius_ = 0.0;
	double croppingMaxRadius_ = 20.0;
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

struct  DistantTransformRejectingParameters{
	double maxTranslationError_ = 0.3;
	double maxAngleError_ = M_PI / 18.0;
};

struct MapInconsistencyRemoval {
	Eigen::Vector3d voxelSize_{0.5,0.5,0.5};
	int minPointsPerVoxel_ = 2;
	int numPointsWithHighestErrorToRemove_ = 50;
	double minErrorThresholdForRemoval_ = 1.0;
};

struct OdometryToolsParameters {
	IcpParameters scanMatcher_;
	ScanProcessingParameters scanProcessing_;
	double minAcceptableFitness_ = 0.1;
};

struct OdometryParameters {
	OdometryToolsParameters scanToScanToolsParams_;
	OdometryToolsParameters mapInitializingToolsParams_;
	bool isMapInitializing_ = false;
	bool isPublishOdometryMsgs_ = false;
};

struct SpaceCarvingParameters{
	double voxelSize_=0.1;
	double maxRaytracingLength_ = 20.0;
	double truncationDistance_ = 0.1;
	int carveSpaceEveryNscans_ = 10;
	double minDotProductWithNormal_ = 0.5;
	double neighborhoodRadiusDenseMap_ = 0.1;
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
	double adjacencyBasedRevisitingMinFitness_ = 0.4;
};

struct PlaceRecognitionConsistencyCheckParameters{
	double maxDriftRoll_ = 90.0 * params_internal::kDegToRad;
	double maxDriftPitch_ = 90.0 * params_internal::kDegToRad;
	double maxDriftYaw_ = 90.0 * params_internal::kDegToRad;
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
	bool isDumpPlaceRecognitionAlignmentsToFile_ = false;
	PlaceRecognitionConsistencyCheckParameters consistencyCheck_;
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
	DistantTransformRejectingParameters mapInitializationRejection_;
	bool isAttemptLoopClosures_ = true;
	bool isDumpSubmapsToFileBeforeAndAfterLoopClosures_ = false;
	bool isPrintTimingStatistics_ = true;
	bool isRefineOdometryConstraintsBetweenSubmaps_ = false;
	bool isMapInitializing_ = false;
};

struct VisualizationParameters {
	double assembledMapVoxelSize_ = 0.1;
	double submapVoxelSize_ = 0.1;
	double visualizeEveryNmsec_ = 250.0;
};

struct SavingParameters {
	bool isSaveAtMissionEnd_ = false;
	bool isSaveMap_ = false;
	bool isSaveSubmaps_ = false;
};

struct ConstantVelocityMotionCompensationParameters {
	bool isUndistortInputCloud_ = false;
	bool isSpinningClockwise_ = true; // otherwise it spins counter clockwise
	double scanDuration_ = 0.1; // sec
	int numPosesVelocityEstimation_ = 3;
};

void loadParameters(const std::string &filename, ConstantVelocityMotionCompensationParameters *p);
void loadParameters(const YAML::Node &node, ConstantVelocityMotionCompensationParameters *p);
void loadParameters(const std::string &filename, SavingParameters *p);
void loadParameters(const YAML::Node &node, SavingParameters *p);
void loadParameters(const std::string &filename, PlaceRecognitionConsistencyCheckParameters *p);
void loadParameters(const YAML::Node &node, PlaceRecognitionConsistencyCheckParameters *p);
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
void loadParameters(const std::string &filename, OdometryParameters *p);
void loadParameters(const YAML::Node &node, OdometryParameters *p);
void loadParameters(const YAML::Node &node, OdometryToolsParameters *p);
void loadParameters(const YAML::Node &node, DistantTransformRejectingParameters *p);
void loadParameters(const YAML::Node &node, SpaceCarvingParameters *p);
void loadParameters(const std::string &filename, SpaceCarvingParameters *p);
void loadParameters(const YAML::Node &node, ScanCroppingParameters *p);
void loadParameters(const std::string &filename, ScanCroppingParameters *p);

} // namespace o3d_slam
