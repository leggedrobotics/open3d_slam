/*
 * Parameters.hpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <map>
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include <typeindex>
#include <boost/concept_check.hpp>


namespace o3d_slam {

namespace params_internal {
	const double kDegToRad =  M_PI / 180.0;
}

struct Parameters {
	Parameters() = default;
	virtual ~Parameters() = default;
	template<class T>
	T* as() {
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Parameters*>));
		return static_cast<T*>(this);
	}
	template<class T>
	const T* as() const {
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Parameters*>));
		return static_cast<const T*>(this);
	}
};

enum class CloudRegistrationType : int {
	PointToPlaneIcp,
	PointToPointIcp,
	GeneralizedIcp
};

static const std::map<std::string, CloudRegistrationType> CloudRegistrationStringToEnumMap {
	{"PointToPlaneIcp",CloudRegistrationType::PointToPlaneIcp},
	{"PointToPointIcp",CloudRegistrationType::PointToPointIcp},
	{"GeneralizedIcp",CloudRegistrationType::GeneralizedIcp}
};

enum class ScanToMapRegistrationType : int {
	PointToPlaneIcp,
	PointToPointIcp,
	GeneralizedIcp
};

static const std::map<std::string, ScanToMapRegistrationType> ScanToMapRegistrationStringToEnumMap {
	{"PointToPlaneIcp",ScanToMapRegistrationType::PointToPlaneIcp},
	{"PointToPointIcp",ScanToMapRegistrationType::PointToPointIcp},
	{"GeneralizedIcp",ScanToMapRegistrationType::GeneralizedIcp}
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
	int maxNumIter_ = 50;
	double maxCorrespondenceDistance_ = 0.2;
	int knn_ = 5;
	double maxDistanceKnn_ = 10.0;
};

struct CloudRegistrationParameters : public Parameters {
	CloudRegistrationType regType_ = CloudRegistrationType::PointToPlaneIcp;
	IcpParameters icp_;
};

struct OdometryParameters {
	CloudRegistrationParameters scanMatcher_;
	ScanProcessingParameters scanProcessing_;
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
	int numScansOverlap_ = 3;
};

struct PlaceRecognitionConsistencyCheckParameters{
	double maxDriftRoll_ = 90.0 * params_internal::kDegToRad;
	double maxDriftPitch_ = 90.0 * params_internal::kDegToRad;
	double maxDriftYaw_ = 90.0 * params_internal::kDegToRad;
	double maxDriftZ_ = 15.0;
	double maxDriftY_ = 10.0;
	double maxDriftX_ = 10.0;
};

struct PlaceRecognitionParameters{
	double normalEstimationRadius_=1.0;
	double featureVoxelSize_ = 0.5;
	double featureRadius_ = 2.5;
	int featureKnn_=100;
	int normalKnn_=10;
	int ransacNumIter_ = 1000000;
	double ransacProbability_ = 0.99;
	int ransacModelSize_=3;
	double ransacMaxCorrespondenceDistance_= 0.75;
	double correspondenceCheckerDistance_=0.75;
	double correspondenceCheckerEdgeLength_=0.5;
	int ransacMinCorrespondenceSetSize_ = 25;
	double maxIcpCorrespondenceDistance_ = 0.3;
	double minRefinementFitness_ = 0.7;
	bool isDumpPlaceRecognitionAlignmentsToFile_ = false;
	PlaceRecognitionConsistencyCheckParameters consistencyCheck_;
	int minSubmapsBetweenLoopClosures_ = 2;
	double loopClosureSearchRadius_ = 20;
};

struct GlobalOptimizationParameters {
	double maxCorrespondenceDistance_ = 10.0;
	double loopClosurePreference_ = 2.0;
	double edgePruneThreshold_ = 0.2;
	int referenceNode_ = 0;
};

struct ScanToMapRegistrationParameters : public Parameters {
	ScanToMapRegistrationType scanToMapRegType_ = ScanToMapRegistrationType::PointToPlaneIcp;
	double minRefinementFitness_ = 0.7;
	IcpParameters icp_;
};

struct MapInitializingParameters { //todo these are a bit implementation specific
  std::string frameId_ = "";
  Eigen::Isometry3d initialPose_ = Eigen::Isometry3d::Identity();
	std::string pcdFilePath_ = "";
	bool isInitializeInteractively_= false;
};

struct MapperParameters {
	ScanToMapRegistrationParameters scanMatcher_;
	ScanProcessingParameters scanProcessing_;
	double minMovementBetweenMappingSteps_ = 0.0;
	bool isIgnoreMinRefinementFitness_ = false;
	MapBuilderParameters mapBuilder_;
	MapBuilderParameters denseMapBuilder_;
	bool isBuildDenseMap_ = true;
	SubmapParameters submaps_;
	PlaceRecognitionParameters placeRecognition_;
	GlobalOptimizationParameters globalOptimization_;
	bool isAttemptLoopClosures_ = true;
	bool isDumpSubmapsToFileBeforeAndAfterLoopClosures_ = false;
	bool isPrintTimingStatistics_ = true;
	bool isRefineOdometryConstraintsBetweenSubmaps_ = false;
	bool isUseInitialMap_ = false;
  bool isMergeScansIntoMap_ = true;
  MapInitializingParameters mapInit_;
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
	bool isSaveDenseSubmaps_ = false;
};

struct ConstantVelocityMotionCompensationParameters {
	bool isUndistortInputCloud_ = false;
	bool isSpinningClockwise_ = true; // otherwise it spins counter clockwise
	double scanDuration_ = 0.1; // sec
	int numPosesVelocityEstimation_ = 3;
};


struct SlamParameters {
	MapperParameters mapper_;
	OdometryParameters odometry_;
	VisualizationParameters visualization_;
	SavingParameters saving_;
	ConstantVelocityMotionCompensationParameters motionCompensation_;
};

} // namespace o3d_slam
