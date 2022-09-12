/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/output.hpp"

#include "open3d/utility/Eigen.h"
#include "open3d/utility/Helper.h"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
const bool isCheckTransformChainingAndPrintResult = false;
} // namespace

Mapper::Mapper(const TransformInterpolationBuffer &odomToRangeSensorBuffer,
		std::shared_ptr<SubmapCollection> submaps) :
		odomToRangeSensorBuffer_(odomToRangeSensorBuffer), submaps_(submaps) {
	update(params_);
}

void Mapper::setParameters(const MapperParameters &p) {
	params_ = p;
	update(p);
}

MapperParameters *Mapper::getParametersPtr(){
	return &params_;
}

void Mapper::loopClosureUpdate(const Transform &loopClosureCorrection) {
	mapToRangeSensor_ = loopClosureCorrection * mapToRangeSensor_;
	mapToRangeSensorPrev_ = loopClosureCorrection * mapToRangeSensorPrev_;
}

bool Mapper::hasProcessedMeasurements() const {
	return !mapToRangeSensorBuffer_.empty();
}

void Mapper::update(const MapperParameters &p) {
	icpCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective = icpObjectiveFactory(p.scanMatcher_.icpObjective_);
	mapBuilderCropper_ = croppingVolumeFactory(params_.mapBuilder_.cropper_);
	scanMatcherCropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
	submaps_->setParameters(p);
}

Transform Mapper::getMapToOdom(const Time &timestamp) const {
	const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
	const Transform mapToRangeSensor = getTransform(timestamp, mapToRangeSensorBuffer_);
	return mapToRangeSensor * odomToRangeSensor.inverse();
//	return getTransform(timestamp, mapToOdomBuffer_);
}
Transform Mapper::getMapToRangeSensor(const Time &timestamp) const {
	return getTransform(timestamp, mapToRangeSensorBuffer_);
}

const SubmapCollection& Mapper::getSubmaps() const {
	return *submaps_;
}

const Submap& Mapper::getActiveSubmap() const {
	return submaps_->getActiveSubmap();
}

const TransformInterpolationBuffer& Mapper::getMapToRangeSensorBuffer() const {
	return mapToRangeSensorBuffer_;
}

SubmapCollection* Mapper::getSubmapsPtr() {
	return submaps_.get();
}

void Mapper::setMapToRangeSensor(const Transform &t) {
	mapToRangeSensor_ = t;
}
void Mapper::setMapToRangeSensorPrev(const Transform &t){
	mapToRangeSensorPrev_ = t;
}

void Mapper::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (!pcl->HasNormals() && params_.scanMatcher_.icpObjective_ == o3d_slam::IcpObjective::PointToPlane) {
		estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals(); //todo, dunno if I need this
	}
}

const PointCloud& Mapper::getPreprocessedScan() const {
	return preProcessedScan_;
}

bool Mapper::addRangeMeasurement(const Mapper::PointCloud &rawScan, const Time &timestamp) {
	isMatchingInProgress_ = true;
	scanMatcherCropper_->setPose(Transform::Identity());
	submaps_->setMapToRangeSensor(mapToRangeSensor_);

	//insert first scan
	if (submaps_->getActiveSubmap().isEmpty()) {
		if (params_.isUseInitialMap_){
			submaps_->insertScan(rawScan, rawScan, Transform::Identity(), timestamp);
		} else {
			auto wideCroppedCloud = preProcessScan(rawScan);
			submaps_->insertScan(rawScan, *wideCroppedCloud, Transform::Identity(), timestamp);
			mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
		}
		isMatchingInProgress_ = false;
		return true;
	}

	if (timestamp < lastMeasurementTimestamp_) {
		std::cerr << "\n\n !!!!! MAPER WARNING: Measurements came out of order!!!! \n\n";
		isMatchingInProgress_ = false;
		return false;
	}

	bool isOdomOkay = odomToRangeSensorBuffer_.has(timestamp);
	if (!isOdomOkay) {
		std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
		std::cerr << "  going to attempt the scan to map refinement anyway \n";
	}

	checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);

	Transform mapToRangeSensorEstimate =  mapToRangeSensorPrev_;

	if (isOdomOkay){
		const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
		const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_);
		const Transform odometryMotion = odomToRangeSensorPrev.inverse()*odomToRangeSensor;
		mapToRangeSensorEstimate =  mapToRangeSensorPrev_*odometryMotion ;
	}

	const PointCloud &activeSubmapPointCloud = submaps_->getActiveSubmap().getMapPointCloud();
	std::shared_ptr<PointCloud> narrowCropped, wideCroppedCloud, mapPatch;
	{
		Timer timer;
		wideCroppedCloud = preProcessScan(rawScan);
		narrowCropped = scanMatcherCropper_->crop(*wideCroppedCloud);
		preProcessedScan_ = *narrowCropped;
		scanMatcherCropper_->setPose(mapToRangeSensor_);
		mapPatch = scanMatcherCropper_->crop(activeSubmapPointCloud);
		assert_gt<int>(mapPatch->points_.size(),0,"map patch size is zero");
		assert_gt<int>(narrowCropped->points_.size(),0,"narrow cropped size is zero");
	}

	const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
			params_.scanMatcher_.maxCorrespondenceDistance_, mapToRangeSensorEstimate.matrix(), *icpObjective,
			icpCriteria_);

	//todo see how to handle this
//	if (params_.isUseInitialMap_) {
//		if (rejectDistantTransform(mapToRangeSensorEstimate, Transform(result.transformation_), params_.mapInitializationRejection_)) {
//			return false;
//		}
//		params_.isUseInitialMap_ = false;
//	}

	if (result.fitness_ < params_.minRefinementFitness_) {
			std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
			std::cout << "preeIcp: " << asString(mapToRangeSensorEstimate) << "\n";
			std::cout << "postIcp: " << asString(Transform(result.transformation_)) << "\n\n";
			isMatchingInProgress_ = false;
			return false;
	}

	// update transforms
	mapToRangeSensor_.matrix() = result.transformation_;
	mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
	submaps_->setMapToRangeSensor(mapToRangeSensor_);

	if (!params_.isMergeScansIntoMap_){
		lastMeasurementTimestamp_ = timestamp;
		mapToRangeSensorPrev_ = mapToRangeSensor_;
		isMatchingInProgress_ = false;
		return true;
	}

	// concatenate registered cloud into map
	const Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
	const bool isMovedTooLittle = sensorMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
	if (!isMovedTooLittle) {
		//Timer t("scan_insertion_and_bookeeping");
		submaps_->insertScan(rawScan, *wideCroppedCloud, mapToRangeSensor_, timestamp);
		mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
	}

	lastMeasurementTimestamp_ = timestamp;
	mapToRangeSensorPrev_ = mapToRangeSensor_;
	isMatchingInProgress_ = false;
	return true;
}

std::shared_ptr<Mapper::PointCloud> Mapper::preProcessScan(const PointCloud &rawScan) const {
	mapBuilderCropper_->setPose(Transform::Identity());
	std::shared_ptr<PointCloud> wideCroppedCloud, voxelized, downsampled;
	wideCroppedCloud = mapBuilderCropper_->crop(rawScan);
	if (params_.scanProcessing_.voxelSize_ <= 0.0) {
		voxelized = wideCroppedCloud;
	} else {
		voxelized = wideCroppedCloud->VoxelDownSample(params_.scanProcessing_.voxelSize_);
	}

	if (params_.scanProcessing_.downSamplingRatio_ >= 1.0) {
		downsampled = voxelized;
	} else {
		downsampled = voxelized->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);
	}
	estimateNormalsIfNeeded(downsampled.get());
	return downsampled;
}

Mapper::PointCloud Mapper::getAssembledMapPointCloud() const {
	PointCloud cloud;
	const int nPoints = submaps_->getTotalNumPoints();
	const Submap &activeSubmap = getActiveSubmap();
	cloud.points_.reserve(nPoints);
	if (activeSubmap.getMapPointCloud().HasColors()) {
		cloud.colors_.reserve(nPoints);
	}
	if (activeSubmap.getMapPointCloud().HasNormals()) {
		cloud.normals_.reserve(nPoints);
	}

	for (size_t j = 0; j < submaps_->getNumSubmaps(); ++j) {
		const PointCloud submap = submaps_->getSubmap(j).getMapPointCloudCopy();
		for (size_t i = 0; i < submap.points_.size(); ++i) {
			cloud.points_.push_back(submap.points_.at(i));
			if (submap.HasColors()) {
				cloud.colors_.push_back(submap.colors_.at(i));
			}
			if (submap.HasNormals()) {
				cloud.normals_.push_back(submap.normals_.at(i));
			}
		}
	}
	return cloud;
}

void Mapper::checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const {
	if (isCheckTransformChainingAndPrintResult && odomToRangeSensorBuffer_.size() > 70 && mapToRangeSensorBuffer_.size() > 70) {
		const auto odom1 = odomToRangeSensorBuffer_.latest_measurement(60).transform_;
		const auto odom2 = odomToRangeSensorBuffer_.latest_measurement(20).transform_;
		const auto start = mapToRangeSensorBuffer_.latest_measurement(60).transform_;
		const auto gt = mapToRangeSensorBuffer_.latest_measurement(20).transform_;
		const Transform mapMotion = start.inverse() * gt;
		const Transform odomMotion = odom1.inverse() * odom2;
		std::cout << "start      :  " << asString(start) << "\n";
		std::cout << "gt         :  " << asString(gt) << "\n";
		std::cout << "gt computed:  " << asString(start*mapMotion) << "\n";
		std::cout << "est        : " << asString(start*odomMotion) << "\n\n";
	}
}


} /* namespace o3d_slam */
