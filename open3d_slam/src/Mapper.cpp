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
#include "open3d_slam/ScanToMapRegistration.hpp"

#include "open3d/utility/Eigen.h"
#include "open3d/utility/Helper.h"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
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
	scan2MapReg_ = scanToMapRegistrationFactory(p);
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
void Mapper::setMapToRangeSensorInitial(const Transform &t){
	mapToRangeSensorPrev_ = t;
	mapToRangeSensor_ = t;
	isNewInitialValueSet_ = true;
}

const PointCloud& Mapper::getPreprocessedScan() const {
	return preProcessedScan_;
}

const ScanToMapRegistration &Mapper::getScanToMapRegistration() const{
	return *scan2MapReg_;
}

bool Mapper::addRangeMeasurement(const Mapper::PointCloud &rawScan, const Time &timestamp) {
	submaps_->setMapToRangeSensor(mapToRangeSensor_);

	//insert first scan
	if (submaps_->getActiveSubmap().isEmpty()) {
		if (params_.isUseInitialMap_){
			assert_true(scan2MapReg_->isMergeScanValid(rawScan),"Init map invalid!!!!");
			submaps_->insertScan(rawScan, rawScan, Transform::Identity(), timestamp);
		} else {
			const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
			submaps_->insertScan(rawScan, *processed.merge_, Transform::Identity(), timestamp);
			mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
		}
		return true;
	}

	if (timestamp < lastMeasurementTimestamp_) {
		std::cerr << "\n\n !!!!! MAPER WARNING: Measurements came out of order!!!! \n\n";
		return false;
	}

	bool isOdomOkay = odomToRangeSensorBuffer_.has(timestamp);
	if (!isOdomOkay) {
		std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
		std::cerr << "  going to attempt the scan to map refinement anyway \n";
	}

	checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);

	Transform mapToRangeSensorEstimate =  mapToRangeSensorPrev_;

	if (isOdomOkay && !isNewInitialValueSet_ && !isIgnoreOdometryPrediction_){
		const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
		const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_);
		const Transform odometryMotion = odomToRangeSensorPrev.inverse()*odomToRangeSensor;
		mapToRangeSensorEstimate = mapToRangeSensorPrev_*odometryMotion ;
	}
	isIgnoreOdometryPrediction_ = false;
	const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
	const RegistrationResult result = scan2MapReg_->scanToMapRegistration(*processed.match_, submaps_->getActiveSubmap(),
			mapToRangeSensor_, mapToRangeSensorEstimate);
	preProcessedScan_ = *processed.match_;
	if (isNewInitialValueSet_){
		mapToRangeSensorPrev_ = mapToRangeSensor_;
		mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
		isNewInitialValueSet_ = false;
		isIgnoreOdometryPrediction_ = true;
		return true;
	}

	if (!params_.isIgnoreMinRefinementFitness_ && result.fitness_ < params_.scanMatcher_.minRefinementFitness_) {
			std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
			std::cout << "preeIcp: " << asString(mapToRangeSensorEstimate) << "\n";
			std::cout << "postIcp: " << asString(Transform(result.transformation_)) << "\n\n";
			return false;
	}

	// update transforms
	mapToRangeSensor_.matrix() = result.transformation_;
	mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
	submaps_->setMapToRangeSensor(mapToRangeSensor_);

	if (params_.isUseInitialMap_ && !params_.isMergeScansIntoMap_){
		lastMeasurementTimestamp_ = timestamp;
		mapToRangeSensorPrev_ = mapToRangeSensor_;
		return true;
	}

	// concatenate registered cloud into map
	const Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
	const bool isMovedTooLittle = sensorMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
	if (!isMovedTooLittle) {
		//Timer t("scan_insertion_and_bookeeping");
		submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
		mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
	}

	lastMeasurementTimestamp_ = timestamp;
	mapToRangeSensorPrev_ = mapToRangeSensor_;
	return true;
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
