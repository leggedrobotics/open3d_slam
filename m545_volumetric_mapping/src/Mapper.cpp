/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mapper.hpp"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/Voxel.hpp"
#include "open3d/utility/Eigen.h"
#include "open3d/utility/Helper.h"

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
} // namespace

Mapper::Mapper() :
		tfListener_(tfBuffer_) {
	update(params_);

}

void Mapper::setParameters(const MapperParameters &p) {
	params_ = p;
	update(p);
}

void Mapper::update(const MapperParameters &p) {
	icpCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective = icpObjectiveFactory(p.scanMatcher_.icpObjective_);
	scanMatcherCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.scanProcessing_.croppingRadius_);
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilder_.scanCroppingRadius_);
	denseMapCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.denseMapBuilder_.scanCroppingRadius_);
	submaps_.setParameters(p);
}

bool Mapper::isMatchingInProgress() const {
	return isMatchingInProgress_;
}

Eigen::Isometry3d Mapper::getMapToOdom() const {
	return mapToOdom_;
}
Eigen::Isometry3d Mapper::getMapToRangeSensor() const {
	return mapToRangeSensor_;
}

void Mapper::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (params_.scanMatcher_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals(); //todo, dunno if I need this
	}
}

void Mapper::addRangeMeasurement(const Mapper::PointCloud &rawScan, const ros::Time &timestamp) {
	isMatchingInProgress_ = true;
	lastMeasurementTimestamp_ = timestamp;
	scanMatcherCropper_->setPose(Eigen::Isometry3d::Identity());
	submaps_.setMapToRangeSensor(mapToRangeSensor_);

	//insert first scan
	if (submaps_.getActiveSubmap().isEmpty()) {
		auto wideCroppedCloud = preProcessScan(rawScan);
		submaps_.insertScan(rawScan, *wideCroppedCloud, Eigen::Isometry3d::Identity());
		isMatchingInProgress_ = false;
		return;
	}

	Eigen::Isometry3d odomToRangeSensor;
	const bool lookupStatus = lookupTransform(frames::odomFrame, frames::rangeSensorFrame, timestamp, tfBuffer_,
			&odomToRangeSensor);
	const auto odometryMotion = odomToRangeSensorPrev_.inverse() * odomToRangeSensor;
	//todo check rotation and trans
	if (!lookupStatus) {
		isMatchingInProgress_ = false;
		return;
	}

	const auto mapToRangeSensorEstimate = mapToOdom_ * odomToRangeSensor;
	const auto &activeSubmap = submaps_.getActiveSubmap().getMap();
	std::shared_ptr<PointCloud> narrowCropped, wideCroppedCloud, mapPatch;
	{
		static double avgTime = 0;
		static int count = 0;
		Timer timer;
		wideCroppedCloud = preProcessScan(rawScan);
		narrowCropped = scanMatcherCropper_->crop(*wideCroppedCloud);
		scanMatcherCropper_->setPose(mapToRangeSensor_);
		mapPatch = scanMatcherCropper_->crop(activeSubmap);
//		avgTime += timer.elapsedMsec();
//		++count;
//		std::cout << "avg preprocess: " << avgTime / count << "\n";
	}

//	mapRef_=*wideCroppedCloud;
	// wee need to get an active map here

	const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
			params_.scanMatcher_.maxCorrespondenceDistance_, mapToRangeSensorEstimate.matrix(), *icpObjective,
			icpCriteria_);

	if (result.fitness_ < params_.minRefinementFitness_) {
		std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
		isMatchingInProgress_ = false;
		return;
	}

	// update transforms
	mapToRangeSensor_.matrix() = result.transformation_;
	mapToOdom_ = mapToRangeSensor_ * odomToRangeSensor.inverse();
	submaps_.setMapToRangeSensor(mapToRangeSensor_);

	// concatenate registered cloud into map
	const bool isMovedTooLittle = odometryMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
	if (!isMovedTooLittle) {
		submaps_.insertScan(rawScan, *wideCroppedCloud, mapToRangeSensor_);
		odomToRangeSensorPrev_ = odomToRangeSensor;
	}

	isMatchingInProgress_ = false;
}

std::shared_ptr<Mapper::PointCloud> Mapper::preProcessScan(const PointCloud &rawScan) const {
	mapBuilderCropper_->setPose(Eigen::Isometry3d::Identity());
	auto wideCroppedCloud = mapBuilderCropper_->crop(rawScan);
	m545_mapping::voxelize(params_.scanProcessing_.voxelSize_, wideCroppedCloud.get());
	m545_mapping::randomDownSample(params_.scanProcessing_.downSamplingRatio_, wideCroppedCloud.get());
	estimateNormalsIfNeeded(wideCroppedCloud.get());
	return wideCroppedCloud;
}

void Mapper::carve(const PointCloud &scan, const CroppingVolume &cropper, const SpaceCarvingParameters &params,
		PointCloud *map, Timer *timer) const {
	if (map->points_.empty() || timer->elapsedSec() < params.carveSpaceEveryNsec_) {
		return;
	}
//	Timer timer("carving");
	const auto wideCroppedIdxs = cropper.getIndicesWithinVolume(*map);
	auto idxsToRemove = std::move(
			getIdxsOfCarvedPoints(scan, *map, mapToRangeSensor_.translation(), wideCroppedIdxs, params));

//	std::cout << "Would remove: " << idxsToRemove.size() << std::endl;
	removeByIds(idxsToRemove, map);
	timer->reset();
}

const Mapper::PointCloud& Mapper::getMap() const {
	return submaps_.getActiveSubmap().getMap();
}

const Submap& Mapper::getActiveSubmap() const {
  return submaps_.getActiveSubmap();
}

const Mapper::PointCloud& Mapper::getDenseMap() const {
	return submaps_.getActiveSubmap().getDenseMap();
}

bool Mapper::isManipulatingMap() const {
	return isManipulatingMap_;
}

Mapper::PointCloud* Mapper::getMapPtr() {
	return &map_;
}
Mapper::PointCloud* Mapper::getDenseMapPtr() {
	return &denseMap_;
}

} /* namespace m545_mapping */
