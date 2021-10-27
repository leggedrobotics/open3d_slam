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

void Mapper::setParameters(const MapperParameters &p, const SpaceCarvingParameters &carvingParams, const m545_mapping::LocalMapParameters &localMapParams) {
	params_ = p;
	carvingParameters_ = carvingParams;
	localMapParams_ = localMapParams;
	update(p);
}

void Mapper::update(const MapperParameters &p) {
	icpCriteria_.max_iteration_ = params_.maxNumIter_;
	icpObjective = icpObjectiveFactory(params_.icpObjective_);
	scanMatcherCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.scanMatcherCroppingRadius_);
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilderCroppingRadius_);
	denseMapCropper_ = std::make_shared<MaxRadiusCroppingVolume>(localMapParams_.croppingRadius_);
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
	if (params_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		estimateNormals(params_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals();
	}
}

void Mapper::addRangeMeasurement(const Mapper::PointCloud &cloudIn, const ros::Time &timestamp) {
	isMatchingInProgress_ = true;
	lastMeasurementTimestamp_ = timestamp;
	mapBuilderCropper_->setPose(Eigen::Isometry3d::Identity());
	scanMatcherCropper_->setPose(Eigen::Isometry3d::Identity());
	//insert first scan
	if (map_.points_.empty()) {
		insertFirstScan(cloudIn);
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
//	auto cloud = cloudIn;
	Timer timer;
	auto wideCroppedCloud = mapBuilderCropper_->crop(cloudIn);
	{
//		Timer timer("voxelize_input_cloud");
		m545_mapping::voxelize(params_.voxelSize_, wideCroppedCloud.get());
	}

	auto narrowCropped = scanMatcherCropper_->crop(*wideCroppedCloud);
	m545_mapping::randomDownSample(params_.downSamplingRatio_, narrowCropped.get());
	scanMatcherCropper_->setPose(mapToRangeSensor_);
	auto mapPatch = scanMatcherCropper_->crop(map_);
	const Eigen::Matrix4d initTransform = (mapToOdom_ * odomToRangeSensor).matrix();
	const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
			params_.maxCorrespondenceDistance_, initTransform, *icpObjective, icpCriteria_);

	if (result.fitness_ < params_.minRefinementFitness_) {
		std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
		isMatchingInProgress_ = false;
		return;
	}

	// update transforms
	mapToRangeSensor_.matrix() = result.transformation_;
	mapToOdom_ = mapToRangeSensor_ * odomToRangeSensor.inverse();

	// concatenate registered cloud into map
	const bool isMovedTooLittle = odometryMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
	if (!isMovedTooLittle) {
		insertScanInMap(wideCroppedCloud, result, cloudIn);
		odomToRangeSensorPrev_ = odomToRangeSensor;
	}

	isMatchingInProgress_ = false;
}

void Mapper::insertFirstScan(const PointCloud &scan) {
	mapBuilderCropper_->setPose(Eigen::Isometry3d::Identity());
	scanMatcherCropper_->setPose(Eigen::Isometry3d::Identity());

	std::lock_guard<std::mutex> lck(mapManipulationMutex_);
	isManipulatingMap_ = true;
	auto croppedCloud = mapBuilderCropper_->crop(scan);
	m545_mapping::voxelize(params_.voxelSize_, croppedCloud.get());
	estimateNormalsIfNeeded(croppedCloud.get());
	map_ += *croppedCloud;
	denseMap_ += *(denseMapCropper_->crop(scan));
	isManipulatingMap_ = false;
}

void Mapper::insertScanInMap(const std::shared_ptr<PointCloud> &wideCroppedCloud,
		const open3d::pipelines::registration::RegistrationResult &result, const PointCloud &rawScan) {
	std::lock_guard<std::mutex> lck(mapManipulationMutex_);
	isManipulatingMap_ = true;


	//		Timer timer("Map update");
	m545_mapping::randomDownSample(params_.downSamplingRatio_, wideCroppedCloud.get());
	wideCroppedCloud->Transform(result.transformation_);
	estimateNormalsIfNeeded(wideCroppedCloud.get());
	auto transformedCloud = std::move(*(transform(result.transformation_, rawScan)));
	mapBuilderCropper_->setPose(mapToRangeSensor_);
	carve(transformedCloud, *mapBuilderCropper_,carvingParameters_,&map_,&carvingTimer_);
	map_ += *wideCroppedCloud;

	if (params_.mapVoxelSize_ > 0.0) {
		//			Timer timer("voxelize_map",true);
		auto voxelizedMap = voxelizeWithinCroppingVolume(params_.mapVoxelSize_, *mapBuilderCropper_, map_);
		map_ = *voxelizedMap;
	}

	{
					Timer timer("merge_dense_map");
		denseMapCropper_->setPose(mapToRangeSensor_);
		auto denseCropped = denseMapCropper_->crop(transformedCloud);
		carve(transformedCloud, *denseMapCropper_, carvingParameters_, &denseMap_,&carveDenseMapTimer_);
		denseMap_ += *denseCropped;
		auto voxelizedDense = voxelizeWithinCroppingVolume(localMapParams_.voxelSize_, *denseMapCropper_,  denseMap_);
		denseMap_= *voxelizedDense;
	}
	isManipulatingMap_ = false;

}

void Mapper::carve(const PointCloud &scan,const CroppingVolume &cropper,const SpaceCarvingParameters &params, PointCloud *map,Timer *timer) const {

	if (map->points_.empty() || timer->elapsedSec() < params.carveSpaceEveryNsec_) {
		return;
	}
//	Timer timer("carving");
	const auto wideCroppedIdxs = cropper.getIndicesWithinVolume(*map);
	auto idxsToRemove = std::move(
			getIdxsOfCarvedPoints(scan, *map, mapToRangeSensor_.translation(), wideCroppedIdxs, params));
	toRemove_ = std::move(*(map->SelectByIndex(idxsToRemove)));
	scanRef_ = scan;
//	std::cout << "Would remove: " << idxsToRemove.size() << std::endl;
	removeByIds(idxsToRemove, map);
	timer->reset();

}

const Mapper::PointCloud& Mapper::getMap() const {
	return map_;
}

const Mapper::PointCloud& Mapper::getDenseMap() const {
	return denseMap_;
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
