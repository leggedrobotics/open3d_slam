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

void Mapper::setParameters(const MapperParameters &p, const SpaceCarvingParameters &carvingParams) {
	params_ = p;
	update(p);
	carvingParameters_=carvingParams;
}

void Mapper::update(const MapperParameters &p) {
	icpCriteria_.max_iteration_ = params_.maxNumIter_;
	icpObjective = icpObjectiveFactory(params_.icpObjective_);
	scanMatcherCropper_ = std::make_shared<BallCropper>(p.scanMatcherCroppingRadius_);
	mapBuilderCropper_ = std::make_shared<BallCropper>(p.mapBuilderCroppingRadius_);
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
	//insert first scan
	if (map_.points_.empty()) {
//		auto cloud = cloudIn;
//		auto bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_, params_.mapBuilderCropBoxHighBound_);
//		auto croppedCloud = cloud.Crop(bbox);
		mapBuilderCropper_->setPose(Eigen::Isometry3d::Identity());
		auto croppedCloud = mapBuilderCropper_->crop(cloudIn);
		m545_mapping::voxelize(params_.voxelSize_, croppedCloud.get());
//		auto voxelizedCloud = croppedCloud->VoxelDownSample(params_.mapVoxelSize_);
		estimateNormalsIfNeeded(croppedCloud.get());
		map_ += *croppedCloud;
		denseMap_ += *(mapBuilderCropper_->crop(cloudIn));
		isMatchingInProgress_ = false;
		return;
	}

	Eigen::Isometry3d odomToRangeSensor;
	const bool lookupStatus = lookupTransform(frames::odomFrame, frames::rangeSensorFrame, timestamp,tfBuffer_,
			&odomToRangeSensor);
	const auto odometryMotion = odomToRangeSensorPrev_.inverse() * odomToRangeSensor;
	//todo check rotation and trans
	if (!lookupStatus) {
		isMatchingInProgress_ = false;
		return;
	}
//	auto cloud = cloudIn;
	Timer timer;
//	open3d::geometry::AxisAlignedBoundingBox bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_,
//			params_.mapBuilderCropBoxHighBound_);
//	auto wideCroppedCloud = cloud.Crop(bbox);
	mapBuilderCropper_->setPose(Eigen::Isometry3d::Identity());
	scanMatcherCropper_->setPose(Eigen::Isometry3d::Identity());
	auto wideCroppedCloud = mapBuilderCropper_->crop(cloudIn);


//	std::cout << "Cropping input cloud finished\n";
//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";
//	auto voxelizedCloud = croppedCloud->VoxelDownSample(params_.voxelSize_);
	{
//		Timer timer("voxelize_input_cloud");
		m545_mapping::voxelize(params_.voxelSize_, wideCroppedCloud.get());
	}

//	bbox = boundingBoxAroundPosition(params_.cropBoxLowBound_, params_.cropBoxHighBound_);
//	auto narrowCropped = wideCroppedCloud->Crop(bbox);

	auto narrowCropped = scanMatcherCropper_->crop(*wideCroppedCloud);
	m545_mapping::randomDownSample(params_.downSamplingRatio_, narrowCropped.get());

//	bbox = boundingBoxAroundPosition(params_.cropBoxLowBound_, params_.cropBoxHighBound_,
//			mapToRangeSensor_.translation());
//	auto mapPatch = map_.Crop(bbox);
	scanMatcherCropper_->setPose(mapToRangeSensor_);
	auto mapPatch = scanMatcherCropper_->crop(map_);

//	std::cout << "Map and scan pre processing finished\n";
//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";

	Timer timer2;
//	const Eigen::Matrix4d initTransform = (mapToRangeSensor_).matrix();
	const Eigen::Matrix4d initTransform = (mapToOdom_ * odomToRangeSensor).matrix();
	const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
			params_.maxCorrespondenceDistance_, initTransform, *icpObjective, icpCriteria_);

//	std::cout << "Scan to map matching finished \n";
//	std::cout << "Time elapsed: " << timer2.elapsedMsec() << " msec \n";
//	std::cout << "fitness: " << result.fitness_ << std::endl;
//	std::cout << "RMSE: " << result.inlier_rmse_ << std::endl;

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
		std::lock_guard<std::mutex> lck(mapManipulationMutex_);
		isManipulatingMap_ = true;
//		Timer timer("Map update");
		m545_mapping::randomDownSample(params_.downSamplingRatio_, wideCroppedCloud.get());
		wideCroppedCloud->Transform(result.transformation_);
		estimateNormalsIfNeeded(wideCroppedCloud.get());
		auto transformedCloud = std::move(*(transform(result.transformation_,cloudIn)));
		carve(transformedCloud, &map_);
		map_ += *wideCroppedCloud;

		if (params_.mapVoxelSize_ > 0.0) {
//			Timer timer("voxelize_map",true);
//			auto bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_, params_.mapBuilderCropBoxHighBound_,
//					mapToRangeSensor_.translation());
//			auto voxelizedMap = voxelizeAroundPosition(params_.mapVoxelSize_, bbox, map_);
			mapBuilderCropper_->setPose(mapToRangeSensor_);
			auto voxelizedMap = voxelizeWithinCroppingVolume(params_.mapVoxelSize_, *mapBuilderCropper_, map_);
			map_ = *voxelizedMap;
		}

		{
//			Timer timer("merge_dense_map");
//			denseMap_ += transformedCloud;
//			shaveOffArtifacts(cloud, &denseMap_);
		}

//		std::cout << "\n";
		isManipulatingMap_ = false;
		odomToRangeSensorPrev_ = odomToRangeSensor;
	}

	isMatchingInProgress_ = false;
}

void Mapper::carve(const PointCloud &scan, PointCloud *map) {

	if(map->points_.empty() || carvingTimer_.elapsedSec()<carvingParameters_.carveSpaceEveryNsec_){
		return;
	}
//	Timer timer("carving");

	mapBuilderCropper_->setPose(mapToRangeSensor_);
	 const auto wideCroppedIdxs = mapBuilderCropper_->getIndicesWithinVolume(map_);
	auto idxsToRemove = std::move(getIdxsOfCarvedPoints(scan,*map,mapToRangeSensor_.translation(),wideCroppedIdxs,carvingParameters_));
//
//	auto idxsToRemove = std::move(getIdxsOfCarvedPoints(scan,*map,mapToRangeSensor_.translation(),carvingParameters_));
	toRemove_ = std::move(*(map->SelectByIndex(idxsToRemove)));
	scanRef_ = scan;
//	std::cout << "Would remove: " << idxsToRemove.size() << std::endl;
	removeByIds(idxsToRemove, map);
	carvingTimer_.reset();

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
