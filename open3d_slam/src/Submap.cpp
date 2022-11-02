/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Submap.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam/typedefs.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <utility>
#include <thread>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const std::string voxelMapLayer = "layer";
} // namespace

Submap::Submap(size_t id, size_t parentId) :
		id_(id), parentId_(parentId) {
	update(params_);
}

size_t Submap::getId() const {
	return id_;
}

size_t Submap::getParentId() const {
	return parentId_;
}

bool Submap::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &time, bool isPerformCarving) {

	if (preProcessedScan.IsEmpty()){
		return true;
	}

	mapToRangeSensor_ = mapToRangeSensor;

	if (params_.isUseInitialMap_ && mapCloud_.IsEmpty()){
		std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
		mapCloud_ = preProcessedScan;
		voxelize(params_.mapBuilder_.mapVoxelSize_, &mapCloud_);
		return true;
	}

	auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), preProcessedScan);
	if (isPerformCarving) {
		carvingStatisticsTimer_.startStopwatch();
		{
			std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
			carve(rawScan, mapToRangeSensor, *mapBuilderCropper_, params_.mapBuilder_.carving_, &mapCloud_);
		}
		const double timeMeasurement = carvingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		carvingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (nScansInsertedMap_ % 100 == 1) {
			std::cout << "Space carving timing stats: Avg execution time: "
					<< carvingStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / carvingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			carvingStatisticsTimer_.reset();
		}
	}
	std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
	mapCloud_ += *transformedCloud;
	mapBuilderCropper_->setPose(mapToRangeSensor);
	voxelizeInsideCroppingVolume(*mapBuilderCropper_, params_.mapBuilder_, &mapCloud_);
	++nScansInsertedMap_;
	return true;
}

bool Submap::insertScanDenseMap(const PointCloud &rawScan, const Transform &mapToRangeSensor,
		const Time &time, bool isPerformCarving) {

	denseMapCropper_->setPose(Transform::Identity());
	auto cropped = denseMapCropper_->crop(rawScan);
	auto validColors = colorCropper_.crop(*cropped);
	auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), *validColors);
	{
		std::lock_guard<std::mutex> lck(denseMapMutex_);
		denseMap_.insert(*transformedCloud);
	}
	if (isPerformCarving) {
		std::lock_guard<std::mutex> lck(denseMapMutex_);
		carve(rawScan, mapToRangeSensor.translation(), params_.denseMapBuilder_.carving_, &denseMap_);
	}
	++nScansInsertedDenseMap_;
	return true;
}

void Submap::transform(const Transform &T) {
	const Eigen::Matrix4d mat(T.matrix());
	sparseMapCloud_.Transform(mat);
	{
		std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
		mapCloud_.Transform(mat);

	}
	{
		std::lock_guard<std::mutex> lck(denseMapMutex_);
		denseMap_.transform(T);
	}
	mapToRangeSensor_ = mapToRangeSensor_ * T;
	submapCenter_ = T * submapCenter_;
}

void Submap::carve(const PointCloud &rawScan, const Transform &mapToRangeSensor,
		const CroppingVolume &cropper, const SpaceCarvingParameters &params, PointCloud *map) {
	if (map->points_.empty() || !(nScansInsertedMap_ % params.carveSpaceEveryNscans_ == 1)) {
		return;
	}
//	Timer timer("carving");
	auto scan = o3d_slam::transform(mapToRangeSensor.matrix(), rawScan);
//	auto croppedScan = removeDuplicatePointsWithinSameVoxels(*scan, Eigen::Vector3d::Constant(params_.mapBuilder_.mapVoxelSize_));
	const auto wideCroppedIdxs = cropper.getIndicesWithinVolume(*map);
	auto idxsToRemove = std::move(
			getIdxsOfCarvedPoints(*scan, *map, mapToRangeSensor.translation(), wideCroppedIdxs, params));
	toRemove_ = std::move(*(map->SelectByIndex(idxsToRemove)));
	scanRef_ = std::move(*scan);
//	std::cout << "Would remove: " << idxsToRemove.size() << std::endl;
	removeByIds(idxsToRemove, map);
}

void Submap::carve(const PointCloud &scan, const Eigen::Vector3d &sensorPosition, const SpaceCarvingParameters &param, VoxelizedPointCloud *cloud){
	if (cloud->empty() || !(nScansInsertedDenseMap_ % param.carveSpaceEveryNscans_ == 1)) {
			return;
		}
	const PointCloudPtr croppedScanPtr = removeDuplicatePointsWithinSameVoxels(scan, Eigen::Vector3d::Constant(params_.denseMapBuilder_.mapVoxelSize_));
	std::vector<Eigen::Vector3i> keysToRemove = getKeysOfCarvedPoints(*croppedScanPtr, *cloud, sensorPosition, param);
	for (const auto &k : keysToRemove){
		cloud->removeKey(k);
	}
}

void Submap::voxelizeInsideCroppingVolume(const CroppingVolume &cropper, const MapBuilderParameters &param,
		PointCloud *map) const {
	if (param.mapVoxelSize_ > 0.0) {
		//			Timer timer("voxelize_map",true);
		auto voxelizedMap = voxelizeWithinCroppingVolume(param.mapVoxelSize_, cropper, *map);
		*map = *voxelizedMap;
	}
}

void Submap::setParameters(const MapperParameters &mapperParams) {
	params_ = mapperParams;
	update(mapperParams);
}

Submap::Submap(const Submap &other) :
    Submap(other.id_, other.parentId_) {

  colorCropper_ = other.colorCropper_;
  denseMap_ = other.denseMap_;
  voxelMap_ = other.voxelMap_;
  scanCounter_ = other.scanCounter_;
  carvingStatisticsTimer_ = other.carvingStatisticsTimer_;
  parentId_ = other.parentId_;
  isCenterComputed_ = other.isCenterComputed_;
  id_ = other.id_;
  feature_ = other.feature_;
  nScansInsertedDenseMap_ = other.nScansInsertedDenseMap_;
  nScansInsertedMap_ = other.nScansInsertedMap_;
  featureTimer_ = other.featureTimer_;
  params_ = other.params_;
  denseMapCropper_ = other.denseMapCropper_;
  mapBuilderCropper_ = other.mapBuilderCropper_;
  submapCenter_ = other.submapCenter_;
  mapToRangeSensor_ = other.mapToRangeSensor_;
  mapToSubmap_ = other.mapToSubmap_;
  mapCloud_ = other.mapCloud_;
  sparseMapCloud_ = other.sparseMapCloud_;

//	update(params_);
}

const Transform& Submap::getMapToSubmapOrigin() const {
	return mapToSubmap_;
}

Eigen::Vector3d Submap::getMapToSubmapCenter() const {
	return isCenterComputed_ ? submapCenter_ : mapToSubmap_.translation();
}

const Submap::PointCloud& Submap::getMapPointCloud() const {
	return mapCloud_;
}
PointCloud Submap::getMapPointCloudCopy() const {
	std::lock_guard<std::mutex> lck(mapPointCloudMutex_);
	auto copy = mapCloud_;
	return std::move(copy);
}
const VoxelizedPointCloud& Submap::getDenseMap() const {
	return denseMap_;
}

VoxelizedPointCloud Submap::getDenseMapCopy() const {
	std::lock_guard<std::mutex> lck(denseMapMutex_);
	auto copy = denseMap_;
	return std::move(copy);
}

const Submap::PointCloud& Submap::getSparseMapPointCloud() const {
	return sparseMapCloud_;
}

void Submap::setMapToSubmapOrigin(const Transform &T) {
	mapToSubmap_ = T;
}

void Submap::update(const MapperParameters &p) {
	mapBuilderCropper_ = croppingVolumeFactory(p.mapBuilder_.cropper_);
	denseMapCropper_ = croppingVolumeFactory(p.denseMapBuilder_.cropper_);
	denseMap_ = std::move(VoxelizedPointCloud(Eigen::Vector3d::Constant(p.denseMapBuilder_.mapVoxelSize_)));

	//todo remove magic
	voxelMap_ = std::move(
			VoxelMap(
					Eigen::Vector3d::Constant(
							magic::voxelExpansionFactorAdjacencyBasedRevisiting * p.mapBuilder_.mapVoxelSize_)));
}

bool Submap::isEmpty() const {
	return mapCloud_.points_.empty();
}

const VoxelMap& Submap::getVoxelMap() const {
	return voxelMap_;
}

void Submap::computeFeatures() {
	if (feature_ != nullptr
			&& featureTimer_.elapsedSec() < params_.submaps_.minSecondsBetweenFeatureComputation_) {
		return;
	}

	std::thread computeVoxelMapThread([this]() {
//		Timer t("compute_voxel_submap");
		voxelMap_.clear();
		voxelMap_.insertCloud(voxelMapLayer,mapCloud_);
	});

	auto mapCopy = getMapPointCloudCopy();
	const auto &p = params_.placeRecognition_;
	sparseMapCloud_ = *(mapCopy.VoxelDownSample(p.featureVoxelSize_));
	sparseMapCloud_.EstimateNormals(
			open3d::geometry::KDTreeSearchParamHybrid(p.normalEstimationRadius_, p.normalKnn_));
	sparseMapCloud_.NormalizeNormals();
	sparseMapCloud_.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
	feature_ = registration::ComputeFPFHFeature(sparseMapCloud_,
			open3d::geometry::KDTreeSearchParamHybrid(p.featureRadius_, p.featureKnn_));
	computeVoxelMapThread.join();
	featureTimer_.reset();
}

const Submap::Feature& Submap::getFeatures() const {
	assert_nonNullptr(feature_, "Feature ptr is nullptr");
	return *feature_;
}

void Submap::computeSubmapCenter() {
	auto mapCopy = getMapPointCloudCopy();
	submapCenter_ = mapCopy.GetCenter();
	isCenterComputed_ = true;
}

} // namespace o3d_slam
