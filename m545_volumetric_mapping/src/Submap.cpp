/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include <algorithm>
#include <numeric>
#include <utility>

namespace m545_mapping {

Submap::Submap() {
	update(params_);
}

bool Submap::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Eigen::Isometry3d &mapToRangeSensor) {

	mapToRangeSensor_ = mapToRangeSensor;
	auto transformedCloud = transform(mapToRangeSensor.matrix(), preProcessedScan);
	estimateNormalsIfNeeded(transformedCloud.get());
	carve(rawScan, mapToRangeSensor, *mapBuilderCropper_, params_.mapBuilder_.carving_, &map_, &carvingTimer_);
	map_ += *transformedCloud;
	mapBuilderCropper_->setPose(mapToRangeSensor);
	voxelizeInsideCroppingVolume(*mapBuilderCropper_, params_.mapBuilder_, &map_);

	if (params_.isBuildDenseMap_) {
//		Timer timer("merge_dense_map");
		denseMapCropper_->setPose(mapToRangeSensor);
		auto denseCropped = denseMapCropper_->crop(*transformedCloud);
		carve(rawScan, mapToRangeSensor, *denseMapCropper_, params_.denseMapBuilder_.carving_, &denseMap_,
				&carveDenseMapTimer_);
		denseMap_ += *denseCropped;
		auto voxelizedDense = voxelizeWithinCroppingVolume(params_.denseMapBuilder_.mapVoxelSize_, *denseMapCropper_,
				denseMap_);
		denseMap_ = *voxelizedDense;
	}

	return true;
}

void Submap::carve(const PointCloud &rawScan, const Eigen::Isometry3d &mapToRangeSensor, const CroppingVolume &cropper,
		const SpaceCarvingParameters &params, PointCloud *map, Timer *timer) const {
	if (map->points_.empty() || timer->elapsedSec() < params.carveSpaceEveryNsec_) {
		return;
	}
//	Timer timer("carving");
	auto scan = transform(mapToRangeSensor.matrix(), rawScan);
	const auto wideCroppedIdxs = cropper.getIndicesWithinVolume(*map);
	auto idxsToRemove = std::move(
			getIdxsOfCarvedPoints(*scan, *map, mapToRangeSensor.translation(), wideCroppedIdxs, params));
	toRemove_ = std::move(*(map->SelectByIndex(idxsToRemove)));
	scanRef_ = std::move(*scan);
//	std::cout << "Would remove: " << idxsToRemove.size() << std::endl;
	removeByIds(idxsToRemove, map);
	timer->reset();
}

void Submap::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (!pcl->HasNormals() && params_.scanMatcher_.icpObjective_ == IcpObjective::PointToPlane) {
		estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals(); //todo, dunno if I need this
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

const Eigen::Isometry3d& Submap::getMapToSubmap() const {
	return mapToSubmap_;
}
const Submap::PointCloud& Submap::getMap() const {
	return map_;
}
const Submap::PointCloud& Submap::getDenseMap() const {
	return denseMap_;
}

void Submap::setMapToSubmap(const Eigen::Isometry3d &T) {
	mapToSubmap_ = T;
}

void Submap::update(const MapperParameters &p) {
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilder_.scanCroppingRadius_);
	denseMapCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.denseMapBuilder_.scanCroppingRadius_);
}

bool Submap::isEmpty() const {
	return map_.points_.empty();
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
	createNewSubmap(mapToRangeSensor_);
}

void SubmapCollection::setMapToRangeSensor(const Eigen::Isometry3d &T) {
	mapToRangeSensor_ = T;
}

bool SubmapCollection::isEmpty() const {
	return submaps_.empty();
}

const SubmapCollection::Submaps& SubmapCollection::getSubmaps() const {
	return submaps_;
}

size_t SubmapCollection::getTotalNumPoints() const {
	const int nSubmaps = submaps_.size();
	return std::accumulate(submaps_.begin(), submaps_.end(), 0, [](size_t sum, const Submap &s) {
		return sum + s.getMap().points_.size();
	});
}

void SubmapCollection::updateActiveSubmap() {
	if (numScansMergedInActiveSubmap_ < params_.submaps_.minNumRangeData_) {
		return;
	}
	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	Eigen::Vector3d submapPosition = submaps_.at(closestMapIdx).getMapToSubmap().translation();
	const bool isOriginWithinRange = (mapToRangeSensor_.translation() - submapPosition).norm() < params_.submaps_.radius_;
	if (isOriginWithinRange) {
		activeSubmapIdx_ = closestMapIdx;
	} else {
		createNewSubmap(mapToRangeSensor_);
	}
}

void SubmapCollection::createNewSubmap(const Eigen::Isometry3d &mapToSubmap) {
	Submap newSubmap;
	newSubmap.setMapToSubmap(mapToSubmap);
	newSubmap.setParameters(params_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
	numScansMergedInActiveSubmap_ = 0;
}

size_t SubmapCollection::findClosestSubmap(const Eigen::Isometry3d &mapToRangeSensor) const {

	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	auto lessThan = [this, &mapToRangeSensor](size_t idxa, size_t idxb) {
		const auto p0 = mapToRangeSensor.translation();
		const auto pa = submaps_.at(idxa).getMapToSubmap().translation();
		const auto pb = submaps_.at(idxb).getMapToSubmap().translation();
		return (p0 - pa).norm() < (p0 - pb).norm();
	};
	return *(std::min_element(idxs.begin(), idxs.end(), lessThan));
}

const Submap& SubmapCollection::getActiveSubmap() const {
	return submaps_.at(activeSubmapIdx_);
}

bool SubmapCollection::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Eigen::Isometry3d &mapToRangeSensor) {

	mapToRangeSensor_ = mapToRangeSensor;
	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor);
		++numScansMergedInActiveSubmap_;
		return true;
	}
	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap();
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if (isActiveSubmapChanged) {
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor);
	}
	submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor);
	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
}

} // namespace m545_mapping
