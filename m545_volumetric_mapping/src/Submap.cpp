/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/assert.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
} // namespace

Submap::Submap(size_t id) :
		id_(id) {
	update(params_);

}

int64 Submap::getId() const {
	return id_;
//	return toUniversal(creationTime_);
}

Time Submap::getCreationTime() const {
	return creationTime_;
}

bool Submap::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &time, bool isPerformCarving) {

	if (map_.points_.empty()) {
		creationTime_ = time;
	}
	mapToRangeSensor_ = mapToRangeSensor;
	auto transformedCloud = transform(mapToRangeSensor.matrix(), preProcessedScan);
	estimateNormalsIfNeeded(params_.scanMatcher_.kNNnormalEstimation_, transformedCloud.get());
	if (isPerformCarving) {
		carve(rawScan, mapToRangeSensor, *mapBuilderCropper_, params_.mapBuilder_.carving_, &map_, &carvingTimer_);
	}
	map_ += *transformedCloud;
	mapBuilderCropper_->setPose(mapToRangeSensor);
	voxelizeInsideCroppingVolume(*mapBuilderCropper_, params_.mapBuilder_, &map_);

	if (params_.isBuildDenseMap_) {
//		Timer timer("merge_dense_map");
		denseMapCropper_->setPose(mapToRangeSensor);
		auto denseCropped = denseMapCropper_->crop(*transformedCloud);
		if (isPerformCarving) {
			carve(rawScan, mapToRangeSensor, *denseMapCropper_, params_.denseMapBuilder_.carving_, &denseMap_,
					&carveDenseMapTimer_);
		}
		denseMap_ += *denseCropped;
		auto voxelizedDense = voxelizeWithinCroppingVolume(params_.denseMapBuilder_.mapVoxelSize_, *denseMapCropper_,
				denseMap_);
		denseMap_ = *voxelizedDense;
	}

	return true;
}

void Submap::carve(const PointCloud &rawScan, const Transform &mapToRangeSensor, const CroppingVolume &cropper,
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

void Submap::estimateNormalsIfNeeded(int knn, PointCloud *pcl) const {
	if (!pcl->HasNormals() && params_.scanMatcher_.icpObjective_ == IcpObjective::PointToPlane) {
		estimateNormals(knn, pcl);
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

const Transform& Submap::getMapToSubmapOrigin() const {
	return mapToSubmap_;
}

Eigen::Vector3d Submap::getMapToSubmapCenter() const {
	return isCenterComputed_ ? submapCenter_ : mapToSubmap_.translation();
}

const Submap::PointCloud& Submap::getMap() const {
	return map_;
}
const Submap::PointCloud& Submap::getDenseMap() const {
	return denseMap_;
}

const Submap::PointCloud& Submap::getSparseMap() const {
	return sparseMap_;
}

void Submap::setMapToSubmapOrigin(const Transform &T) {
	mapToSubmap_ = T;
}

void Submap::update(const MapperParameters &p) {
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilder_.scanCroppingRadius_);
	denseMapCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.denseMapBuilder_.scanCroppingRadius_);
}

bool Submap::isEmpty() const {
	return map_.points_.empty();
}

void Submap::computeFeatures() {
	if (featureTimer_.elapsedSec() < params_.submaps_.minSecondsBetweenFeatureComputation_) {
		return;
	}
	const auto &p = params_.placeRecognition_;
	sparseMap_ = *(map_.VoxelDownSample(p.featureVoxelSize_));
	sparseMap_.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(p.normalEstimationRadius_, p.normalKnn_));
	sparseMap_.NormalizeNormals();
	sparseMap_.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
	feature_ = registration::ComputeFPFHFeature(sparseMap_,
			open3d::geometry::KDTreeSearchParamHybrid(p.featureRadius_, p.featureKnn_));
//	std::cout <<"map num points: " << map_.points_.size() << ", sparse map: " << sparseMap_.points_.size() << "\n";
	featureTimer_.reset();
}

const Submap::Feature& Submap::getFeatures() const {
	assert_nonNullptr(feature_, "Feature ptr is nullptr");
	return *feature_;
}

void Submap::computeSubmapCenter() {
	submapCenter_ = map_.GetCenter();
	isCenterComputed_=true;
}

} // namespace m545_mapping
