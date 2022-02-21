/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Submap.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"

#include <algorithm>
#include <numeric>
#include <utility>
#include <thread>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
} // namespace

Submap::Submap(size_t id, size_t parentId) :
		id_(id), parentId_(parentId) {
	update(params_);
	colorProjectionPtr_ = std::make_shared<ColorProjection>();
}

int64 Submap::getId() const {
	return id_;
//	return toUniversal(creationTime_);
}

Time Submap::getCreationTime() const {
	return creationTime_;
}

size_t Submap::getParentId() const {
	return parentId_;
}

bool Submap::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &time, bool isPerformCarving) {

	if (map_.points_.empty()) {
		creationTime_ = time;
	}
	mapToRangeSensor_ = mapToRangeSensor;
	auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), preProcessedScan);
	estimateNormalsIfNeeded(params_.scanMatcher_.kNNnormalEstimation_, transformedCloud.get());
	if (isPerformCarving) {
		carvingStatisticsTimer_.startStopwatch();
		carve(rawScan, mapToRangeSensor, *mapBuilderCropper_, params_.mapBuilder_.carving_, &map_,
				&carvingTimer_);
		const double timeMeasurement = carvingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		carvingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (carvingStatisticsTimer_.elapsedSec() > 20.0) {
			std::cout << "Space carving timing stats: Avg execution time: "
					<< carvingStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / carvingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			carvingStatisticsTimer_.reset();
		}
	}
	map_ += *transformedCloud;
	mapBuilderCropper_->setPose(mapToRangeSensor);
	voxelizeInsideCroppingVolume(*mapBuilderCropper_, params_.mapBuilder_, &map_);

	return true;
}

bool Submap::insertScanDenseMap(const PointCloud &rawScan, const Transform &mapToRangeSensor,
		const Time &time, bool isPerformCarving) {
	if (isFirstDenseScan_) {
		isFirstDenseScan_ = false;
		return false;
	}

	auto colored = colorProjectionPtr_->filterColor(rawScan);
	denseMapCropper_->setPose(Transform::Identity());
	auto cropped = denseMapCropper_->crop(colored);
	o3d_slam::voxelize(params_.denseMapBuilder_.mapVoxelSize_, cropped.get());
	auto transformedCloud = o3d_slam::transform(mapToRangeSensor.matrix(), *cropped);
	denseMapCropper_->setPose(mapToRangeSensor);
	if (isPerformCarving) {
		carve(rawScan, mapToRangeSensor, *denseMapCropper_, params_.denseMapBuilder_.carving_, &denseMap_,
				&carveDenseMapTimer_);
	}
	denseMap_ += *transformedCloud;
	if (++scanCounter_ >= params_.denseMapBuilder_.voxelizeEveryNscans_) {
		auto voxelizedDense = voxelizeWithinCroppingVolume(params_.denseMapBuilder_.mapVoxelSize_,
				*denseMapCropper_, denseMap_);
		denseMap_ = *voxelizedDense;
		scanCounter_ = 0;
	}
	return true;
}

void Submap::transform(const Transform &T) {
	const Eigen::Matrix4d mat(T.matrix());
	sparseMap_.Transform(mat);
	map_.Transform(mat);
	denseMap_.Transform(mat);
	// not really sure whether I should update this map to submap thingy, since
	// that guy is used to compute the odometry
//	mapToSubmap_ = mapToSubmap_ * T;
	mapToRangeSensor_ = mapToRangeSensor_ * T;
//	std::cout << "Submap " << getId() << " center before: " << submapCenter_.transpose() << std::endl;
	submapCenter_ = T * submapCenter_;
//	std::cout << "Submap " << getId() << " center after: " << submapCenter_.transpose() << std::endl;
//	std::cout << "test for submap " << getId() << " " << map_.GetCenter().transpose() << std::endl;
}

void Submap::carve(const PointCloud &rawScan, const Transform &mapToRangeSensor,
		const CroppingVolume &cropper, const SpaceCarvingParameters &params, PointCloud *map,
		Timer *timer) const {
	if (map->points_.empty() || timer->elapsedSec() < params.carveSpaceEveryNsec_) {
		return;
	}
//	Timer timer("carving");
	auto scan = o3d_slam::transform(mapToRangeSensor.matrix(), rawScan);
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
//	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilder_.scanCroppingRadius_);
//	denseMapCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.denseMapBuilder_.scanCroppingRadius_);
	{
		const auto &par = p.mapBuilder_.cropper_;
		mapBuilderCropper_ = croppingVolumeFactory(par.cropperName_, par.croppingRadius_, par.croppingMinZ_,
				par.croppingMaxZ_);
	}
	{
		//todo remove magic
		const auto &par = p.denseMapBuilder_.cropper_;
		denseMapCropper_ = croppingVolumeFactory(par.cropperName_, 1.2 * par.croppingRadius_, par.croppingMinZ_,
				par.croppingMaxZ_);
	}

}

bool Submap::isEmpty() const {
	return map_.points_.empty();
}

const VoxelMap &Submap::getVoxelMap() const{
	return voxelMap_;
}


void Submap::computeFeatures() {
	if (feature_ != nullptr
			&& featureTimer_.elapsedSec() < params_.submaps_.minSecondsBetweenFeatureComputation_) {
		return;
	}

	std::thread computeVoxelMapThread ([this](){
		voxelMap_.clear();
		voxelMap_.buildFromCloud(map_);
	});

	const auto &p = params_.placeRecognition_;
	sparseMap_ = *(map_.VoxelDownSample(p.featureVoxelSize_));
	sparseMap_.EstimateNormals(
			open3d::geometry::KDTreeSearchParamHybrid(p.normalEstimationRadius_, p.normalKnn_));
	sparseMap_.NormalizeNormals();
	sparseMap_.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
	feature_.reset();
	feature_ = registration::ComputeFPFHFeature(sparseMap_,
			open3d::geometry::KDTreeSearchParamHybrid(p.featureRadius_, p.featureKnn_));
//	std::cout <<"map num points: " << map_.points_.size() << ", sparse map: " << sparseMap_.points_.size() << "\n";
	computeVoxelMapThread.join();
	featureTimer_.reset();
}

const Submap::Feature& Submap::getFeatures() const {
	assert_nonNullptr(feature_, "Feature ptr is nullptr");
	return *feature_;
}

void Submap::computeSubmapCenter() {
	submapCenter_ = map_.GetCenter();
	isCenterComputed_ = true;
}

} // namespace o3d_slam
