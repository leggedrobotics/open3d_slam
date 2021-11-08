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
#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>

namespace m545_mapping {

namespace {
const double featureVoxelSize = 0.5;
}

Submap::Submap() {
	update(params_);
}

bool Submap::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Eigen::Isometry3d &mapToRangeSensor) {

	mapToRangeSensor_ = mapToRangeSensor;
	auto transformedCloud = transform(mapToRangeSensor.matrix(), preProcessedScan);
	estimateNormalsIfNeeded(params_.scanMatcher_.kNNnormalEstimation_,transformedCloud.get());
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

const Eigen::Isometry3d& Submap::getMapToSubmap() const {
	return mapToSubmap_;
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

void Submap::computeFeatures() {
	const int featureKnn = 100; //todo magic
	const int normalKnn = 30;
	sparseMap_ = *(map_.VoxelDownSample(featureVoxelSize));
	sparseMap_.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(featureVoxelSize*2.0,normalKnn));
	sparseMap_.NormalizeNormals();
	sparseMap_.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
	feature_ = open3d::pipelines::registration::ComputeFPFHFeature(sparseMap_,
			open3d::geometry::KDTreeSearchParamHybrid(featureVoxelSize * 5,featureKnn));
//	std::cout <<"map num points: " << map_.points_.size() << ", sparse map: " << sparseMap_.points_.size() << "\n";
}

const Submap::Feature& Submap::getFeatures() const {
	return *feature_;
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
	const bool isOriginWithinRange = (mapToRangeSensor_.translation() - submapPosition).norm()
			< params_.submaps_.radius_;
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
		std::lock_guard<std::mutex> lck(featureComputationMutex_);
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor);
		isFinishedSubmap_ = true;
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
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

bool SubmapCollection::isFinishedSubmap() const {
	return isFinishedSubmap_;
}

bool SubmapCollection::isBuildingLoopClosureConstraints() const {
	return isBuildingLoopClosureConstraints_;
}

void SubmapCollection::computeFeaturesInLastFinishedSubmap() {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	Timer t("feature computation");

	isFinishedSubmap_ = false;
	std::cout << "computing features for submap: " << lastFinishedSubmapIdx_ << std::endl;
	submaps_.at(lastFinishedSubmapIdx_).computeFeatures();
}

void SubmapCollection::buildLoopClosureConstraints() {

	std::lock_guard<std::mutex> lck(loopClosureConstraintMutex_);
	isBuildingLoopClosureConstraints_ = true;
	// look at the last submap finished
	// iterate over all submaps, except the active one
	// prune the search!!!!
	// do the fast matching
	// refine with icp
	Timer t("loop closing");
	using namespace open3d::pipelines::registration;
	const auto edgeLengthChecker = CorrespondenceCheckerBasedOnEdgeLength(0.5);
	const auto distanceChecker = CorrespondenceCheckerBasedOnDistance(1.5 * featureVoxelSize);
//	std::vector<std::reference_wrapper<const CorrespondenceChecker>> checkers{edgeLengthChecker,distanceChecker};

	const auto &lastBuiltSubmap = submaps_.at(lastFinishedSubmapIdx_);
	const auto closeSubmapsIdxs = std::move(getCloseSubmapsIdxs(mapToRangeSensor_));
	std::cout << "submaps in the pruned set: " << closeSubmapsIdxs.size() << std::endl;
	for (const auto id : closeSubmapsIdxs) {
		std::cout << "matching submap: " << lastFinishedSubmapIdx_ << " with submap: " << id << "\n";
		auto source = lastBuiltSubmap.getSparseMap();
		const auto &sourceFeature = lastBuiltSubmap.getFeatures();
		const auto &candidateSubmap = submaps_.at(id);
		auto target = candidateSubmap.getSparseMap();
		const auto &targetFeature = candidateSubmap.getFeatures();
		RegistrationResult ransacResult;
		while(ransacResult.correspondence_set_.size() < 30 ) {
			RegistrationResult ransacResult = RegistrationRANSACBasedOnFeatureMatching(source, target, sourceFeature,
					targetFeature, true, featureVoxelSize * 1.5, TransformationEstimationPointToPoint(false), 3, {
							distanceChecker, edgeLengthChecker }, RANSACConvergenceCriteria(1000000, 0.99));
			std::cout << "source features num: " << sourceFeature.Num() << "\n";
			std::cout << "target features num: " << targetFeature.Num() << "\n";
			std::cout << "num points source: " << source.points_.size()<< "\n";
			std::cout << "num points target: " << target.points_.size() << "\n";
			std::cout << "registered num correspondences: " << ransacResult.correspondence_set_.size() << std::endl;
			std::cout << "registered with fitness: " << ransacResult.fitness_ << std::endl;
			std::cout << "registered with rmse: " << ransacResult.inlier_rmse_ << std::endl;
			std::cout << "registered with transformation: \n" << ransacResult.transformation_ << std::endl;
		}
//
//		const std::string folder =
//				"/home/jelavice/catkin_workspaces/open3d_ws/src/m545_volumetric_mapping/m545_volumetric_mapping/data/";
//		open3d::io::WritePointCloudToPCD(folder + "target.pcd", target, open3d::io::WritePointCloudOption());
//		open3d::io::WritePointCloudToPCD(folder + "source.pcd", source, open3d::io::WritePointCloudOption());
//
//		auto sourceCopy = source;
//		auto registeredGlobal = source.Transform(ransacResult.transformation_);
////		auto registeredRefined = sourceCopy.Transform(result.transformation_);
//
//		open3d::io::WritePointCloudToPCD(folder + "source_global.pcd", registeredGlobal,
//				open3d::io::WritePointCloudOption());
////		open3d::io::WritePointCloudToPCD(folder + "source_refined.pcd", registeredRefined, open3d::io::WritePointCloudOption());

	}
	isBuildingLoopClosureConstraints_ = false;
}

std::vector<size_t> SubmapCollection::getCloseSubmapsIdxs(const Eigen::Isometry3d &mapToRangeSensor) const {
	std::vector<size_t> idxs;
	const size_t nSubmaps = submaps_.size();
	idxs.reserve(nSubmaps);
	for (size_t i = 0; i < nSubmaps; ++i) {
		if (i == lastFinishedSubmapIdx_ || i == activeSubmapIdx_) {
			continue;
		}

		const double distance =
				(mapToRangeSensor_.translation() - submaps_.at(i).getMapToSubmap().translation()).norm();
		const bool isTooFar = distance > params_.submaps_.radius_ * 2.0;
		std::cout << "d: " << distance << std::endl;
		if (isTooFar) {
			continue;
		}

		idxs.push_back(i);
	}
	return idxs;
}

} // namespace m545_mapping
