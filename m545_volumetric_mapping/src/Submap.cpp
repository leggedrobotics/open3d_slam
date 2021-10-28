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
	update(mapBuilderParams_);
}

bool Submap::insertScan(const PointCloud &rawScan, const Eigen::Isometry3d &transformation) {

	if(map_.points_.empty()){
		insertFirstScan(rawScan,transformation);
		return true;
	}
	mapBuilderCropper_->setPose(mapToRangeSensor_);

	auto transformedCloud = transform(transformation.matrix(), rawScan);
	auto wideCroppedCloud = mapBuilderCropper_->crop(*transformedCloud);
	m545_mapping::voxelize(mapBuilderParams_.mapVoxelSize_, wideCroppedCloud.get());

	//		Timer timer("Map update");
	m545_mapping::randomDownSample(scanProcessingParams_.downSamplingRatio_, wideCroppedCloud.get());
	estimateNormalsIfNeeded(wideCroppedCloud.get());
//	carve(transformedCloud, *mapBuilderCropper_,params_.mapBuilder_.carving_,&map_,&carvingTimer_);
	map_ += *wideCroppedCloud;

	if (mapBuilderParams_.mapVoxelSize_ > 0.0) {
		//			Timer timer("voxelize_map",true);
		auto voxelizedMap = voxelizeWithinCroppingVolume(mapBuilderParams_.mapVoxelSize_, *mapBuilderCropper_, map_);
		map_ = *voxelizedMap;
	}

//	if(params_.isBuildDenseMap_){
////		Timer timer("merge_dense_map");
//		denseMapCropper_->setPose(mapToRangeSensor_);
//		auto denseCropped = denseMapCropper_->crop(transformedCloud);
//		carve(transformedCloud, *denseMapCropper_, params_.denseMapBuilder_.carving_, &denseMap_,&carveDenseMapTimer_);
//		denseMap_ += *denseCropped;
//		auto voxelizedDense = voxelizeWithinCroppingVolume(params_.denseMapBuilder_.mapVoxelSize_, *denseMapCropper_,  denseMap_);
//		denseMap_= *voxelizedDense;
//	}

	return true;
}

void Submap::insertFirstScan(const PointCloud &scan,const Eigen::Isometry3d &transform) {
	mapBuilderCropper_->setPose(mapToRangeSensor_);
	auto croppedCloud = mapBuilderCropper_->crop(scan);
	m545_mapping::voxelize(mapBuilderParams_.mapVoxelSize_, croppedCloud.get());
	croppedCloud->Transform(transform.matrix());
	estimateNormalsIfNeeded(croppedCloud.get());
	map_ += *croppedCloud;
}

void Submap::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (scanMatcherParams_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		estimateNormals(scanMatcherParams_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals(); //todo, dunno if I need this
	}
}


void Submap::setParameters(const MapBuilderParameters &mapBuilderParam, const IcpParameters &scanMatcherParameters,const ScanProcessingParameters &scanProcessingParameters){
	mapBuilderParams_ = mapBuilderParam;
	scanMatcherParams_ = scanMatcherParameters;
	scanProcessingParams_ = scanProcessingParameters;
	update(mapBuilderParam);
}

void Submap::voxelizeAroundPosition(const Eigen::Vector3d &p) {

}

const Eigen::Isometry3d& Submap::getMapToSubmap() const {
	return mapToSubmap_;
}
const Submap::PointCloud& Submap::getMap() const {
	return map_;
}
void Submap::setMapToSubmap(const Eigen::Isometry3d &T) {
	mapToSubmap_ = T;
}

void Submap::update(const MapBuilderParameters &p) {
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.scanCroppingRadius_);
}

bool Submap::isEmpty() const{
	return map_.points_.empty();
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
}

void SubmapCollection::setMapToRangeSensor(const Eigen::Isometry3d &T) {
	mapToRangeSensor_ = T;
}


bool SubmapCollection::isEmpty() const{
	return submaps_.empty();
}

void SubmapCollection::updateActiveSubmap() {
	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	Eigen::Vector3d submapPosition = submaps_.at(closestMapIdx).getMapToSubmap().translation();
	const bool isWithinRange = (mapToRangeSensor_.translation() - submapPosition).norm() < 1e6;
	//todo fix magic
	//todo ensure min num scans in each submap
	if (isWithinRange) {
		activeSubmapIdx_ = closestMapIdx;
	} else {
		createNewSubmap(mapToRangeSensor_);
	}
}

void SubmapCollection::createNewSubmap(const Eigen::Isometry3d &mapToSubmap) {
	Submap newSubmap;
	newSubmap.setMapToSubmap(mapToSubmap);
	newSubmap.setParameters(params_.mapBuilder_, params_.scanMatcher_,params_.scanProcessing_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
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

bool SubmapCollection::insertScan(const PointCloud &rawScan, const Eigen::Isometry3d &transform){

	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, transform);
		return true;
	}

	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap();
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if(isActiveSubmapChanged){
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, transform);
	}
	submaps_.at(activeSubmapIdx_).insertScan(rawScan, transform);
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
}


} // namespace m545_mapping
