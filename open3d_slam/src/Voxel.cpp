/*
 * Voxel.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/time.hpp"
#include <numeric>
#include <iostream>
#include <unordered_set>

namespace o3d_slam {

const Eigen::Vector3d zero3d(0.0,0.0,0.0);

Eigen::Vector3d AggregatedVoxel::getAggregatedPosition() const {
	return numAggregatedPoints_ == 0 ? zero3d : aggregatedPosition_ / numAggregatedPoints_;
}
 Eigen::Vector3d AggregatedVoxel::getAggregatedNormal() const {
	return numAggregatedPoints_ == 0 ? zero3d : aggregatedNormal_ / numAggregatedPoints_;
}
 Eigen::Vector3d AggregatedVoxel::getAggregatedColor() const {
	return numAggregatedPoints_ == 0 ? zero3d : aggregatedColor_ / numAggregatedPoints_;
}
void AggregatedVoxel::aggregatePoint(const Eigen::Vector3d &p) {
	aggregatedPosition_ += p;
	++numAggregatedPoints_;
}
void AggregatedVoxel::aggregateNormal(const Eigen::Vector3d &normal) {
	aggregatedNormal_ += normal;
}
void AggregatedVoxel::aggregateColor(const Eigen::Vector3d &c) {
	aggregatedColor_ += c;
}

VoxelizedPointCloud::VoxelizedPointCloud() :
		VoxelizedPointCloud(Eigen::Vector3d::Constant(0.25)) {
}

VoxelizedPointCloud::VoxelizedPointCloud(const Eigen::Vector3d &voxelSize) :
		BASE(voxelSize) {
}

bool VoxelizedPointCloud::hasColors() const {
	return isHasColors_;
}
bool VoxelizedPointCloud::hasNormals() const {
	return isHasNormals_;
}

void VoxelizedPointCloud::transform(const Transform &T){
	std::unordered_map<Eigen::Vector3i, AggregatedVoxel, EigenVec3iHash> voxels;
	if (empty()){
			return;
		}
	voxels.reserve(voxels_.size());
		for (const auto &v : voxels_) {
			if (v.second.numAggregatedPoints_ > 0) {
				AggregatedVoxel vTransformed(v.second);
				vTransformed.aggregatedNormal_ = T * vTransformed.aggregatedNormal_;
				vTransformed.aggregatedPosition_ = T * vTransformed.aggregatedPosition_;
				voxels[v.first] = vTransformed;
			}
		}
		voxels_ = std::move(voxels);
}

void VoxelizedPointCloud::insert(const open3d::geometry::PointCloud &cloud) {
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
		const auto voxelIdx = getKey(cloud.points_[i]);
		auto search = voxels_.find(voxelIdx);
		if (search == voxels_.end()) {
			auto insertResult = voxels_.insert({voxelIdx,AggregatedVoxel()});
			if (!insertResult.second){
				std::cerr << "VoxelizedPointCloud:: Insertion failed \n";
				return;
			}
			search = insertResult.first;
		}
		search->second.aggregatePoint(cloud.points_[i]);
		if (cloud.HasNormals()) {
			search->second.aggregateNormal(cloud.normals_[i]);
			isHasNormals_ = true;
		}
		if (cloud.HasColors()) {
			search->second.aggregateColor(cloud.colors_[i]);
			isHasColors_ = true;
		}

	}
}

PointCloud VoxelizedPointCloud::toPointCloud() const {
	if (empty()){
		return PointCloud();
	}
	PointCloud ret;
	ret.points_.reserve(voxels_.size());
	if (isHasNormals_) {
		ret.normals_.reserve(voxels_.size());
	}
	if (isHasColors_) {
		ret.colors_.reserve(voxels_.size());
	}
	for (const auto &voxel : voxels_) {
		if (voxel.second.numAggregatedPoints_ > 0) {
			ret.points_.push_back(voxel.second.getAggregatedPosition());
			if (isHasNormals_) {
				ret.normals_.push_back(voxel.second.getAggregatedNormal());
			}
			if (isHasColors_) {
				ret.colors_.push_back(voxel.second.getAggregatedColor());
			}
		}
	}
	return ret;
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

VoxelMap::VoxelMap() :
		VoxelMap(Eigen::Vector3d::Constant(0.25)) {
}
VoxelMap::VoxelMap(const Eigen::Vector3d &voxelSize) :
		BASE(voxelSize) {
}

void VoxelMap::insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs) {
	for (size_t i = 0; i < idxs.size(); ++i) {
		const size_t idx = idxs[i];
		const auto voxelIdx = getKey(cloud.points_[idx]);
		voxels_[voxelIdx].idxs_[layer].emplace_back(idx);
	}
}
void VoxelMap::insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud) {
	std::vector<size_t> idxs(cloud.points_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	insertCloud(layer, cloud, idxs);
}

std::vector<size_t> VoxelMap::getIndicesInVoxel(const std::string &layer,
		const Eigen::Vector3d &p) const {
	return getIndicesInVoxel(layer, getKey(p));
}

std::vector<size_t> VoxelMap::getIndicesInVoxel(const std::string &layer,
		const Eigen::Vector3i &key) const {
	const auto searchVoxel = voxels_.find(key);
	if (searchVoxel != voxels_.end()) {
		const auto searchLayer = searchVoxel->second.idxs_.find(layer);
		if (searchLayer != searchVoxel->second.idxs_.end()) {
			return searchLayer->second;
		}
	}
	return std::vector<size_t>();
}

bool VoxelMap::isVoxelHasLayer(const Eigen::Vector3i &key, const std::string &layer) const {

	const auto searchVoxel = voxels_.find(key);
	if (searchVoxel != voxels_.end()) {
		const auto searchLayer = searchVoxel->second.idxs_.find(layer);
		if (searchLayer != searchVoxel->second.idxs_.end()) {
			return true;
		}
	}
	return false;
}

std::shared_ptr<PointCloud> removeDuplicatePointsWithinSameVoxels(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize){

	std::unordered_set<Eigen::Vector3i, EigenVec3iHash> voxelSet;
	voxelSet.reserve(cloud.points_.size());
	auto retVal = std::make_shared<PointCloud>();
	retVal->points_.reserve(cloud.points_.size());
	if (cloud.HasNormals()){
		retVal->normals_.reserve(cloud.points_.size());
	}
	if (cloud.HasColors()){
		retVal->colors_.reserve(cloud.points_.size());
	}
	const InverseVoxelSize invVoxelSize = fromVoxelSize(voxelSize);
	for (size_t i =0; i < cloud.points_.size(); ++i){
		const Eigen::Vector3i voxelIdx = getVoxelIdx(cloud.points_.at(i), invVoxelSize);
		const bool hasPointAlready = !voxelSet.insert(voxelIdx).second;
		if (hasPointAlready){
			continue;
		}
		retVal->points_.push_back(cloud.points_.at(i));
		if (cloud.HasNormals()){
			retVal->normals_.push_back(cloud.normals_.at(i));
		}
		if (cloud.HasColors()){
			retVal->colors_.push_back(cloud.colors_.at(i));
		}
	}

	return retVal;
}

} // namespace o3d_slam
