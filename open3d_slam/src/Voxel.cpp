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

namespace o3d_slam {

 Eigen::Vector3d AggregatedVoxel::getAggregatedPosition() const {
	return aggregatedPosition_ / (numAggregatedPoints_+1e-4);
}
 Eigen::Vector3d AggregatedVoxel::getAggregatedNormal() const {
	return aggregatedNormal_ / (numAggregatedPoints_+1e-4);
}
 Eigen::Vector3d AggregatedVoxel::getAggregatedColor() const {
	return aggregatedColor_ / (numAggregatedPoints_+1e-4);
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

Eigen::Vector3i VoxelizedPointCloud::getKey(const Eigen::Vector3d &p) const{
	return getVoxelIdx(p, voxelSize_);
}

VoxelizedPointCloud::VoxelizedPointCloud(const Eigen::Vector3d &voxelSize) :
		voxelSize_(voxelSize) {
}

void VoxelizedPointCloud::clear() {
	voxels_.clear();
}

bool VoxelizedPointCloud::empty() const {
	return voxels_.empty();
}

bool VoxelizedPointCloud::hasColors() const {
	return isHasColors_;
}
bool VoxelizedPointCloud::hasNormals() const {
	return isHasNormals_;
}

void VoxelizedPointCloud::removeKey(const Eigen::Vector3i &k){
	voxels_.erase(k);
}

size_t VoxelizedPointCloud::size() const{
	return voxels_.size();
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

bool VoxelizedPointCloud::hasVoxelContainingPoint(const Eigen::Vector3d &p) const {
	const auto voxelIdx = getVoxelIdx(p, voxelSize_);
	const auto search = voxels_.find(voxelIdx);
	return search != voxels_.end();
}

bool VoxelizedPointCloud::hasVoxelWithKey(const Eigen::Vector3i &key) const{
	const auto search = voxels_.find(key);
	return search != voxels_.end();
}

void VoxelizedPointCloud::insert(const open3d::geometry::PointCloud &cloud) {
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
		const auto voxelIdx = getVoxelIdx(cloud.points_[i], voxelSize_);
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

} // namespace o3d_slam
