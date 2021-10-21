/*
 * Voxel.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Voxel.hpp"
#include "m545_volumetric_mapping/time.hpp"

#include <iostream>

namespace m545_mapping {

//std::vector<size_t> Voxel::idxsAsVector() const {
//	std::vector<size_t> ret;
//	ret.insert(ret.end(), idxs_.begin(), idxs_.end());
//	return ret;
//}

void VoxelMap::buildFromCloud(const open3d::geometry::PointCloud &cloud) {
	voxels_.reserve(cloud.points_.size());
//	const auto bound= computeVoxelBounds(cloud,voxelSize_);
//	double inserting = 0;
//	double computing = 0;
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
//		Timer t;
		const auto voxelIdx = getVoxelIdx(cloud.points_[i], voxelSize_);
//		computing +=t.elapsedMsec();
//		Timer t2;
		// Don't care about duplicates
		voxels_[voxelIdx].idxs_.push_back(i);
//		inserting +=t2.elapsedMsec();
	}
//	std::cout << "hash map size: " << voxels_.size()<<", point cloud size: " <<cloud.points_.size() << std::endl;
//	std::cout << "num buckets: " << voxels_.bucket_count() <<"\n";
//	std::cout << "computing: " << computing / cloud.points_.size() <<", inserting: "<<inserting/cloud.points_.size() <<"\n";
//	std::cout << "min bound: " << bound.first.transpose() << std::endl;
}

VoxelMap::VoxelMap(const Eigen::Vector3d &voxelSize) :
		voxelSize_(voxelSize) {
}

std::vector<size_t> VoxelMap::getIndicesInVoxel(const Eigen::Vector3d &p) const {
	const auto voxelIdx = getVoxelIdx(p, voxelSize_);
	const auto search = voxels_.find(voxelIdx);
	if (search != voxels_.end()) {
		return search->second.idxs_;
	}
	return std::vector<size_t>();
}

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize) {
	Eigen::Vector3d coord = p.array() / voxelSize.array();
	return Eigen::Vector3i(int(floor(coord(0))), int(floor(coord(1))), int(floor(coord(2))));
}

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize,
		const Eigen::Vector3d &minBound) {
	Eigen::Vector3d coord = (p - minBound).array() / voxelSize.array();
	return Eigen::Vector3i(int(floor(coord(0))), int(floor(coord(1))), int(floor(coord(2))));
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud,
		const Eigen::Vector3d &voxelSize) {
	const Eigen::Vector3d voxelMinBound = cloud.GetMinBound() - voxelSize * 0.5;
	const Eigen::Vector3d voxelMaxBound = cloud.GetMaxBound() + voxelSize * 0.5;
	return {voxelMinBound, voxelMaxBound};
}

} // namespace m545_mapping
