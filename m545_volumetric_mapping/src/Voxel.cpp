/*
 * Voxel.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Voxel.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include <numeric>
#include <iostream>

namespace m545_mapping {

//std::vector<size_t> Voxel::idxsAsVector() const {
//	std::vector<size_t> ret;
//	ret.insert(ret.end(), idxs_.begin(), idxs_.end());
//	return ret;
//}

void VoxelMap::buildFromCloud(const open3d::geometry::PointCloud &cloud) {
	std::vector<size_t> idxs(cloud.points_.size());
	std::iota(idxs.begin(),idxs.end(),0);
	buildFromCloud(cloud, idxs);
}

VoxelMap::VoxelMap():VoxelMap(Eigen::Vector3d::Constant(0.25)){}

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

void VoxelMap::clearIndicesOnly(){
    for(auto iter = voxels_.begin(); iter != voxels_.end(); ++iter){
        iter->second.idxs_.clear();
    }
}

void VoxelMap::buildFromCloud(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs){
//	Timer t("hash_map_build");
	voxels_.reserve(idxs.size());
//		double inserting = 0;
//		double computing = 0;
		for (size_t i = 0; i < idxs.size(); ++i) {
//			Timer t;
			const size_t idx = idxs[i];
			const auto voxelIdx = getVoxelIdx(cloud.points_[idx], voxelSize_);
//			std::cout<<"Voxel idx: " << voxelIdx.transpose() << "\n";
			voxels_[voxelIdx].idxs_.emplace_back(idx);

////			computing +=t.elapsedMsec();
////			Timer t2;
//			// Don't care about duplicates

//			inserting +=t2.elapsedMsec();
		}
	//	std::cout << "hash map size: " << voxels_.size()<<", point cloud size: " <<cloud.points_.size() << std::endl;
	//	std::cout << "num buckets: " << voxels_.bucket_count() <<"\n";
//		std::cout << "computing: " << computing <<", inserting: "<<inserting <<"\n";


}

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize) {
	Eigen::Vector3d coord = p.array() / voxelSize.array();
	return Eigen::Vector3i(int(std::floor(coord(0))), int(std::floor(coord(1))), int(std::floor(coord(2))));
}

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize,
		const Eigen::Vector3d &minBound) {
	Eigen::Vector3d coord = (p - minBound).array() / voxelSize.array();
	return Eigen::Vector3i(int(std::floor(coord(0))), int(std::floor(coord(1))), int(std::floor(coord(2))));
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud,
		const Eigen::Vector3d &voxelSize) {
	const Eigen::Vector3d voxelMinBound = cloud.GetMinBound() - voxelSize * 0.5;
	const Eigen::Vector3d voxelMaxBound = cloud.GetMaxBound() + voxelSize * 0.5;
	return {voxelMinBound, voxelMaxBound};
}

} // namespace m545_mapping
