/*
 * Voxel.hpp
 *
 *  Created on: Oct 19, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <open3d/geometry/PointCloud.h>
#include <unordered_map>
#include <unordered_set>
#include <open3d/utility/Eigen.h>
#include <open3d/utility/Helper.h>

namespace m545_mapping {
class Voxel {

public:
	std::vector<size_t> idxs_;
};

struct EigenVec3iHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const Eigen::Vector3i& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

class VoxelMap{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VoxelMap() = default;
	VoxelMap(const Eigen::Vector3d &voxelSize);
	void buildFromCloud(const open3d::geometry::PointCloud &cloud);
	void buildFromCloud(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs);
	std::vector<size_t> getIndicesInVoxel(const Eigen::Vector3d &p) const;
	void clearIndicesOnly();


Eigen::Vector3d voxelSize_;
//std::unordered_map<Eigen::Vector3i, Voxel, open3d::utility::hash_eigen<Eigen::Vector3i>> voxels_;
std::unordered_map<Eigen::Vector3i, Voxel, EigenVec3iHash> voxels_;

};

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize, const Eigen::Vector3d &minBound);
Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize);
std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize);

} // namespace m545_mapping
