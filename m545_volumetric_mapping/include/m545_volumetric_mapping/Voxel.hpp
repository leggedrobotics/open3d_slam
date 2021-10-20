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

namespace m545_mapping {
class Voxel {
public:
	std::vector<size_t> ids_;
};

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize, const Eigen::Vector3d &minBound,
		const Eigen::Vector3d &maxBound);
std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize);

} // namespace m545_mapping
