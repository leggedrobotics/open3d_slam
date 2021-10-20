/*
 * Voxel.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */




#include "m545_volumetric_mapping/Voxel.hpp"

namespace m545_mapping {


Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize, const Eigen::Vector3d &minBound,
		const Eigen::Vector3d &maxBound){
	Eigen::Vector3d coord = (p - minBound).array() / voxelSize.array();
	return Eigen::Vector3i(int(floor(coord(0))), int(floor(coord(1))), int(floor(coord(2))));
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize){
	const Eigen::Vector3d voxelMinBound = cloud.GetMinBound() - voxelSize * 0.5;
	const Eigen::Vector3d voxelMaxBound = cloud.GetMaxBound() + voxelSize * 0.5;
	return {voxelMinBound, voxelMaxBound};
}

} // namespace m545_mapping
