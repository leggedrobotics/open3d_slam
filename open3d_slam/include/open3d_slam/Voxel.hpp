/*
 * Voxel.hpp
 *
 *  Created on: Oct 19, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>
#include <mutex>
#include <open3d_slam/VoxelHashMap.hpp>
#include <open3d_slam/Transform.hpp>

namespace o3d_slam {


struct VoxelWithIdxs {
	std::map<std::string,std::vector<size_t>> idxs_;
};


class VoxelMap : public VoxelHashMap<VoxelWithIdxs>{
	using BASE = VoxelHashMap<VoxelWithIdxs>;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VoxelMap();
	VoxelMap(const Eigen::Vector3d &voxelSize);
	void insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud);
	void insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs);
	std::vector<size_t> getIndicesInVoxel(const std::string &layer, const Eigen::Vector3d &p) const;
	std::vector<size_t> getIndicesInVoxel(const std::string &layer, const Eigen::Vector3i &voxelKey) const;
	bool isVoxelHasLayer(const Eigen::Vector3i &key, const std::string &layer ) const;

};

class VoxelizedPointCloud;
class AggregatedVoxel {
	friend class VoxelizedPointCloud;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d getAggregatedPosition() const;
	Eigen::Vector3d getAggregatedNormal() const;
	Eigen::Vector3d getAggregatedColor() const;

	int numAggregatedPoints_ = 0;
	Eigen::Vector3d aggregatedPosition_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d aggregatedNormal_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d aggregatedColor_ = Eigen::Vector3d::Zero();

private:
	// aggregate point has to be called before aggregate normal and aggregate color!!!!
	void aggregatePoint(const Eigen::Vector3d &p);
	void aggregateNormal(const Eigen::Vector3d &n);
	void aggregateColor(const Eigen::Vector3d &c);
};

class VoxelizedPointCloud : public VoxelHashMap<AggregatedVoxel> {
	using BASE = VoxelHashMap<AggregatedVoxel>;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VoxelizedPointCloud();
	VoxelizedPointCloud(const Eigen::Vector3d &voxelSize);
	void insert(const PointCloud &cloud);
	PointCloud toPointCloud() const;
	bool hasColors() const;
	bool hasNormals() const;
	void transform(const Transform &T);

	bool isHasNormals_ =false;
	bool isHasColors_ =false;
	//std::mutex mutex_;
};

std::shared_ptr<PointCloud> removeDuplicatePointsWithinSameVoxels(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize);

} // namespace o3d_slam
