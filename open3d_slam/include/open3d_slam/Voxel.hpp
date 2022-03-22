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
#include <map>
#include <mutex>
#include <open3d/utility/Eigen.h>
#include <open3d/utility/Helper.h>
#include <open3d_slam/typedefs.hpp>
#include <open3d_slam/Transform.hpp>

namespace o3d_slam {
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
	VoxelMap();
	VoxelMap(const Eigen::Vector3d &voxelSize);
	void buildFromCloud(const open3d::geometry::PointCloud &cloud);
	void buildFromCloud(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs);
	std::vector<size_t> getIndicesInVoxel(const Eigen::Vector3d &p) const;
	bool hasVoxelContainingPoint(const Eigen::Vector3d &p) const;
	void clear();
	bool empty() const;

Eigen::Vector3d voxelSize_;
//std::unordered_map<Eigen::Vector3i, Voxel, open3d::utility::hash_eigen<Eigen::Vector3i>> voxels_;
std::unordered_map<Eigen::Vector3i, Voxel, EigenVec3iHash> voxels_;

};


class MultiLayerVoxelMap{

	using VoxelLayers = std::map<std::string, Voxel>;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MultiLayerVoxelMap();
	MultiLayerVoxelMap(const Eigen::Vector3d &voxelSize);
	void insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud);
	void insertCloud(const std::string &layer, const open3d::geometry::PointCloud &cloud, std::vector<size_t> &idxs);
	std::vector<size_t> getIndicesInVoxel(const std::string &layer, const Eigen::Vector3d &p) const;
	std::vector<size_t> getIndicesInVoxel(const std::string &layer, const Eigen::Vector3i &voxelKey) const;
	bool isVoxelHasLayer(const Eigen::Vector3i &key, const std::string &layer ) const;

Eigen::Vector3d voxelSize_;
std::unordered_map<Eigen::Vector3i, VoxelLayers, EigenVec3iHash> voxels_;

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

class VoxelizedPointCloud {
public:EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VoxelizedPointCloud();
	VoxelizedPointCloud(const Eigen::Vector3d &voxelSize);
	void insert(const PointCloud &cloud);
	PointCloud toPointCloud() const;
	bool hasVoxelContainingPoint(const Eigen::Vector3d &p) const;
	void clear();
	bool empty() const;
	bool hasColors() const;
	bool hasNormals() const;
	void transform(const Transform &T);

	bool isHasNormals_ =false;
	bool isHasColors_ =false;
	Eigen::Vector3d voxelSize_;
	std::unordered_map<Eigen::Vector3i, AggregatedVoxel, EigenVec3iHash> voxels_;
	//std::mutex mutex_;

};

Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize, const Eigen::Vector3d &minBound);
Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize);
std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &voxelSize);

} // namespace o3d_slam
