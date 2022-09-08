/*
 * VoxelHashMap.hpp
 *
 *  Created on: Mar 30, 2022
 *      Author: jelavice
 */

/*
 * Voxel.hpp
 *
 *  Created on: Oct 19, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <unordered_map>
#include <map>
#include <open3d_slam/typedefs.hpp>

namespace o3d_slam {

struct EigenVec3iHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const Eigen::Vector3i& index) const {
    // same a OpenVDB
//    return ((1 << 20) - 1) & (index.x() * 73856093 ^ index.y() * 19349663 ^ index.z() * 83492791);
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

struct InverseVoxelSize {
  double invSizeX_ = 1.0 / 0.1;
  double invSizeY_ = 1.0 / 0.1;
  double invSizeZ_ = 1.0 / 0.1;
};

inline InverseVoxelSize fromVoxelSize(const Eigen::Vector3d &voxSize){
  return InverseVoxelSize{1.0 / voxSize.x(), 1.0 / voxSize.y(), 1.0 / voxSize.z()};
}

inline Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const InverseVoxelSize &invSize) {
  return Eigen::Vector3i(int(std::floor(p.x() * invSize.invSizeX_)),
      int(std::floor(p.y() * invSize.invSizeY_)), int(std::floor(p.z() * invSize.invSizeZ_)));
}

inline Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize) {
	Eigen::Vector3d coord = p.array() / voxelSize.array();
	return Eigen::Vector3i(int(std::floor(coord(0))), int(std::floor(coord(1))), int(std::floor(coord(2))));
}

inline Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize,
		const Eigen::Vector3d &minBound) {
	Eigen::Vector3d coord = (p - minBound).array() / voxelSize.array();
	return Eigen::Vector3i(int(std::floor(coord(0))), int(std::floor(coord(1))), int(std::floor(coord(2))));
}

inline std::pair<Eigen::Vector3d, Eigen::Vector3d> computeVoxelBounds(const open3d::geometry::PointCloud &cloud,
		const Eigen::Vector3d &voxelSize) {
	const Eigen::Vector3d voxelMinBound = cloud.GetMinBound() - voxelSize * 0.5;
	const Eigen::Vector3d voxelMaxBound = cloud.GetMaxBound() + voxelSize * 0.5;
	return {voxelMinBound, voxelMaxBound};
}

inline Eigen::Vector3d getVoxelCenter(const Eigen::Vector3i &key, const Eigen::Vector3d &voxelSize) {
    return key.cast<double>().array() * voxelSize.array() + voxelSize.array() * 0.5;
}

inline Eigen::Vector3d getCenterOfCorrespondingVoxel(const Eigen::Vector3d &p, const Eigen::Vector3d &voxelSize) {
    return getVoxelCenter(getVoxelIdx(p,voxelSize),voxelSize);
}

inline Eigen::Vector3d getVoxelLowerBound(const Eigen::Vector3i &key, const Eigen::Vector3d &voxelSize) {
    return key.cast<double>().array() * voxelSize.array();
}

inline Eigen::Vector3d getVoxelUpperBound(const Eigen::Vector3i &key, const Eigen::Vector3d &voxelSize) {
    return key.cast<double>().array() * voxelSize.array() + voxelSize.array();
}

template<typename Scalar>
inline bool isWithinBounds(const Eigen::Matrix<Scalar,Eigen::Dynamic,1> &val, const Eigen::Matrix<Scalar,Eigen::Dynamic,1> &lower, const Eigen::Matrix<Scalar,Eigen::Dynamic,1> &upper){
    return (lower.array() <= val.array()).all() && (val.array() <= upper.array()).all();
}

inline bool isFirstVoxelWithinSecondVoxel(const Eigen::Vector3i &key1, const Eigen::Vector3d &voxelSize1, const Eigen::Vector3i &key2, const Eigen::Vector3d &voxelSize2){
    const Eigen::Vector3d secondVoxelLowerBound = getVoxelLowerBound(key2, voxelSize2);
    const Eigen::Vector3d secondVoxelUpperBound = getVoxelUpperBound(key2, voxelSize2);
    const Eigen::Vector3d firstVoxelCenter = getVoxelCenter(key1, voxelSize1);
    return isWithinBounds<double>(firstVoxelCenter, secondVoxelLowerBound, secondVoxelUpperBound);
}

std::vector<Eigen::Vector3i> getSmallerVoxelsWithinBigVoxel(const Eigen::Vector3i &bigVoxelKey, const Eigen::Vector3d &bigVoxelSize, const Eigen::Vector3d &smallVoxelSize);
std::vector<Eigen::Vector3i> getVoxelsWithinPointNeighborhood(const Eigen::Vector3d &p,
    double neighborhoodRadius, const Eigen::Vector3d &smallVoxelSize);
template<typename Voxel>
class VoxelHashMap {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Voxel_t = Voxel;
  using ContainerImpl_t = std::unordered_map<Eigen::Vector3i, Voxel_t, EigenVec3iHash>;
	VoxelHashMap() :
			VoxelHashMap(Eigen::Vector3d::Constant(0.25)) {
	}
  VoxelHashMap(const Eigen::Vector3d &voxelSize) :
      voxelSize_(voxelSize) {
    inverseVoxelSize_ = fromVoxelSize(voxelSize);
  }
	virtual ~VoxelHashMap() = default;

	bool hasVoxelContainingPoint(const Eigen::Vector3d &p) const {
		const auto voxelIdx = getVoxelIdx(p, inverseVoxelSize_);
		const auto search = voxels_.find(voxelIdx);
		return search != voxels_.end();
	}
	bool hasVoxelWithKey(const Eigen::Vector3i &key) const {
		const auto search = voxels_.find(key);
		return search != voxels_.end();
	}
	size_t size() const {
		return voxels_.size();
	}
	void clear() {
		voxels_.clear();
	}
	bool empty() const {
		return voxels_.empty();
	}
	Eigen::Vector3i getKey(const Eigen::Vector3d &p) const {
		return getVoxelIdx(p, inverseVoxelSize_);
	}
	void removeKey(const Eigen::Vector3i &k) {
		voxels_.erase(k);
	}

	Voxel *getVoxelPtr(const Eigen::Vector3i &key) {
		auto search = voxels_.find(key);
		return search != voxels_.end() ? &(search->second) : nullptr;
	}

	Voxel *getVoxelContainingPointPtr(const Eigen::Vector3d &p) {
		const Eigen::Vector3i key = getVoxelIdx(p, inverseVoxelSize_);
		auto search = voxels_.find(key);
		return search != voxels_.end() ? &(search->second) : nullptr;
	}

	const Voxel *getVoxelPtr(const Eigen::Vector3i &key) const {
		const auto search = voxels_.find(key);
		return search != voxels_.end() ? &(search->second) : nullptr;
	}

	const Voxel *getVoxelContainingPointPtr(const Eigen::Vector3d &p) const {
		const Eigen::Vector3i key = getVoxelIdx(p, inverseVoxelSize_);
		const auto search = voxels_.find(key);
		return search != voxels_.end() ? &(search->second) : nullptr;
	}

	Eigen::Vector3d getVoxelSize() const {
		return voxelSize_;
	}

  ContainerImpl_t voxels_;
protected:
	Eigen::Vector3d voxelSize_;
	InverseVoxelSize inverseVoxelSize_;

};
} // namespace o3d_slam

