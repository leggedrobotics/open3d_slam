//
// Created by peyschen on 26/06/23.
//

#ifndef RELATIME_MESHING_TYPES_H
#define RELATIME_MESHING_TYPES_H
#include <open3d_slam_utils/typedefs.hpp>
#include <open3d_slam_utils/Transform.hpp>
#include <Eigen/Core>
#include <cstdint>

struct Triangle {
  size_t i_, j_, k_;

  Triangle(size_t i, size_t j, size_t k) : i_(i), j_(j), k_(k) {};

  bool operator==(const Triangle &other) const {
    return (i_ == other.i_ || i_ == other.j_ || i_ == other.k_) && (j_ == other.i_ || j_ == other.j_ || j_ == other.k_) &&
           (k_ == other.i_ || k_ == other.j_ || k_ == other.k_);
  }

  std::vector<size_t> toVector() { return {i_, j_, k_}; };
};

struct EigenVec3dHash {
  size_t operator()(const Eigen::Vector3d &vec) const {
    size_t seed = 0;
    for (size_t i = 0; i < vec.size(); ++i) {
      double elem = vec(i);
      seed ^= std::hash<double>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct EigenVec3iHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const Eigen::Vector3i& index) const {
    // same a OpenVDB
    //    return ((1 << 20) - 1) & (index.x() * 73856093 ^ index.y() * 19349663 ^ index.z() * 83492791);
    return static_cast<unsigned int>(index.x() + index.y() * sl + index.z() * sl2);
  }
};

#endif  // RELATIME_MESHING_TYPES_H
