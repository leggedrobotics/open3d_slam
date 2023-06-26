//
// Created by peyschen on 13/06/23.
//

#ifndef REALTIME_MESHING_HELPERS_H
#define REALTIME_MESHING_HELPERS_H

#include <Eigen/Core>
#include <unordered_set>
#include <vector>
#include <string>
#include "Parameters.h"
#include "realtime_meshing/types.h"

o3d_slam::PointCloudPtr guidedFiltering(const o3d_slam::PointCloudPtr& in, double eps, double radius);
o3d_slam::PointCloud cropMaxRadius(const o3d_slam::PointCloud& pointCloud, const o3d_slam::Transform& rangeSensorToMap);


inline Eigen::Vector3i getVoxelIdx(const Eigen::Vector3d& point, const Eigen::Vector3d& voxelSize) {
  Eigen::Vector3d voxelCoordinates = point.array() / voxelSize.array();
  return {int(std::floor(voxelCoordinates.x())), int(std::floor(voxelCoordinates.y())), int(std::floor(voxelCoordinates.z()))};
}

template <typename T>
void appendToSet(std::unordered_set<T>& vec, const std::vector<T>& other) {
  vec.insert(other.begin(), other.end());
}

void loadParameters(const std::string& path, MeshingParameters& parameters);
#endif  // REALTIME_MESHING_HELPERS_H
