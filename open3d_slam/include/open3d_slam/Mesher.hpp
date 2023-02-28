//
// Created by peyschen on 20/02/23.
//

#ifndef O3D_SLAM_MESHER_HPP
#define O3D_SLAM_MESHER_HPP
#include <iostream>
#include <utility>
#include "MeshVoxel.hpp"
#include "Parameters.hpp"
#include "croppers.hpp"

namespace o3d_slam {
class Mesher {
 public:
  using MeshMapPtr = std::shared_ptr<MeshMap>;

  struct MeshSubMap {
    size_t submapId_;
    MeshMapPtr map_;

    MeshSubMap(const size_t& submapId, MeshMapPtr map) : submapId_(submapId), map_(std::move(map)){};
  };

  Mesher() { addNewSubmap(activeMapIdx_); }

  void addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc);
  MeshMapPtr getActiveMeshMap() { return meshMaps_.at(activeMapIdx_).map_; };

  void switchActiveSubmap(size_t newSubmapId);
  size_t getActiveMeshMapId() const { return activeMapIdx_; };

  void mesh();
  void removePoints(const PointCloud& pts);

  void updateParameters();
  void setParameters(MeshingParameters params) {
    params_ = params;
    updateParameters();
  };

 private:
  std::unordered_map<size_t, MeshSubMap> meshMaps_;
  size_t activeMapIdx_ = 0;
  MeshingParameters params_;

  void addNewSubmap(size_t submapId) {
    meshMaps_.insert(std::make_pair(
        submapId, MeshSubMap(submapId, std::make_shared<MeshMap>(params_.downsamplingVoxelSize_, params_.newVertexDistanceThreshold_,
                                                                 params_.meshingVoxelSize_, params_.voxelDilationRatio_,
                                                                 params_.shouldFilter_, params_.filterEps_, params_.filterRadius_))));
  }
};
}  // namespace o3d_slam
#endif  // O3D_SLAM_MESHER_HPP
