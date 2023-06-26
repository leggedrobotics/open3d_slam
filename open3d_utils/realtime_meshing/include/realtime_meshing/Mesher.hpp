//
// Created by peyschen on 20/02/23.
//

#ifndef O3D_SLAM_MESHER_HPP
#define O3D_SLAM_MESHER_HPP
#include <open3d/geometry/TriangleMesh.h>
#include <iostream>
#include <memory>
#include <utility>
#include "realtime_meshing/Parameters.h"
#include "realtime_meshing/MeshMap.h"
#include "realtime_meshing/types.h"

class Mesher {
 public:
  using MeshMapPtr = std::shared_ptr<MeshMap>;

  struct MeshSubMap {
    size_t submapId_;
    MeshMapPtr map_;
    Eigen::Vector3d mapOrigin_;
    Eigen::Vector3d mapCenter_ = Eigen::Vector3d::Zero();
    bool hasCalculatedCenter_ = false;

    MeshSubMap(const size_t& submapId, MeshMapPtr map, Eigen::Vector3d  mapToRange)
        : submapId_(submapId), map_(std::move(map)), mapOrigin_(std::move(mapToRange)){};
    Eigen::Vector3d getCenter() { return (hasCalculatedCenter_ ? mapCenter_ : mapOrigin_); };
    void calculateMapCenter() {
      mapCenter_ = map_->getVertices().GetCenter();
      hasCalculatedCenter_ = true;
    }
  };

  Mesher() { addNewSubmap(o3d_slam::Transform::Identity()); }

  void addNewPointCloud(o3d_slam::PointCloud& pc, const o3d_slam::Transform& mapToPc);
  MeshMapPtr getActiveMeshMap() { return meshMaps_.at(activeMapIdx_).map_; };

  void updateActiveMeshMap(const o3d_slam::Transform& mapToRangeSensor);
  size_t getClosestMapIdx(const o3d_slam::Transform& mapToRangeSensor);

  size_t getActiveMeshMapId() const { return activeMapIdx_; };

  void mesh();
  void removePoints(const o3d_slam::PointCloud& pts);

  void updateParameters();
  void setParameters(MeshingParameters params) {
    params_ = params;
    updateParameters();
  };

  open3d::geometry::TriangleMesh getMesh();

 private:
  std::unordered_map<size_t, MeshSubMap> meshMaps_;
  size_t activeMapIdx_ = 0;
  size_t previousActiveMapIdx_ = 0;
  size_t nextMapId_ = 0;

  MeshingParameters params_;

  void addNewSubmap(o3d_slam::Transform mapToRange) {
    auto mapId = nextMapId_++;
    meshMaps_.insert(std::make_pair(
        mapId, MeshSubMap(mapId, std::make_shared<MeshMap>(params_.downsamplingVoxelSize_, params_.newVertexDistanceThreshold_,
                                                                 params_.meshingVoxelSize_, params_.shouldFilter_, params_.filterEps_,
                                                                 params_.filterRadius_, params_.voxelMaxUpdates_,
                                                                 params_.sliverDeletionThreshold_), mapToRange.translation())));
    previousActiveMapIdx_ = activeMapIdx_;
    activeMapIdx_ = mapId;
  }

};
#endif  // O3D_SLAM_MESHER_HPP
