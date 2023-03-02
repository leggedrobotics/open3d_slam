//
// Created by peyschen on 20/02/23.
//

#include "open3d_slam/Mesher.hpp"
namespace o3d_slam {
void Mesher::addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc) {
  MaxRadiusCroppingVolume meshCropper(15);
  meshCropper.setPose(mapToPc);
  PointCloudPtr meshInput = meshCropper.crop(pc);
  getActiveMeshMap()->setMapToRange(mapToPc);
  getActiveMeshMap()->addNewPointCloud(*meshInput);
}
void Mesher::mesh() {
  getActiveMeshMap()->mesh();
}

void Mesher::switchActiveSubmap(size_t newSubmapId) {
  if (meshMaps_.find(newSubmapId) == meshMaps_.end()) {
    addNewSubmap(newSubmapId);
  }
  activeMapIdx_ = newSubmapId;
}
void Mesher::updateParameters() {
  for (auto& map : meshMaps_) {
    map.second.map_->updateParameters(params_);
  }
};

void Mesher::removePoints(const PointCloud& pts) {
  getActiveMeshMap()->removePoints(pts);
}

open3d::geometry::TriangleMesh Mesher::getAggregatedMesh() {
  open3d::geometry::TriangleMesh mesh;
  for (const auto& map : meshMaps_) {
    mesh += map.second.map_->toO3dMesh();
  }
  mesh.RemoveDuplicatedVertices();
  mesh.RemoveDuplicatedTriangles();
  mesh.RemoveUnreferencedVertices();
  return mesh;
}

}  // namespace o3d_slam
