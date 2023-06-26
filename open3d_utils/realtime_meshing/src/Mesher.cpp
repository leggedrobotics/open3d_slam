#include "realtime_meshing/Mesher.hpp"
#include "realtime_meshing/helpers.h"

void Mesher::addNewPointCloud(o3d_slam::PointCloud& pc, const Eigen::Isometry3d& mapToPc) {
  auto registeredCloud = pc.Transform((params_.cloudInMapFrame_?o3d_slam::Transform::Identity():mapToPc).matrix());
  o3d_slam::PointCloud croppedPointCloud = cropMaxRadius(registeredCloud, mapToPc);
  updateActiveMeshMap(mapToPc);
  getActiveMeshMap()->setMapToRange(mapToPc);
  getActiveMeshMap()->addNewPointCloud(croppedPointCloud);
}
void Mesher::mesh() {
  getActiveMeshMap()->mesh();
}

void Mesher::updateParameters() {
  for (auto& map : meshMaps_) {
    map.second.map_->updateParameters(params_);
  }
};

void Mesher::removePoints(const o3d_slam::PointCloud& pts) {
  getActiveMeshMap()->removePoints(pts);
}

open3d::geometry::TriangleMesh Mesher::getMesh() {
  open3d::geometry::TriangleMesh mesh;
  mesh = meshMaps_.at(activeMapIdx_).map_->toO3dMesh();
  if(previousActiveMapIdx_ != activeMapIdx_) {
    mesh += meshMaps_.at(previousActiveMapIdx_).map_->toO3dMesh();
  }
  mesh.RemoveDuplicatedVertices();
  mesh.RemoveDuplicatedTriangles();
  mesh.RemoveUnreferencedVertices();
  return mesh;
}

void Mesher::updateActiveMeshMap(const o3d_slam::Transform& mapToRangeSensor) {
  size_t closestMapIdx = getClosestMapIdx(mapToRangeSensor);

  auto closestMap = meshMaps_.at(closestMapIdx);
  auto activeMap = meshMaps_.at(activeMapIdx_);

  bool isSubmapInRange = (closestMap.getCenter() - mapToRangeSensor.translation()).norm() < 20;
  if(isSubmapInRange){
    if(closestMapIdx == activeMapIdx_){
      return;
    }
    previousActiveMapIdx_ = activeMapIdx_;
    activeMapIdx_ = closestMapIdx;
    std::cout << "Switched to Map " << activeMapIdx_ << std::endl;
  } else {
    addNewSubmap(mapToRangeSensor);
    meshMaps_.at(previousActiveMapIdx_).calculateMapCenter();
    std::cout << "Added new Map " << activeMapIdx_ << std::endl;
  }


}

size_t Mesher::getClosestMapIdx(const o3d_slam::Transform& mapToRangeSensor) {
  std::vector<size_t> mapIdxs(meshMaps_.size());
  std::iota(mapIdxs.begin(),mapIdxs.end(),0);

  auto closerThan = [this, &mapToRangeSensor](size_t idx1, size_t idx2){
    auto p0 = mapToRangeSensor.translation();
    auto p1 = meshMaps_.at(idx1).getCenter();
    auto p2 = meshMaps_.at(idx2).getCenter();
    return (p0 - p1).norm() < (p0-p2).norm();
  };

  return *std::min_element(mapIdxs.begin(),mapIdxs.end(),closerThan);
}