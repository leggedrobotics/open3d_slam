//
// Created by peyschen on 15/02/23.
//

#include "realtime_meshing/MeshVoxel.hpp"
#include "realtime_meshing/helpers.h"
#include "realtime_meshing/MeshMap.h"

MeshVoxel::MeshVoxel(double voxelSize, int maximumUpdateCount, MeshMap* parentMap)
    : voxelSize_(voxelSize), parentMap_(parentMap), planePtr_(std::make_unique<Plane>()), maxUpdateCount_(maximumUpdateCount) {}

MeshVoxel::MeshVoxel(MeshVoxel&& voxel) noexcept
    : voxelSize_(voxel.voxelSize_),
      parentMap_(std::move(voxel.parentMap_)),
      pts_(std::move(voxel.pts_)),
      planePtr_(std::move(voxel.planePtr_)),
      isModified_(voxel.isModified_),
      updateCount_(voxel.updateCount_),
      maxUpdateCount_(voxel.maxUpdateCount_) {}

MeshVoxel& MeshVoxel::operator=(const MeshVoxel& v) {
  planePtr_ = std::make_unique<Plane>(*v.planePtr_);
  voxelSize_ = v.voxelSize_;
  parentMap_ = v.parentMap_;
  pts_ = v.pts_;
  isModified_ = v.isModified_;
  updateCount_ = v.updateCount_;
  maxUpdateCount_ = v.maxUpdateCount_;
  return *this;
}
MeshVoxel& MeshVoxel::operator=(MeshVoxel&& v) noexcept {
  planePtr_ = std::move(v.planePtr_);
  voxelSize_ = v.voxelSize_;
  parentMap_ = std::move(v.parentMap_);
  pts_ = std::move(v.pts_);
  isModified_ = v.isModified_;
  updateCount_ = v.updateCount_;
  maxUpdateCount_ = v.maxUpdateCount_;
  return *this;
}

void MeshVoxel::initPlane() {
  std::vector<Eigen::Vector3d> pointSet = parentMap_->getPoints(pts_);
  planePtr_->initialize(pointSet);
}
bool MeshVoxel::addPoint(size_t vert) {
  if (updateCount_ < maxUpdateCount_) {
    pts_.push_back(vert);
    updateCount_++;
    isModified_ = true;
    return true;
  }
  return false;
}
bool MeshVoxel::removePoint(size_t vert) {
  if (updateCount_ < maxUpdateCount_) {
    pts_.erase(std::remove(pts_.begin(), pts_.end(), vert), pts_.end());
    updateCount_++;
    isModified_ = true;
    return true;
  }
  return false;
}
