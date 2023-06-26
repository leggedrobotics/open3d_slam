#ifndef O3D_SLAM_MESHVOXEL_H
#define O3D_SLAM_MESHVOXEL_H

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>
#include "realtime_meshing/Plane.hpp"
#include "realtime_meshing/types.h"

class MeshMap;

class MeshVoxel {
 public:
  MeshVoxel() : planePtr_(std::make_unique<Plane>()), parentMap_(nullptr){};
  MeshVoxel(double voxelSize, int maximumUpdateCount, MeshMap* parentMap);
  MeshVoxel(MeshVoxel&& voxel) noexcept;
  MeshVoxel(const MeshVoxel& v) = delete;
  virtual ~MeshVoxel() = default;

  MeshVoxel& operator=(const MeshVoxel& v);
  MeshVoxel& operator=(MeshVoxel&& v) noexcept;

  bool addPoint(size_t vert);
  bool removePoint(size_t vert);
  std::vector<size_t> getPoints() const { return pts_; };

  void initPlane();
  Plane* getPlanePtr() {
    if (!planePtr_->isInitialized) {
      initPlane();
    }
    return planePtr_.get();
  }

  bool isUpdated() const { return isModified_; };

  void deactivate() { isModified_ = false; };
  void activate() { isModified_ = true; }

 private:
  std::vector<size_t> pts_;
  std::unique_ptr<Plane> planePtr_;
  double voxelSize_ = 0.6;
  std::shared_ptr<MeshMap> parentMap_;
  bool isModified_ = false;
  int updateCount_ = 0;
  int maxUpdateCount_ = 50;
};

#endif  // O3D_SLAM_MESHVOXEL_H
