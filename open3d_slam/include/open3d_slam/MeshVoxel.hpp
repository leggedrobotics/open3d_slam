//
// Created by peyschen on 15/02/23.
//

#ifndef O3D_SLAM_MESHVOXEL_H
#define O3D_SLAM_MESHVOXEL_H

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/visualization/visualizer/O3DVisualizer.h>
#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "../../thirdparty/ikd-Tree/ikd-Tree/ikd_Tree.hpp"
#include "open3d_slam/Plane.hpp"
#include "open3d_slam/VoxelHashMap.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

struct Triangle {
  int i, j, k;
  Triangle(int i, int j, int k) : i(i), j(j), k(k){};
  bool operator==(const Triangle& other) const {
    return (i == other.i || i == other.j || i == other.k) && (j == other.i || j == other.j || j == other.k) &&
           (k == other.i || k == other.j || k == other.k);
  }
  Eigen::Vector3i toEigen() const { return {i, j, k}; }
  bool isDegenerate() const {
    std::unordered_set<int> points;
    points.insert(i);
    points.insert(j);
    points.insert(k);
    return points.size() < 3;
  }
};

class MeshMap;
class MeshVoxel {
 public:
  MeshVoxel(double voxelSize, MeshMap* parentMap) : voxelSize_(voxelSize), parentMap_(parentMap), planePtr_(std::make_unique<Plane>()){};

  MeshVoxel(MeshVoxel&& voxel) noexcept
      : voxelSize_(voxel.voxelSize_),
        parentMap_(voxel.parentMap_),
        pts_(voxel.pts_),
        planePtr_(std::move(voxel.planePtr_)),
        isModified_(voxel.isModified_){};

  MeshVoxel(const MeshVoxel& v) = delete;

  void addPoint(int vert);

  bool isUpdated() const { return isModified_; };
  std::vector<size_t> getPoints() const { return pts_; };
  Plane* getPlanePtr() {
    if (!planePtr_->isInitialized) {
      initPlane();
    }
    return planePtr_.get();
  }
  void deactivate() { isModified_ = false; };

 private:
  std::vector<size_t> pts_;
  std::unique_ptr<Plane> planePtr_;
  double voxelSize_;
  std::shared_ptr<MeshMap> parentMap_;
  bool isModified_ = false;
  void initPlane();
};

class MeshMap {
  friend class MeshVoxel;

 public:
  void addNewPointCloud(const PointCloud& pc);
  void mesh();
  open3d::geometry::TriangleMesh toO3dMesh() const;
  PointCloud allPts_;
  MeshMap(double downsampleSize, double newVertexThreshould, double voxelSize)
      : downsampleVoxelSize_(downsampleSize), newVertexThreshold_(newVertexThreshould), l2VoxelSize_(voxelSize) {
    ikdTree_ = std::make_unique<ikd::KD_TREE<Eigen::Vector3d>>(0.3, 0.6, 0.01);
  };

  void printMeshingStats() {
    std::cout << "Meshing timing stats ADDING: Avg execution time: " << addingTimer_.getAvgMeasurementMsec()
              << " msec , frequency: " << 1e3 / addingTimer_.getAvgMeasurementMsec() << " Hz \n";
    addingTimer_.reset();
    std::cout << "Meshing timing stats MESHING: Avg execution time: " << meshingTimer_.getAvgMeasurementMsec()
              << " msec , frequency: " << 1e3 / meshingTimer_.getAvgMeasurementMsec() << " Hz \n";
    meshingTimer_.reset();
  };

 private:
  std::unordered_map<Eigen::Vector3i, MeshVoxel, EigenVec3iHash> l2Voxels_;
  std::unordered_map<size_t, Triangle> triangles_;
  std::unordered_map<size_t, std::unordered_set<size_t>> vertexToTriangles_;

  double downsampleVoxelSize_ = 0.1;
  double newVertexThreshold_ = 0.1;
  double l2VoxelSize_ = 0.4;
  int meshCount_ = 0;

  size_t nextTriIdx_ = 0;

  std::vector<size_t> getVoxelVertexSet(const MeshVoxel& voxel);
  std::vector<Triangle> triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<size_t>& vertices) const;
  std::unordered_set<size_t> getTriangleIndexesForVertex(const size_t& vertex);
  void pullTriangles(const std::vector<size_t>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<size_t>& pulledIdx);
  void eraseTriangle(const size_t& triIdx);
  void addTriangle(const Triangle& tri);
  mutable std::mutex triangleLock_, verToTriLock_, voxelLock_, vertexLock_;
  std::unique_ptr<ikd::KD_TREE<Eigen::Vector3d>> ikdTree_;
  Timer addingTimer_, meshingTimer_;
  void cleanup();
};
}  // namespace o3d_slam

#endif  // O3D_SLAM_MESHVOXEL_H
