//
// Created by peyschen on 15/02/23.
//

#ifndef O3D_SLAM_MESHVOXEL_H
#define O3D_SLAM_MESHVOXEL_H

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/visualization/visualizer/O3DVisualizer.h>
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "../../thirdparty/ikd-Tree/ikd-Tree/ikd_Tree.hpp"
#include "open3d_slam/Plane.hpp"
#include "open3d_slam/VoxelHashMap.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

struct Triangle {
  int i, j, k;
  Triangle(int i, int j, int k) : i(i), j(j), k(k){};
  bool operator==(const Triangle& other) const {
    return (i == other.i || i == other.j || i == other.j) && (j == other.i || j == other.j || j == other.j) &&
           (k == other.i || k == other.j || k == other.j);
  }
  Eigen::Vector3i toEigen() const { return {i, j, k}; }
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
  std::vector<int> getPoints() const { return pts_; };
  Plane* getPlanePtr() {
    if (!planePtr_->isInitialized) {
      initPlane();
    }
    return planePtr_.get();
  }
  void deactivate() { isModified_ = false; };

 private:
  std::vector<int> pts_;
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

  void removePoints(PointCloud& cloud);

 private:
  std::unordered_map<Eigen::Vector3i, MeshVoxel, EigenVec3iHash> l2Voxels_;
  std::unordered_map<int, Triangle> triangles_;
  std::unordered_map<int, std::unordered_set<int>> vertexToTriangles_;

  double downsampleVoxelSize_ = 0.1;
  double newVertexThreshold_ = 0.1;
  double l2VoxelSize_ = 0.4;

  std::vector<int> getVoxelVertexSet(const MeshVoxel& voxel);
  std::vector<Triangle> triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<int>& vertices) const;
  std::unordered_set<int> getTriangleIndexesForVertex(const int& vertex);
  void pullTriangles(std::vector<int>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<int>& pulledIdx);
  void eraseTriangle(const int& triIdx);
  void addTriangle(const Triangle& tri);
  mutable std::mutex meshLock_;
  std::unique_ptr<ikd::KD_TREE<Eigen::Vector3d>> ikdTree_;
};
}  // namespace o3d_slam

#endif  // O3D_SLAM_MESHVOXEL_H
