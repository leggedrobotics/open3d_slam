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
#include "open3d_slam/Plane.hpp"
#include "open3d_slam/VoxelHashMap.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

struct Triangle {
  int i, j, k;
  Triangle(int i, int j, int k) : i(i), j(j), k(k){};
  bool operator==(const Triangle& other) const { return i == other.i && j == other.j && k == other.k; }
  Eigen::Vector3i toEigen() const { return {i, j, k}; }
};

class MeshMap;
class MeshVoxel {
 public:
  Eigen::Vector3d getCenter() const { return center_; }

 protected:
  Eigen::Vector3d center_;
  double voxelSize_;
  std::shared_ptr<MeshMap> parentMap_;
  MeshVoxel(double voxelSize, MeshMap* parentMap, Eigen::Vector3d center) : voxelSize_(voxelSize), parentMap_(parentMap), center_(center){};
  MeshVoxel(MeshVoxel& other) : voxelSize_(other.voxelSize_), parentMap_(other.parentMap_){};
};

class L1Voxel : MeshVoxel {
  L1Voxel(double voxelSize, MeshMap* parentMap, Eigen::Vector3d center) : MeshVoxel(voxelSize, parentMap, center){};
  L1Voxel(const L1Voxel& v) = delete;

 private:
  std::vector<Triangle*> triangles;
};

class L2Voxel : public MeshVoxel {
 public:
  L2Voxel(double voxelSize, MeshMap* parentMap, Eigen::Vector3d center)
      : MeshVoxel(voxelSize, parentMap, center), planePtr_(std::make_unique<Plane>()){};

  L2Voxel(L2Voxel&& voxel) : MeshVoxel(voxel) {
    pts_ = voxel.pts_;
    planePtr_ = std::move(voxel.planePtr_);
    isModified_ = voxel.isModified_;
  }

  L2Voxel(const L2Voxel& v) = delete;

  void addPoint(int vert);

  bool isUpdated() const { return isModified_; };
  std::vector<int> getPoints() const { return pts_; };
  Plane* getPlanePtr() {
    if (!planePtr_->isInitialized) {
      initPlane();
    }
    return planePtr_.get();
  }
  void deactivate() {isModified_=false;};

 private:
  std::vector<int> pts_;
  std::unique_ptr<Plane> planePtr_;
  bool isModified_ = false;
  void initPlane();
};

class MeshMap {
  friend class L1Voxel;
  friend class L2Voxel;

 public:
  void addNewPointCloud(const PointCloud& pc);
  void mesh();
  open3d::geometry::TriangleMesh toO3dMesh() const;
  PointCloud getVoxelCloud() const;
  PointCloud allPts_;

 private:

  open3d::geometry::KDTreeFlann kdTree_;
  // std::unordered_map<Eigen::Vector3i, L1Voxel, EigenVec3iHash> l1Voxels_;
  std::unordered_map<Eigen::Vector3i, L2Voxel, EigenVec3iHash> l2Voxels_;
  std::unordered_map<int, Triangle> triangles_;
  std::unordered_map<int, std::unordered_set<int>> vertexToTriangles_;

  double downsampleVoxelSize_ = 0.1;
  double newVertexThreshold_ = 0.15;
  double l2VoxelSize_ = 0.6;

  std::vector<int> getVoxelVertexSet(const L2Voxel& voxel);
  std::vector<Triangle> triangulateVertexSetForVoxel(L2Voxel& voxel, const std::vector<int>& vertices) const;
  std::unordered_set<int> getTriangleIndexesForVertex(const int& vertex);
  void pullTriangles(std::vector<int>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<int>& pulledIdx);
  void eraseTriangle(const int& triIdx);
  void addTriangle(const Triangle& tri);
  Eigen::Vector3i voxelIdx(const Eigen::Vector3d& p)const ;
  mutable std::mutex meshLock_;
  Eigen::Vector3d voxelCenter(const Eigen::Vector3i& i) const;
};
}  // namespace o3d_slam

#endif  // O3D_SLAM_MESHVOXEL_H
