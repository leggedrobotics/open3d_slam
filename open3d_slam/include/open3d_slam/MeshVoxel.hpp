//
// Created by peyschen on 15/02/23.
//

#ifndef O3D_SLAM_MESHVOXEL_H
#define O3D_SLAM_MESHVOXEL_H

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/visualization/visualizer/O3DVisualizer.h>
#include <Eigen/Core>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "../../thirdparty/ikd-Tree/ikd-Tree/ikd_tree.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Plane.hpp"
#include "open3d_slam/VoxelHashMap.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"
namespace o3d_slam {

struct Triangle {
  size_t i, j, k;
  Triangle(size_t i, size_t j, size_t k) : i(i), j(j), k(k){};
  bool operator==(const Triangle& other) const {
    return (i == other.i || i == other.j || i == other.k) && (j == other.i || j == other.j || j == other.k) &&
           (k == other.i || k == other.j || k == other.k);
  }
  std::vector<size_t> toVector() { return {i, j, k}; };
};

struct EigenVec3dHash {
  size_t operator()(const Eigen::Vector3d& vec) const {
    size_t seed = 0;
    for (size_t i = 0; i < vec.size(); ++i) {
      double elem = vec(i);
      seed ^= std::hash<double>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class MeshMap;
class MeshVoxel {
 public:
  MeshVoxel(double voxelSize, int maximumUpdateCount, MeshMap* parentMap)
      : voxelSize_(voxelSize), parentMap_(parentMap), planePtr_(std::make_unique<Plane>()) {
    maxUpdateCount_ = maximumUpdateCount;
  };

  MeshVoxel(MeshVoxel&& voxel) noexcept
      : voxelSize_(voxel.voxelSize_),
        parentMap_(std::move(voxel.parentMap_)),
        pts_(std::move(voxel.pts_)),
        planePtr_(std::move(voxel.planePtr_)),
        isModified_(voxel.isModified_),
        updateCount_(voxel.updateCount_),
        maxUpdateCount_(voxel.maxUpdateCount_){};

  MeshVoxel& operator=(const MeshVoxel& v) {
    planePtr_ = std::make_unique<Plane>(*v.planePtr_);
    voxelSize_ = v.voxelSize_;
    parentMap_ = v.parentMap_;
    pts_ = v.pts_;
    isModified_ = v.isModified_;
    updateCount_ = v.updateCount_;
    maxUpdateCount_ = v.maxUpdateCount_;
    return *this;
  }

  MeshVoxel& operator=(MeshVoxel&& v)  noexcept {
    planePtr_ = std::move(v.planePtr_);
    voxelSize_ = v.voxelSize_;
    parentMap_ = std::move(v.parentMap_);
    pts_ = std::move(v.pts_);
    isModified_ = v.isModified_;
    updateCount_ = v.updateCount_;
    maxUpdateCount_ = v.maxUpdateCount_;
    return *this;
  }
  MeshVoxel(const MeshVoxel& v) = delete;
  MeshVoxel() : planePtr_(std::make_unique<Plane>()), parentMap_(nullptr) {};

  bool addPoint(size_t vert);
  bool removePoint(size_t vert);
  void initPlane();

  bool isUpdated() const { return isModified_; };
  std::vector<size_t> getPoints() const { return pts_; };
  Plane* getPlanePtr() {
    if (!planePtr_->isInitialized) {
      initPlane();
    }
    return planePtr_.get();
  }
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

class MeshMap {
  friend class MeshVoxel;

 public:
  void setMapToRange(Eigen::Isometry3d mapToRange) { mapToRange_ = mapToRange; };
  void addNewPointCloud(const PointCloud& pc);
  void mesh();
  open3d::geometry::TriangleMesh toO3dMesh() const;
  MeshMap(double downsampleSize, double newVertexThreshold, double voxelSize, double dilationRatio, bool shouldFilter, double filterEps,
          double filterRadius, int voxelMaxUpdates, double sliverDeletionThreshold)
      : downsampleVoxelSize_(downsampleSize),
        newVertexThreshold_(newVertexThreshold),
        voxelSize_(voxelSize),
        dilationRatio_(dilationRatio),
        shouldFilter_(shouldFilter),
        filterEps_(filterEps),
        filterRadius_(filterRadius),
        voxelMaxUpdateCount_(voxelMaxUpdates),
        sliverThreshold_(sliverDeletionThreshold) {
    std::cout << "#########################\n"
              << "## MeshMap initialized ##\n"
              << "#########################\n";
    ikdTree_ = std::make_unique<ikdTree::KdTree<double>>(0.3, 0.6, 0.01);
  };

  void printMeshingStats();
  void updateParameters(MeshingParameters params);
  void removePoints(const PointCloud& pts);
  std::vector<Eigen::Vector3d> getVertices() {
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(points_.left.size());
    for (const auto& it : points_.left) {
      vertices.push_back(it.second);
    }
    return vertices;
  };
  PointCloud getMeshingInput() {
    std::lock_guard<std::mutex> lck{meshCloudLock_};
    if (mesherInput_ != nullptr) return *mesherInput_;
    return {};
  };

 private:
  using PointMap =
      boost::bimaps::bimap<boost::bimaps::unordered_set_of<size_t>, boost::bimaps::unordered_set_of<Eigen::Vector3d, EigenVec3dHash>>;
  using PointPair = PointMap::value_type;
  PointMap points_;
  std::unordered_map<Eigen::Vector3i, MeshVoxel, EigenVec3iHash> voxels_;
  std::unordered_map<size_t, Triangle> triangles_;
  std::unordered_map<size_t, std::unordered_set<size_t>> vertexToTriangles_;

  double downsampleVoxelSize_ = 0.1;
  double newVertexThreshold_ = 0.1;
  double voxelSize_ = 0.4;
  double sliverThreshold_ = 0.0075;
  int meshCount_ = 0;
  double dilationRatio_ = 0.5;

  std::vector<double> dilationDistances_ = {0, 5, 7.5, 10};
  std::vector<double> dilationRatios_ = {0.25, 0.35, 0.45, 0.6};

  bool shouldFilter_ = true;
  double filterEps_ = 0.01;
  double filterRadius_ = 0.3;
  int voxelMaxUpdateCount_ = 50;

  size_t nextTriIdx_ = 0;
  size_t nextVertIdx_ = 0;

  Eigen::Isometry3d mapToRange_ = Eigen::Isometry3d::Identity();

  std::vector<size_t> getVoxelVertexSet(const MeshVoxel& voxel);
  std::vector<Triangle> triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<size_t>& vertices) const;
  std::unordered_set<size_t> getTriangleIndexesForVertex(const size_t& vertex);
  void pullTriangles(const std::vector<size_t>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<size_t>& pulledIdx);
  void eraseTriangle(const size_t& triIdx);
  void addTriangle(const Triangle& tri);
  mutable std::mutex triangleLock_, verToTriLock_, voxelLock_, vertexLock_, meshLock_, meshCloudLock_;
  std::unique_ptr<ikdTree::KdTree<double>> ikdTree_;
  Timer addingTimer_, meshingTimer_;
  std::vector<Eigen::Vector3d> getPoints(const std::vector<size_t>& vertices) const;
  std::vector<size_t> dilateVertexSet(const std::unordered_set<size_t>& vertices);
  PointCloudPtr guidedFiltering(const PointCloudPtr& in, double eps, double radius);
  PointCloudPtr mesherInput_;
  void insertPoint(const Eigen::Vector3d& pt);
};
}  // namespace o3d_slam

#endif  // O3D_SLAM_MESHVOXEL_H
