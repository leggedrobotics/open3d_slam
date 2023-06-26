#ifndef REALTIME_MESHING_MESHMAP_H
#define REALTIME_MESHING_MESHMAP_H

#include <Eigen/Core>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include "CDT.h"
#include "ikd_tree.hpp"
#include "realtime_meshing/Parameters.h"
#include "realtime_meshing/MeshVoxel.hpp"
#include "realtime_meshing/types.h"

class MeshMap {
  friend class MeshVoxel;

 public:
  MeshMap(double downsampleSize, double newVertexThreshold, double voxelSize, bool shouldFilter, double filterEps, double filterRadius,
          int voxelMaxUpdates, double sliverDeletionThreshold);
  MeshMap(const MeshMap& m) = delete;
  MeshMap(const MeshMap&& m) = delete;
  ~MeshMap() = default;

  MeshMap& operator=(MeshMap m) = delete;
  MeshMap& operator=(MeshMap& m) = delete;

  void setMapToRange(const Eigen::Isometry3d& mapToRange) { mapToRange_ = mapToRange; };
  void addNewPointCloud(const o3d_slam::PointCloud& pc);
  void removePoints(const o3d_slam::PointCloud& pts);

  void mesh();

  void updateParameters(MeshingParameters params);

  o3d_slam::PointCloud getVertices();
  o3d_slam::PointCloud getMeshingInput();
  open3d::geometry::TriangleMesh toO3dMesh() const;

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
  std::vector<Eigen::Vector3d> getPoints(const std::vector<size_t>& vertices) const;
  std::vector<size_t> dilateVertexSet(const std::unordered_set<size_t>& vertices);
  void insertPoint(const Eigen::Vector3d& pt);
  static double calculateSliverParameter(const std::vector<Eigen::Vector3d>& meshVertices, const CDT::Triangle& tri);

  void pullTriangles(const std::vector<size_t>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<size_t>& pulledIdx);
  void eraseTriangle(const size_t& triIdx);
  void addTriangle(const Triangle& tri);

  mutable std::mutex triangleLock_, verToTriLock_, voxelLock_, vertexLock_, meshLock_, meshCloudLock_;
  std::unique_ptr<ikdTree::KdTree<double>> ikdTree_;
  o3d_slam::PointCloudPtr mesherInput_;
};

#endif  // REALTIME_MESHING_MESHMAP_H
