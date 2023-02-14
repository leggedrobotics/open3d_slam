//
// Created by peyschen on 10/02/23.
//

#ifndef OPEN3D_SLAM_OCTREE_H
#define OPEN3D_SLAM_OCTREE_H
#include "Plane.hpp"
#include "typedefs.hpp"

namespace o3d_slam {
class Octree {
 public:
  Octree(int depth, int maxDepth, double planeThreshold, int planeUpdateThreshold, int pointUpdateThreshold, int maxCovPoints,
         int maxVoxelPoints, double voxelSize);
  Octree(Octree&& other) {
    tempPoints = other.tempPoints;
    newPoints = other.newPoints;
    planePtr = std::move(other.planePtr);
    center = other.center;
    voxelSize = other.voxelSize;
    maxDepth = other.maxDepth;
    depth = other.depth;
    isLeaf = other.isLeaf;
    shouldUpdate = other.shouldUpdate;
    shouldUpdateCov = other.shouldUpdateCov;
    planeThreshold = other.planeThreshold;
    isPlaneInitialized = other.isPlaneInitialized;
    newPointCount = other.newPointCount;
    allPointCount = other.allPointCount;
    planeUpdateThreshold = other.planeUpdateThreshold;
    pointUpdateThreshold = other.pointUpdateThreshold;
    maxCovPoints = other.maxCovPoints;
    maxVoxelPoints = other.maxCovPoints;
    leaves_ = std::move(other.leaves_);
  }
  void update(const PointWithCov& pt);
  void addWithoutUpdate(const PointWithCov& pt) {
    tempPoints.push_back(pt);
    newPointCount++;
  }
  void initializeOctree();
  Eigen::Vector3d center;

  double voxelSize;
  std::array<Octree*, 8> leaves_{};

  int depth;
  std::unique_ptr<Plane> planePtr;

 private:
  void divideOctree();
  void addToOctree(const PointWithCov& pt);

 protected:
  std::vector<PointWithCov> tempPoints;
  std::vector<PointWithCov> newPoints;
  int maxDepth;
  bool isLeaf;
  bool shouldUpdate = true;
  bool shouldUpdateCov = true;
  double planeThreshold;
  bool isPlaneInitialized = false;
  int newPointCount = 0;
  int allPointCount = 0;
  int planeUpdateThreshold;
  int pointUpdateThreshold;
  int maxCovPoints;
  int maxVoxelPoints;
};
}  // namespace o3d_slam

#endif  // OPEN3D_SLAM_OCTREE_H
