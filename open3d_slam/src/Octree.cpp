//
// Created by peyschen on 10/02/23.
//

#include "open3d_slam/Octree.hpp"
#include "open3d_slam/math.hpp"

namespace o3d_slam {
Octree::Octree(int depth, int maxDepth, double planeThreshold, int planeUpdateThreshold, int pointUpdateThreshold, int maxCovPoints,
               int maxVoxelPoints, double voxelSize)
    : depth(depth),
      maxDepth(maxDepth),
      planeThreshold(planeThreshold),
      isLeaf(false),
      planeUpdateThreshold(planeUpdateThreshold),
      pointUpdateThreshold(pointUpdateThreshold),
      maxCovPoints(maxCovPoints),
      maxVoxelPoints(maxVoxelPoints),
      voxelSize(voxelSize) {
  planePtr = std::make_unique<Plane>();
  for (auto& leaf : leaves_) {
    leaf = nullptr;
  }
}
void Octree::initializeOctree() {
  planePtr->initialize(tempPoints);
  if (planePtr->isPlane) {
    isLeaf = true;
    if (tempPoints.size() > maxCovPoints) {
      shouldUpdateCov = false;
    }
    if (tempPoints.size() > maxVoxelPoints) {
      shouldUpdate = false;
    }
  } else {
    isLeaf = false;
    divideOctree();
  }
  isPlaneInitialized = true;
  newPointCount = 0;
}
void Octree::divideOctree() {
  if (depth >= maxDepth) {
    isLeaf = true;
    return;
  }
  for (const auto& pt : tempPoints) {
    addToOctree(pt);
  }

  for (auto& leaf : leaves_) {
    if (leaf == nullptr) {
      continue;
    }
    if (leaf->tempPoints.size() > leaf->planeUpdateThreshold) {
      leaf->initializeOctree();
    }
  }
}

void Octree::addToOctree(const PointWithCov& pt) {
  int leaf = 0;
  if (pt.point.x() > center.x()) {
    leaf |= 4;
  }
  if (pt.point.y() > center.y()) {
    leaf |= 2;
  }
  if (pt.point.z() > center.z()) {
    leaf |= 1;
  }

  if (leaves_[leaf] == nullptr) {
    leaves_[leaf] = new Octree(depth + 1, maxDepth, planeThreshold, planeUpdateThreshold, pointUpdateThreshold, maxCovPoints,
                               maxVoxelPoints, voxelSize / 2);

    leaves_[leaf]->center.x() = center.x() + ((leaf & 4) != 0 ? 1 : -1) * (voxelSize / 2);
    leaves_[leaf]->center.y() = center.y() + ((leaf & 2) != 0 ? 1 : -1) * (voxelSize / 2);
    leaves_[leaf]->center.z() = center.z() + ((leaf & 1) != 0 ? 1 : -1) * (voxelSize / 2);
  }

  leaves_[leaf]->tempPoints.push_back(pt);
  leaves_[leaf]->newPointCount++;
}

void Octree::update(const PointWithCov& pt) {
  if (shouldUpdate) {
    newPointCount++;
    allPointCount++;
  }
  if (!isPlaneInitialized) {
    tempPoints.push_back(pt);
    if (tempPoints.size() > planeUpdateThreshold) {
      initializeOctree();
    }
  } else {
    if (planePtr->isPlane) {
      if (shouldUpdateCov) {
        tempPoints.push_back(pt);
      } else {
        newPoints.push_back(pt);
      }
      if (newPointCount > pointUpdateThreshold) {
        if (shouldUpdateCov) {
          planePtr->initialize(tempPoints);
        }
        newPointCount = 0;
      }
      if (allPointCount >= maxVoxelPoints) {
        shouldUpdate = false;
        newPoints.clear();
      }
    } else {
      if (depth < maxDepth) {
        if (!tempPoints.empty()) {
          tempPoints.clear();
        }
        if (!newPoints.empty()) {
          newPoints.clear();
        }
        addToOctree(pt);
      } else {
        if (shouldUpdateCov) {
          tempPoints.push_back(pt);
        } else {
          newPoints.push_back(pt);
        }
        if (newPointCount > pointUpdateThreshold) {
          if (shouldUpdateCov) {
            planePtr->initialize(tempPoints);
          } else {
            planePtr->update(newPoints);
            newPoints.clear();
          }
          newPointCount = 0;
        }
        if (allPointCount >= maxCovPoints) {
          shouldUpdateCov = false;
          tempPoints.clear();
        }
        if (allPointCount >= maxVoxelPoints) {
          shouldUpdate = false;
          newPoints.clear();
        }
      }
    }
  }
}

}  // namespace o3d_slam