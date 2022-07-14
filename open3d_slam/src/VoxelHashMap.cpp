/*
 * VoxelHashMap.cpp
 *
 *  Created on: Jul 14, 2022
 *      Author: jelavice
 */

#include <open3d_slam/VoxelHashMap.hpp>
#include <open3d_slam/assert.hpp>

namespace o3d_slam {


std::vector<Eigen::Vector3i> getVoxelsWithinPointNeighborhood(const Eigen::Vector3d &p,
    double neighborhoodRadius, const Eigen::Vector3d &voxelSize) {

  const Eigen::Vector3i centerKey = getVoxelIdx(p, voxelSize);
  const Eigen::Vector3d step = voxelSize;
  if (neighborhoodRadius <= 0.0){
    return {centerKey};
  }

  const int ratio = std::round(neighborhoodRadius / step.minCoeff());
  std::vector<Eigen::Vector3i> retVal;
  retVal.reserve((ratio+1)*(ratio+1)*(ratio+1));
  bool isCenterKeyAdded = false;
  for (double dx = -neighborhoodRadius; dx <= neighborhoodRadius; dx+=step.x()) {
    for (double dy = -neighborhoodRadius; dy <= neighborhoodRadius; dy+=step.y()) {
      for (double dz = -neighborhoodRadius; dz <= neighborhoodRadius; dz+=step.z()) {
        const Eigen::Vector3d testPoint = p + Eigen::Vector3d(dx, dy, dz);
        const Eigen::Vector3d center = getCenterOfCorrespondingVoxel(testPoint, voxelSize);
        if ((testPoint - center).norm() <= neighborhoodRadius) {
          const Eigen::Vector3i key = getVoxelIdx(testPoint, voxelSize);
          retVal.push_back(key);
          if ((key.array() == centerKey.array()).all()){
            isCenterKeyAdded = true;
          }
        }
      }
    }
  }
  if (!isCenterKeyAdded){
    retVal.push_back(centerKey);
  }

  return retVal;
}

std::vector<Eigen::Vector3i> getSmallerVoxelsWithinBigVoxel(const Eigen::Vector3i &bigVoxelKey,
    const Eigen::Vector3d &bigVoxelSize, const Eigen::Vector3d &smallVoxelSize) {

  assert_ge(bigVoxelSize.x(), smallVoxelSize.x(), "getSmallerVoxelsWithinBigVoxel x size");
  assert_ge(bigVoxelSize.y(), smallVoxelSize.y(), "getSmallerVoxelsWithinBigVoxel y size");
  assert_ge(bigVoxelSize.z(), smallVoxelSize.z(), "getSmallerVoxelsWithinBigVoxel z size");

  // todo get big voxel center
  // has it to smaller voxel key
  // get the size ratio and cast it up
  // iterate and check whether centers are inside the big guy

  const Eigen::Vector3d bigVoxelCenter = getVoxelCenter(bigVoxelKey, bigVoxelSize);
  const Eigen::Vector3i smallVoxelKey = getVoxelIdx(bigVoxelCenter, smallVoxelSize);
  const int voxelSizeRatio = (int) std::round((bigVoxelSize.array() / smallVoxelSize.array()).maxCoeff());
  assert_ge(voxelSizeRatio, 1, "voxelSizeRatio");
  if (voxelSizeRatio == 1){
    return {smallVoxelKey}; //todo this is approximation
  }
  const int reach = std::max(1, voxelSizeRatio - 1);
  std::vector<Eigen::Vector3i> retVal;
  retVal.reserve(voxelSizeRatio * voxelSizeRatio * voxelSizeRatio);
  for (int i = -reach; i <= reach; ++i) {
    for (int j = -reach; j <= reach; ++j) {
      for (int k = -reach; k <= reach; ++k) {
        const Eigen::Vector3i testedKey = smallVoxelKey + Eigen::Vector3i(i, j, k);
        if (isFirstVoxelWithinSecondVoxel(testedKey, smallVoxelSize, bigVoxelKey, bigVoxelSize)) {
          retVal.push_back(testedKey);
        }
      }
    }
  }

  return retVal;

}

} // namespace o3d_slam

