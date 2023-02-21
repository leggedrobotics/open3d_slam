//
// Created by peyschen on 20/02/23.
//

#ifndef O3D_SLAM_MESHER_HPP
#define O3D_SLAM_MESHER_HPP
#include <iostream>
#include "MeshVoxel.hpp"
#include "croppers.hpp"

namespace o3d_slam {
class Mesher {
 public:
  Mesher(){
    meshMap_ = std::make_shared<MeshMap>(0.3,0.15,0.6);
  };

  void addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc);
  std::shared_ptr<MeshMap> getMeshMap(){
    return meshMap_;
  };

  void mesh();

 private:
  std::shared_ptr<MeshMap> meshMap_;

};
}  // namespace o3d_slam
#endif  // O3D_SLAM_MESHER_HPP
