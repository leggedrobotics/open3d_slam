//
// Created by peyschen on 20/02/23.
//

#include "open3d_slam/Mesher.hpp"
namespace o3d_slam {
void Mesher::addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc) {
  MaxRadiusCroppingVolume meshCropper(15);
  meshCropper.setPose(mapToPc);
  PointCloudPtr meshInput = meshCropper.crop(pc);
  meshMap_->addNewPointCloud(*meshInput);
}
void Mesher::mesh(){
  meshMap_->mesh();
}
}  // namespace o3d_slam
