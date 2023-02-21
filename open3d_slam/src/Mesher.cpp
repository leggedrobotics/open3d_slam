//
// Created by peyschen on 20/02/23.
//

#include "open3d_slam/Mesher.hpp"
namespace o3d_slam {
void Mesher::addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc) {
  MaxRadiusCroppingVolume meshCropper(15);
  meshCropper.setPose(mapToPc);
  PointCloudPtr meshInput = meshCropper.crop(pc);
  getActiveMeshMap()->addNewPointCloud(*meshInput);
}
void Mesher::mesh() {
  getActiveMeshMap()->mesh();
}

void Mesher::switchActiveSubmap(size_t newSubmapId) {
  if (meshMaps_.find(newSubmapId) == meshMaps_.end()) {
    addNewSubmap(newSubmapId);
  }
  activeMapIdx_ = newSubmapId;
}
void Mesher::updateParameters(){
  std::cout << params_.downsamplingVoxelSize_ << std::endl;
    for(auto& map : meshMaps_){
    map.second.map_->updateParameters(params_);
    }

};

}  // namespace o3d_slam
