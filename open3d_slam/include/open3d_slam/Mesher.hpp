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
  using MeshMapPtr = std::shared_ptr<MeshMap>;

  struct MeshSubMap {
    int submapId_;
    MeshMapPtr map_;

    MeshSubMap(size_t submapId, MeshMapPtr map) : submapId_(submapId), map_(map){};
  };

  Mesher(){
      addNewSubmap(activeMapIdx_);
  };

  void addNewPointCloud(const PointCloud& pc, const Eigen::Isometry3d& mapToPc);
  MeshMapPtr getActiveMeshMap() { try{
      return meshMaps_.at(activeMapIdx_).map_;

    } catch (std::out_of_range& e) {
      std::cout << "trying to access " << activeMapIdx_ <<" when size is" << meshMaps_.size() <<std::endl;
      return nullptr;
    }
  };

  void switchActiveSubmap(size_t newSubmapId);
  size_t getActiveMeshMapId(){
    return activeMapIdx_;
  };

  void mesh();

 private:
  std::unordered_map<size_t, MeshSubMap> meshMaps_;
  size_t activeMapIdx_ = 0;
  void addNewSubmap(size_t submapId) {
    std::cout << "Adding new Submap " << submapId << std::endl;
    meshMaps_.insert(std::make_pair(submapId, MeshSubMap(submapId, std::make_shared<MeshMap>(0.3, 0.15, 0.6))));
  }
};
}  // namespace o3d_slam
#endif  // O3D_SLAM_MESHER_HPP
