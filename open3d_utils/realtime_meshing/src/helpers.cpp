#include "realtime_meshing/helpers.h"
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/t/geometry/PointCloud.h>
#include <yaml-cpp/yaml.h>
#include <mutex>

o3d_slam::PointCloudPtr guidedFiltering(const o3d_slam::PointCloudPtr& in, double eps, double radius) {
  open3d::geometry::KDTreeFlann kdTree;
  kdTree.SetGeometry(*in);
  o3d_slam::PointCloudPtr out = std::make_shared<o3d_slam::PointCloud>();
  std::mutex outCloudLock;
  out->points_.reserve(in->points_.size());
#pragma omp parallel for default(none) shared(in, kdTree, radius, eps, out, outCloudLock) num_threads(4)
  for (const auto& pt : in->points_) {
    std::vector<int> indices;
    std::vector<double> distances;
    kdTree.SearchRadius(pt, radius, indices, distances);
    if (indices.size() < 3) {
      continue;
    }
    o3d_slam::PointCloudPtr neighbours = in->SelectByIndex(std::vector<size_t>(indices.begin(), indices.end()));
    Eigen::MatrixXd ptMatrix(3, neighbours->points_.size());
    for (int i = 0; i < neighbours->points_.size(); i++) {
      ptMatrix.col(i) = neighbours->points_[i];
    }

    Eigen::Vector3d mean = ptMatrix.rowwise().mean();
    ptMatrix.transposeInPlace();
    Eigen::MatrixXd centered = ptMatrix.rowwise() - ptMatrix.colwise().mean();
    Eigen::Matrix3d cov = (centered.adjoint() * centered) / double(ptMatrix.rows() - 1);
    Eigen::Matrix3d e = (cov + eps * Eigen::Matrix3d::Identity()).inverse();

    Eigen::Matrix3d A = cov * e;
    Eigen::Vector3d b = mean - A * mean;
    {
      std::lock_guard<std::mutex> lck{outCloudLock};
      out->points_.emplace_back(A * pt + b);
    }
  }
  return out;
}

o3d_slam::PointCloud cropMaxRadius(const o3d_slam::PointCloud& pointCloud, const o3d_slam::Transform& rangeSensorToMap) {
  o3d_slam::PointCloud croppedPointCloud;
  auto nPoints = pointCloud.points_.size();
  croppedPointCloud.points_.reserve(nPoints);
  auto translation = rangeSensorToMap.translation();
  for (size_t i = 0; i < nPoints; ++i) {
    const auto& p = pointCloud.points_[i];
    double pointDist = (p - translation).norm();
    if (pointDist <= 15.0) {
      croppedPointCloud.points_.push_back(p);
    }
  }
  return croppedPointCloud;
}

void loadParameters(const std::string& path, MeshingParameters& parameters) {
  YAML::Node params = YAML::LoadFile(path);
  parameters.meshingVoxelSize_ = params["mesh_voxel_size"].as<double>();
  parameters.downsamplingVoxelSize_ = params["downsample_voxel_size"].as<double>();
  parameters.newVertexDistanceThreshold_ = params["new_vertex_distance_threshold"].as<double>();
  parameters.voxelMaxUpdates_ = params["voxel_max_updates"].as<int>();
  parameters.sliverDeletionThreshold_ = params["sliver_deletion_threshold"].as<double>();
  parameters.shouldFilter_ = params["should_filter"].as<bool>();
  parameters.filterEps_ = params["filter_epsilon"].as<double>();
  parameters.filterRadius_ = params["filter_radius"].as<double>();
  parameters.cloudInMapFrame_ = params["cloud_in_map_frame"].as<bool>();
  parameters.preRotateRoll_ = params["prerotate_roll"].as<double>();
  parameters.preRotatePitch_ = params["prerotate_pitch"].as<double>();
  parameters.preRotateYaw_ = params["prerotate_yaw"].as<double>();
  parameters.meshCropHeight_ = params["mesh_crop_height"].as<double>();
}
