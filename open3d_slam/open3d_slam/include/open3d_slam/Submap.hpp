/*
 * Submap.hpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Feature.h>
#include <Eigen/Dense>
#include <mutex>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Transform.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/time.hpp"

namespace o3d_slam {

struct TimestampedSubmapId {
  size_t submapId_;
  Time time_;
};

class Submap {
 public:
  using PointCloud = open3d::geometry::PointCloud;
  using Feature = open3d::pipelines::registration::Feature;
  using SubmapId = size_t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Submap(size_t id, size_t parentId);
  ~Submap() = default;

  Submap(const Submap& other);

  // Interface
  void setParameters(const MapperParameters& mapperParams);
  bool insertScan(const PointCloud& rawScan, const PointCloud& preProcessedScan, const Transform& transform,
                  bool isPerformCarving);
  bool insertScanDenseMap(const PointCloud& rawScan, const Transform& transform, const Time& time, bool isPerformCarving);
  void transform(const Transform& T);
  // Information
  bool isEmpty() const;
  void computeSubmapCenter();
  void computeFeatures();

  // Accessors
  const Transform& getMapToSubmapOrigin() const;
  Eigen::Vector3d getMapToSubmapCenter() const;
  const PointCloud& getMapPointCloud() const;
  PointCloud getMapPointCloudCopy() const;
  const VoxelizedPointCloud& getDenseMap() const;
  VoxelizedPointCloud getDenseMapCopy() const;
  const Feature& getFeatures() const;
  const PointCloud& getSparseMapPointCloud() const;
  size_t getId() const;
  size_t getParentId() const;
  const VoxelMap& getVoxelMap() const;

  // Setters
  void setMapToSubmapOrigin(const Transform& T);

 private:
  void carveVectorCloud(const PointCloud& transformedScan, const Eigen::Vector3d& sensorPosition, const CroppingVolume& cropper,
             const SpaceCarvingParameters& params, PointCloud* map);
  void carveVoxelizedCloud(const PointCloud& transformedScan, const Eigen::Vector3d& sensorPosition, const SpaceCarvingParameters& params,
             VoxelizedPointCloud* cloud);
  void update(const MapperParameters& mapperParams);
  void voxelizeInsideCroppingVolume(const CroppingVolume& cropper, const MapBuilderParameters& param, PointCloud* map) const;

  PointCloud sparseMapCloud_, mapCloud_;
  Transform mapToSubmap_ = Transform::Identity();
  Transform mapToRangeSensor_ = Transform::Identity();
  Eigen::Vector3d submapCenter_ = Eigen::Vector3d::Zero();
  std::shared_ptr<CroppingVolume> denseMapCropper_, mapBuilderCropper_;
  MapperParameters params_;
  Timer featureTimer_;
  size_t nScansInsertedMap_ = 0;
  size_t nScansInsertedDenseMap_ = 0;
  std::shared_ptr<Feature> feature_;
  size_t id_ = 0;
  bool isCenterComputed_ = false;
  size_t parentId_ = 0;
  Timer carvingStatisticsTimer_;
  int scanCounter_ = 0;
  VoxelMap voxelMap_;
  VoxelizedPointCloud denseMap_;
  ColorRangeCropper colorCropper_;
  mutable std::mutex denseMapMutex_;
  mutable std::mutex mapPointCloudMutex_;

  // Comfort functions
  mutable PointCloud toRemove_;
  mutable PointCloud scanRef_;
};

}  // namespace o3d_slam
