/*
 * helpers.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/MeshBase.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Transform.hpp"

namespace o3d_slam {

class CroppingVolume;
class VoxelizedPointCloud;

std::shared_ptr<open3d::geometry::PointCloud> transform(const Eigen::Matrix4d &T,
		const open3d::geometry::PointCloud &cloud);

std::shared_ptr<open3d::geometry::PointCloud> voxelizeWithinCroppingVolume(double voxel_size,
		const CroppingVolume &croppingVolume, const open3d::geometry::PointCloud &cloud);
void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud *pcl);
void voxelize(double voxelSize, open3d::geometry::PointCloud *pcl);

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl);

std::pair<std::vector<double>, std::vector<size_t>> computePointCloudDistance(
		const open3d::geometry::PointCloud &reference, const open3d::geometry::PointCloud &cloud,
		const std::vector<size_t> &idsInReference);

void removeByIds(const std::vector<size_t> &ids, open3d::geometry::PointCloud *cloud);
std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud &scan,
		const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &sensorPosition,
		const SpaceCarvingParameters &param);
std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud &scan,
		const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &sensorPosition,
		const std::vector<size_t> &cloudIdxsSubset, const SpaceCarvingParameters &param);

void computeIndicesOfOverlappingPoints(const open3d::geometry::PointCloud &source,
		const open3d::geometry::PointCloud &target, const Transform &sourceToTarget, double voxelSize,
		size_t minNumPointsPerVoxel, std::vector<size_t> *idxsSource, std::vector<size_t> *idxsTarget);

Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs);
double informationMatrixMaxCorrespondenceDistance(double mappingVoxelSize);
double icpMaxCorrespondenceDistance(double mappingVoxelSize);
double getMapVoxelSize(const MapBuilderParameters &p, double valueIfZero);
bool isValidColor(const Eigen::Vector3d &c);

Eigen::Vector3d computeCenter(const VoxelizedPointCloud &voxels);
std::vector<Eigen::Vector3i> getKeysOfCarvedPoints(const PointCloud &scan,
		const VoxelizedPointCloud &cloud, const Eigen::Vector3d &sensorPosition, const SpaceCarvingParameters &param);

std::shared_ptr<PointCloud> removePointsWithNonFiniteValues(const PointCloud& in);

} /* namespace o3d_slam */
