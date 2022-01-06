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

#include <open3d/pipelines/registration/TransformationEstimation.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/Transform.hpp"

namespace m545_mapping {

class CroppingVolume;

std::shared_ptr<open3d::geometry::PointCloud> transform(const Eigen::Matrix4d &T,
		const open3d::geometry::PointCloud &cloud);

std::shared_ptr<open3d::geometry::PointCloud> voxelizeWithinCroppingVolume(double voxel_size,
		const CroppingVolume &croppingVolume, const open3d::geometry::PointCloud &cloud);
void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl);
void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud *pcl);
void voxelize(double voxelSize, open3d::geometry::PointCloud *pcl);

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl);
std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> icpObjectiveFactory(
		const m545_mapping::IcpObjective &obj);

std::string asString(const Transform &T);

bool isInside(const open3d::geometry::AxisAlignedBoundingBox &bbox, const Eigen::Vector3d &p);

open3d::geometry::AxisAlignedBoundingBox boundingBoxAroundPosition(const Eigen::Vector3d &low,
		const Eigen::Vector3d &high, const Eigen::Vector3d &origin = Eigen::Vector3d::Zero());

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

inline Eigen::Quaterniond fromRPY(const double roll, const double pitch, const double yaw) {

	const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
	const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
	return yaw_angle * pitch_angle * roll_angle;
}

inline Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy) {
	return fromRPY(rpy.x(), rpy.y(), rpy.z());
}

} /* namespace m545_mapping */
