/*
 * helpers.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/Voxel.hpp"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
#include "open3d/geometry/KDTreeFlann.h"

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;

class AccumulatedPoint {
public:
	AccumulatedPoint() :
			num_of_points_(0), point_(0.0, 0.0, 0.0), normal_(0.0, 0.0, 0.0), color_(0.0, 0.0, 0.0) {
	}

public:
	void AddPoint(const open3d::geometry::PointCloud &cloud, int index) {
		point_ += cloud.points_[index];
		if (cloud.HasNormals()) {
			if (!std::isnan(cloud.normals_[index](0)) && !std::isnan(cloud.normals_[index](1))
					&& !std::isnan(cloud.normals_[index](2))) {
				normal_ += cloud.normals_[index];
			}
		}

		if (cloud.HasColors() && isValidColor(cloud.colors_[index])) {
			color_ = cloud.colors_[index]; //+= cloud.colors_[index];
		}
		num_of_points_++;
	}

	Eigen::Vector3d GetAveragePoint() const {
		return point_ / double(num_of_points_);
	}

	Eigen::Vector3d GetAverageNormal() const {
		// Call NormalizeNormals() afterwards if necessary
		return normal_ / double(num_of_points_);
	}

	Eigen::Vector3d GetAverageColor() const {
		return color_; // / double(num_of_points_);
	}

public:
	int num_of_points_;
	Eigen::Vector3d point_;
	Eigen::Vector3d normal_;
	Eigen::Vector3d color_;
};

class point_cubic_id {
public:
	size_t point_id;
	int cubic_id;
};

} //namespace

bool isValidColor(const Eigen::Vector3d &c) {
	return (c.array().all() >= 0.0) && (c.array().all() <= 1.0);
}

double informationMatrixMaxCorrespondenceDistance(double mappingVoxelSize) {
	return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (1.5 * mappingVoxelSize);
}

double icpMaxCorrespondenceDistance(double mappingVoxelSize) {
	return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (2.0 * mappingVoxelSize);
}

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl) {
	open3d::geometry::KDTreeSearchParamKNN param(numNearestNeighbours);
	pcl->EstimateNormals(param);
}

std::shared_ptr<registration::TransformationEstimation> icpObjectiveFactory(
		const o3d_slam::IcpObjective &obj) {

	switch (obj) {
	case o3d_slam::IcpObjective::PointToPoint: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPoint>(false);
		return obj;
	}

	case o3d_slam::IcpObjective::PointToPlane: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPlane>();
		return obj;
	}

	default:
		throw std::runtime_error("Unknown icp objective");
	}

}

void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud *pcl) {
	if (downSamplingRatio >= 1.0) {
		return;
	}
	auto downSampled = pcl->RandomDownSample(downSamplingRatio);
	*pcl = std::move(*downSampled);
}
void voxelize(double voxelSize, open3d::geometry::PointCloud *pcl) {
	if (voxelSize <= 0) {
		return;
	}
	auto voxelized = pcl->VoxelDownSample(voxelSize);
	*pcl = std::move(*voxelized);
}

std::shared_ptr<open3d::geometry::PointCloud> voxelizeWithinCroppingVolume(double voxel_size,
		const CroppingVolume &croppingVolume, const open3d::geometry::PointCloud &cloud) {
	using namespace open3d::geometry;
	auto output = std::make_shared<PointCloud>();
	if (voxel_size <= 0.0) {
		*output = cloud;
		return output;
//		throw std::runtime_error("[VoxelDownSample] voxel_size <= 0.");
	}

	const Eigen::Vector3d voxelSize = Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
//	const auto voxelBounds = computeVoxelBounds(cloud, voxelSize);
//	const Eigen::Vector3d voxelMinBound = voxelBounds.first;
//	const Eigen::Vector3d voxelMaxBound = voxelBounds.second;
//	if (voxel_size * std::numeric_limits<int>::max() < (voxelMaxBound - voxelMinBound).maxCoeff()) {
//		throw std::runtime_error("[VoxelDownSample] voxel_size is too small.");
//	}
	std::unordered_map<Eigen::Vector3i, AccumulatedPoint, EigenVec3iHash> voxelindex_to_accpoint;

	const bool has_normals = cloud.HasNormals();
	const bool has_colors = cloud.HasColors();
	output->points_.reserve(cloud.points_.size());
	if (has_colors) {
		output->colors_.reserve(cloud.points_.size());
	}
	if (has_normals) {
		output->normals_.reserve(cloud.points_.size());
	}

	voxelindex_to_accpoint.reserve(cloud.points_.size());
	for (size_t i = 0; i < cloud.points_.size(); i++) {
		if (croppingVolume.isWithinVolume(cloud.points_[i])) {
			const Eigen::Vector3i voxelIdx = getVoxelIdx(cloud.points_[i], voxelSize);
			voxelindex_to_accpoint[voxelIdx].AddPoint(cloud, i);
		} else {
			output->points_.emplace_back(std::move(cloud.points_[i]));
			if (has_normals) {
				output->normals_.emplace_back(std::move(cloud.normals_[i]));
			}
			if (has_colors) {
				output->colors_.emplace_back(std::move(cloud.colors_[i]));
			}
		}
	}

	for (auto accpoint : voxelindex_to_accpoint) {
		output->points_.emplace_back(std::move(accpoint.second.GetAveragePoint()));
		if (has_normals) {
			output->normals_.emplace_back(std::move(accpoint.second.GetAverageNormal().normalized()));
		}
		if (has_colors) {
			output->colors_.emplace_back(std::move(accpoint.second.GetAverageColor()));
		}
	}

	return output;
}

std::pair<std::vector<double>, std::vector<size_t>> computePointCloudDistance(
		const open3d::geometry::PointCloud &reference, const open3d::geometry::PointCloud &cloud,
		const std::vector<size_t> &idsInReference) {
	std::vector<double> distances(idsInReference.size());
	std::vector<int> indices(idsInReference.size());
	open3d::geometry::KDTreeFlann kdtree;
	kdtree.SetGeometry(cloud); // fast cca 1 ms

#pragma omp parallel for schedule(static)
	for (size_t i = 0; i < idsInReference.size(); i++) {
		const size_t idx = idsInReference[i];
		const int knn = 1;
		std::vector<int> ids(knn);
		std::vector<double> dists(knn);
//			if (kdtree.SearchHybrid(reference.points_[idx], 2.0, knn, ids, dists) != 0) {
		if (kdtree.SearchKNN(reference.points_[idx], knn, ids, dists) != 0) {
			distances[i] = std::sqrt(dists[0]);
			indices[i] = idx;
		} else {
			distances[i] = -1.0;
			indices[i] = -1;
//				std::cout << "could not find a nearest neighbour \n";
		}
	} // end for

	// remove distances/ids for which no neighbor was found
	std::vector<double> distsRet;
	distsRet.reserve(distances.size());
	std::copy_if(distances.begin(), distances.end(), std::back_inserter(distsRet), [](double x) {
		return x >= 0;
	});
	std::vector<size_t> idsRet;
	idsRet.reserve(indices.size());
	std::copy_if(indices.begin(), indices.end(), std::back_inserter(idsRet), [](int x) {
		return x >= 0;
	});
	return {distsRet,idsRet};
}

void removeByIds(const std::vector<size_t> &ids, open3d::geometry::PointCloud *cloud) {
	if (ids.empty()) {
		return;
	}
	const bool isInvertSelection = true;
	auto trimmedCloud = cloud->SelectByIndex(ids, isInvertSelection);
	*cloud = std::move(*trimmedCloud);
}

std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud &scan,
		const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &sensorPosition,
		const SpaceCarvingParameters &param) {
	std::vector<size_t> subsetIdxs(cloud.points_.size(), 0);
	std::iota(subsetIdxs.begin(), subsetIdxs.end(), 0);
	return getIdxsOfCarvedPoints(scan, cloud, sensorPosition, subsetIdxs, param);
}
std::vector<size_t> getIdxsOfCarvedPoints(const open3d::geometry::PointCloud &scan,
		const open3d::geometry::PointCloud &cloud, const Eigen::Vector3d &sensorPosition,
		const std::vector<size_t> &cloudIdxsSubset, const SpaceCarvingParameters &param) {

	const double stepSize = param.voxelSize_;
	const std::string layer = "layer";
  VoxelMap voxelMap(Eigen::Vector3d::Constant(param.voxelSize_));
	voxelMap.insertCloud(layer, cloud, cloudIdxsSubset);
	std::unordered_set<size_t> setOfIdsToRemove;
	setOfIdsToRemove.reserve(scan.points_.size());
#pragma omp parallel for schedule(static)
	for (size_t i = 0; i < scan.points_.size(); ++i) {
		const Eigen::Vector3d &p = scan.points_[i];
		const double length = (p - sensorPosition).norm();
		const Eigen::Vector3d direction = (p - sensorPosition) / length;
		double distance = 0.0;
		const double maximalPathTraveled = std::max(param.voxelSize_,
				std::min(length - param.truncationDistance_, param.maxRaytracingLength_));
		while (distance < maximalPathTraveled) {
			const Eigen::Vector3d currentPosition = distance * direction + sensorPosition;
			auto ids = voxelMap.getIndicesInVoxel(layer, currentPosition);
			for (const auto id : ids) {
				bool isRemoveId = true;
				if (cloud.HasNormals()) {
					const auto n = cloud.normals_[id].normalized();
					isRemoveId = std::abs(direction.dot(n)) > param.minDotProductWithNormal_;
				}
				if (isRemoveId) {
#pragma omp critical
					setOfIdsToRemove.insert(id);
				}
			}
			distance += stepSize;
		}
	}
	std::vector<size_t> vecOfIdsToRemove;
	vecOfIdsToRemove.insert(vecOfIdsToRemove.end(), setOfIdsToRemove.begin(), setOfIdsToRemove.end());
	return vecOfIdsToRemove;
}

std::shared_ptr<open3d::geometry::PointCloud> transform(const Eigen::Matrix4d &T,
		const open3d::geometry::PointCloud &cloud) {

	auto out = std::make_shared<open3d::geometry::PointCloud>();
	const auto isIdentity = (T - Eigen::Matrix4d::Identity()).array().abs().maxCoeff() < 1e-4;
	if (isIdentity) {
		*out = cloud;
	}

	out->points_.reserve(cloud.points_.size());
	out->colors_ = cloud.colors_;
	if (cloud.HasNormals()) {
		out->normals_.reserve(cloud.points_.size());
	}
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
		const auto &p = cloud.points_[i];
		Eigen::Vector4d new_point = T * Eigen::Vector4d(p(0), p(1), p(2), 1.0);
		Eigen::Vector3d xyz = new_point.head<3>() / new_point(3);
		out->points_.emplace_back(std::move(xyz));
		if (cloud.HasNormals()) {
			const auto &n = cloud.normals_[i];
			Eigen::Vector4d new_normal = T * Eigen::Vector4d(n(0), n(1), n(2), 0.0);
			out->normals_.emplace_back(std::move(new_normal.head<3>()));
		}
	}
	return out;

}

void computeIndicesOfOverlappingPoints(const open3d::geometry::PointCloud &source,
		const open3d::geometry::PointCloud &target, const Transform &sourceToTarget, double voxelSize,
		size_t minNumPointsPerVoxel, std::vector<size_t> *idxsSource, std::vector<size_t> *idxsTarget) {
	assert_ge<size_t>(minNumPointsPerVoxel, 1);
	const std::string targetLayer = "target";
	const std::string sourceLayer = "source";
	VoxelMap voxelMap(Eigen::Vector3d::Constant(voxelSize));
	voxelMap.insertCloud(targetLayer, target);
	auto sourceTransformed = source;
	sourceTransformed.Transform(sourceToTarget.matrix());
	voxelMap.insertCloud(sourceLayer, sourceTransformed);
	idxsSource->clear();
	idxsSource->reserve(source.points_.size());
	idxsTarget->clear();
	idxsTarget->reserve(target.points_.size());
	const auto &voxels = voxelMap.voxels_;
	for (auto it = voxels.cbegin(); it != voxels.cend(); ++it) {
		const auto voxelKey = it->first;
		const auto sourceIdxs = voxelMap.getIndicesInVoxel(sourceLayer, voxelKey);
		const auto targetIdxs = voxelMap.getIndicesInVoxel(targetLayer, voxelKey);
		if (sourceIdxs.size() >= minNumPointsPerVoxel && targetIdxs.size() >= minNumPointsPerVoxel) {
			idxsTarget->insert(idxsTarget->end(), targetIdxs.begin(), targetIdxs.end());
			idxsSource->insert(idxsSource->end(), sourceIdxs.begin(), sourceIdxs.end());
		}
	}
}

Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs) {

	assert_gt<size_t>(idxs.size(), 0,"you're trying to compute center of a empty pointcloud");
	Eigen::Vector3d center(0.0, 0.0, 0.0);
	for (const auto idx : idxs) {
		center += cloud.points_.at(idx);
	}
	return center / static_cast<double>(idxs.size());

}

double getMapVoxelSize(const MapBuilderParameters &p, double valueIfZero) {
	return std::abs(p.mapVoxelSize_) <= 1e-3 ? valueIfZero : p.mapVoxelSize_;
}




std::vector<Eigen::Vector3i> getKeysOfCarvedPoints(const open3d::geometry::PointCloud &scan,
		const VoxelizedPointCloud &cloud, const Eigen::Vector3d &sensorPosition, const SpaceCarvingParameters &param) {

	const double stepSize = param.voxelSize_;
	std::unordered_set<Eigen::Vector3i,EigenVec3iHash> setOfIdsToRemove;
	setOfIdsToRemove.reserve(scan.points_.size());
#pragma omp parallel for schedule(static)
	for (size_t i = 0; i < scan.points_.size(); ++i) {
		const Eigen::Vector3d &p = scan.points_[i];
		const double length = (p - sensorPosition).norm();
		const Eigen::Vector3d direction = (p - sensorPosition) / length;
		double distance = 0.0;
		const double maximalPathTraveled = std::max(param.voxelSize_,
				std::min(length - param.truncationDistance_, param.maxRaytracingLength_));
		while (distance < maximalPathTraveled) {
			const Eigen::Vector3d currentPosition = distance * direction + sensorPosition;
			const Eigen::Vector3i key = cloud.getKey(currentPosition);
			//todo also check the dot product
			if (cloud.hasVoxelWithKey(key)){
#pragma omp critical
					setOfIdsToRemove.insert(key);
			}
			distance += stepSize;
		}
	}
	std::vector<Eigen::Vector3i> vecOfIdsToRemove;
	vecOfIdsToRemove.insert(vecOfIdsToRemove.end(), setOfIdsToRemove.begin(), setOfIdsToRemove.end());
	return vecOfIdsToRemove;
}

PointCloud getPointCloudWithinCroppingVolume(const CroppingVolume &croppingVolume,
		const VoxelizedPointCloud &voxels, bool isIgnoreColors) {

	if (voxels.empty()) {
		return PointCloud();
	}
	PointCloud ret;
	ret.points_.reserve(voxels.size());
	if (!isIgnoreColors && voxels.hasColors()) {
		ret.colors_.reserve(voxels.size());
	}
	if (voxels.hasNormals()) {
		ret.normals_.reserve(voxels.size());
	}

	for (const auto &voxel : voxels.voxels_) {
		if (croppingVolume.isWithinVolume(voxel.second.getAggregatedPosition())) {
			ret.points_.push_back(voxel.second.getAggregatedPosition());
			if (!isIgnoreColors && voxels.hasColors()) {
				ret.colors_.push_back(voxel.second.getAggregatedColor());
			}
			if (voxels.hasNormals()) {
				ret.normals_.push_back(voxel.second.getAggregatedNormal());
			}
		}
	}
	return ret;
}

Eigen::Vector3d computeCenter(const VoxelizedPointCloud &voxels) {
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	int n = 0;
	for (const auto &voxel : voxels.voxels_) {
		if (voxel.second.numAggregatedPoints_ > 0) {
			center += voxel.second.getAggregatedPosition();
			++n;
		}
	}
	return center / static_cast<double>(n+1e-6);
}

std::shared_ptr<PointCloud> removePointsWithNonFiniteValues(const PointCloud &cloud){
	std::shared_ptr<CroppingVolume::PointCloud> filtered(new PointCloud());
	const int nPoints = cloud.points_.size();
	filtered->points_.reserve(nPoints);
	if (cloud.HasColors()) {
		filtered->colors_.reserve(nPoints);
	}
	if (cloud.HasNormals()) {
		filtered->normals_.reserve(nPoints);
	}

	for (size_t i = 0; i < nPoints; ++i) {
		if (cloud.points_[i].array().isFinite().all()) {
			filtered->points_.push_back(cloud.points_[i]);
			if (cloud.HasColors()) {
				filtered->colors_.push_back(cloud.colors_[i]);
			}
			if (cloud.HasNormals()) {
				filtered->normals_.push_back(cloud.normals_[i]);
			}
		} // end if
	}
//	std::cout << "Filter: " << filtered->points_.size() << "/" << nPoints << std::endl;
	return filtered;
}

} /* namespace o3d_slam */

