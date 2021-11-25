/*
 * helpers.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/output.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Voxel.hpp"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
#include "open3d/geometry/KDTreeFlann.h"

#ifdef M545_VOLUMETRIC_MAPPING_OPENMP_FOUND
#include <omp.h>
#endif

namespace m545_mapping {

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
		if (cloud.HasColors()) {
			color_ += cloud.colors_[index];
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
		return color_ / double(num_of_points_);
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

double informationMatrixMaxCorrespondenceDistance(double mappingVoxelSize){
	return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (2.0*mappingVoxelSize);
}

double icpMaxCorrespondenceDistance(double mappingVoxelSize){
	return isClose(mappingVoxelSize, 0.0, 1e-3) ? 0.05 : (2.0*mappingVoxelSize);
}

void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl) {
	auto croppedCloud = pcl->Crop(bbox);
	*pcl = std::move(*croppedCloud);
}

std::string asString(const Transform &T) {
	const double kRadToDeg = 180.0 / M_PI;
	const auto &t = T.translation();
	const auto &q = Eigen::Quaterniond(T.rotation());
	const std::string trans = string_format("t:[%f, %f, %f]", t.x(), t.y(), t.z());
	const std::string rot = string_format("q:[%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());
	const auto rpy = toRPY(q) * kRadToDeg;
	const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]", rpy.x(), rpy.y(), rpy.z());
	return trans + " ; " + rot + " ; " + rpyString;

}

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl) {
	open3d::geometry::KDTreeSearchParamKNN param(numNearestNeighbours);
	pcl->EstimateNormals(param);
}

std::shared_ptr<registration::TransformationEstimation> icpObjectiveFactory(const m545_mapping::IcpObjective &obj) {

	switch (obj) {
	case m545_mapping::IcpObjective::PointToPoint: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPoint>(false);
		return obj;
	}

	case m545_mapping::IcpObjective::PointToPlane: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPlane>();
		return obj;
	}

	default:
		throw std::runtime_error("Unknown icp objective");
	}

}

open3d::geometry::AxisAlignedBoundingBox boundingBoxAroundPosition(const Eigen::Vector3d &low,
		const Eigen::Vector3d &high, const Eigen::Vector3d &origin /*= Eigen::Vector3d::Zero()*/) {
	open3d::geometry::AxisAlignedBoundingBox bbox;
	bbox.min_bound_ = origin + low;
	bbox.max_bound_ = origin + high;
	return bbox;
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

bool isInside(const open3d::geometry::AxisAlignedBoundingBox &bbox, const Eigen::Vector3d &p) {
	return p.x() <= bbox.max_bound_.x() && p.y() <= bbox.max_bound_.y() && p.z() <= bbox.max_bound_.z()
			&& p.x() >= bbox.min_bound_.x() && p.y() >= bbox.min_bound_.y() && p.z() >= bbox.min_bound_.z();
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
	const auto voxelBounds = computeVoxelBounds(cloud, voxelSize);
	const Eigen::Vector3d voxelMinBound = voxelBounds.first;
	const Eigen::Vector3d voxelMaxBound = voxelBounds.second;
	if (voxel_size * std::numeric_limits<int>::max() < (voxelMaxBound - voxelMinBound).maxCoeff()) {
		throw std::runtime_error("[VoxelDownSample] voxel_size is too small.");
	}
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
			output->normals_.emplace_back(std::move(accpoint.second.GetAverageNormal()));
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
	auto trimmedCloud = cloud->SelectByIndex(ids, true);
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
	VoxelMap voxelMap(Eigen::Vector3d::Constant(param.voxelSize_));
	voxelMap.buildFromCloud(cloud, cloudIdxsSubset);
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
			auto ids = voxelMap.getIndicesInVoxel(currentPosition);
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

//			if (!ids.empty()) {
//#pragma omp critical
//					setOfIdsToRemove.insert(ids.begin(), ids.end());
//			}
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
		size_t minNumPointsPerVoxel,
		std::vector<size_t> *idxsSource, std::vector<size_t> *idxsTarget) {
	assert_ge<size_t>(minNumPointsPerVoxel,1);
	VoxelMap targetMap(Eigen::Vector3d::Constant(voxelSize));
	targetMap.buildFromCloud(target);
	idxsSource->clear();
	idxsSource->reserve(source.points_.size());
	idxsTarget->clear();
	idxsTarget->reserve(target.points_.size());
	std::set<size_t> setTargetIdxs;
	for (size_t i = 0; i < source.points_.size(); ++i) {
		const auto p = sourceToTarget * source.points_.at(i);
		const auto targetIdxsInVoxel = targetMap.getIndicesInVoxel(p);
		if (targetIdxsInVoxel.size() >= minNumPointsPerVoxel) {
			setTargetIdxs.insert(targetIdxsInVoxel.begin(), targetIdxsInVoxel.end());
			idxsSource->push_back(i);
		}
	}
	idxsTarget->insert(idxsTarget->end(), setTargetIdxs.begin(), setTargetIdxs.end());
}

Eigen::Vector3d computeCenter(const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idxs){

	assert_gt<size_t>(idxs.size(),0);
	Eigen::Vector3d center(0.0,0.0,0.0);
	for(const auto idx : idxs){
		center += cloud.points_.at(idx);
	}
	return center / static_cast<double>(idxs.size());

}

//void removeInconsistencies(const PointCloud &scan, double icpRMSE, PointCloud *map) const {
//
//	if (map->IsEmpty()) {
//		return;
//	}
//
////
////	open3d::geometry::OrientedBoundingBox bbox;
////	const auto extent = Eigen::Vector3d(30.0, 30.0, 100.0);
////	bbox.R_ = mapToRangeSensor_.rotation();
////	bbox.center_ = mapToRangeSensor_.translation() + bbox.R_.inverse() * Eigen::Vector3d(10.0, -15.0, 0.0);
////	bbox.extent_ = extent;
//	Timer timer;
//	const Eigen::Vector3d voxelSize = Eigen::Vector3d(0.5, 0.5, 0.5);
//	std::unordered_map<Eigen::Vector3i, Voxel, EigenVec3iHash> scanToVoxel;
//	scanToVoxel.reserve(scan.points_.size());
//	for (size_t i = 0; i < scan.points_.size(); i++) {
//		const Eigen::Vector3i voxelIdx = getVoxelIdx(scan.points_[i], voxelSize);
//		scanToVoxel[voxelIdx].idxs_.emplace_back(i);
//	}
//
//	const int minNumPointsPerVoxel = 4;
//	std::vector<size_t> idsCoveredByScan;
//	idsCoveredByScan.reserve(map->points_.size());
//#pragma omp parallel for schedule(static)
//	for (size_t i = 0; i < map->points_.size(); i++) {
//		const Eigen::Vector3i voxelIdx = getVoxelIdx(map->points_[i], voxelSize);
//		const auto search = scanToVoxel.find(voxelIdx);
//		if (search != scanToVoxel.end()) {
//			if (search->second.idxs_.size() >= minNumPointsPerVoxel) {
//#pragma omp critical
//				{
//					idsCoveredByScan.emplace_back(i);
//				}
//			}
//		}
//	}
////		std::cout << "finding scan overlap: " << timer.elapsedMsec() << " msec \n";
////	std::cout << "num points overlapping with scan: " << idsCoveredByScan.size() << std::endl;
//
////	mapRef_ = *(map->SelectByIndex(idsCoveredByScan));
////	scanRef_ = scan;
//
//	std::vector<std::pair<double, size_t>> distsAndIds;
//	{
////		Timer timer("bulk_computation");
//		mapRef_ = *map;
//		auto ret = computePointCloudDistance(*map, scan, idsCoveredByScan);
//		distsAndIds.reserve(ret.first.size());
//		for (int i = 0; i < (int) ret.first.size(); ++i) {
//			distsAndIds.emplace_back( std::make_pair(ret.first[i], ret.second[i] ));
//		}
//	}
//	{
////		Timer timer("sort");
//		std::sort(distsAndIds.begin(), distsAndIds.end(),
//				[](const std::pair<double, size_t> &a, const std::pair<double, size_t> &b) {
//					return a.first > b.first;
//				});
//	}
////	std::cout << "max element: " << distsAndIds.front().first << std::endl;
////	std::cout << "min element: " << distsAndIds.back().first << std::endl;
////	std::cout << "icp rmse: " << icpRMSE << std::endl;
//	std::vector<size_t> idsToRemove;
//	idsToRemove.reserve(distsAndIds.size());
//	for (size_t i = 0; i < distsAndIds.size() && i < 100; ++i) {
//		if (distsAndIds[i].first > icpRMSE) {
//			idsToRemove.emplace_back(distsAndIds[i].second);
//		}
//	}
//
////	std::cout << "Would remove: " << idsToRemove.size() << "\n";
////	toRemove_ = *(map->SelectByIndex(idsToRemove));
//	removeByIds(idsToRemove, map);
//}

} /* namespace m545_mapping */

