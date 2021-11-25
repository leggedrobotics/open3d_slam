/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/typedefs.hpp"
#include "m545_volumetric_mapping/math.hpp"

#include <open3d/io/PointCloudIO.h>
#include <open3d/pipelines/registration/Registration.h>

#include <algorithm>
#include <numeric>
#include <utility>
#include <set>

namespace m545_mapping {

SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
	createNewSubmap(mapToRangeSensor_);
}

void SubmapCollection::setMapToRangeSensor(const Transform &T) {
	mapToRangeSensor_ = T;
}

bool SubmapCollection::isEmpty() const {
	return submaps_.empty();
}

const SubmapCollection::Submaps& SubmapCollection::getSubmaps() const {
	return submaps_;
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popFinishedSubmapIds() {
	return finishedSubmapsIdxs_.popAllElements();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popLoopClosureCandidates() {
	return loopClosureCandidatesIdxs_.popAllElements();
}

size_t SubmapCollection::numFinishedSubmaps() const{
	return finishedSubmapsIdxs_.size();
}

size_t SubmapCollection::numLoopClosureCandidates() const{
	return loopClosureCandidatesIdxs_.size();
}

size_t SubmapCollection::getTotalNumPoints() const {
	const int nSubmaps = submaps_.size();
	return std::accumulate(submaps_.begin(), submaps_.end(), 0, [](size_t sum, const Submap &s) {
		return sum + s.getMap().points_.size();
	});
}

void SubmapCollection::updateActiveSubmap(const Transform &mapToRangeSensor) {
	if (numScansMergedInActiveSubmap_ < params_.submaps_.minNumRangeData_) {
		return;
	}
	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	const auto &closestSubmap = submaps_.at(closestMapIdx);
	const auto &activeSubmap = submaps_.at(activeSubmapIdx_);
	const Eigen::Vector3d closestSubmapPosition = closestSubmap.getMapToSubmapCenter();
	const bool isAnotherSubmapWithinRange = (mapToRangeSensor_.translation() - closestSubmapPosition).norm()
			< params_.submaps_.radius_;
	if (isAnotherSubmapWithinRange) {
		if (closestMapIdx == activeSubmapIdx_) {
			return;
		}
		if (adjacencyMatrix_.isAdjacent(closestSubmap.getId(), activeSubmap.getId())) {
			activeSubmapIdx_ = closestMapIdx;
		} else {
			const bool isTraveledSufficientDistance = (mapToRangeSensor_.translation()
					- activeSubmap.getMapToSubmapCenter()).norm() > params_.submaps_.radius_;
			if (isTraveledSufficientDistance) {
				createNewSubmap(mapToRangeSensor_);
			}
		}
	} else {
		createNewSubmap(mapToRangeSensor_);
	}
}

void SubmapCollection::createNewSubmap(const Transform &mapToSubmap) {
	const size_t submapId = submapId_++;
	const size_t submapParentId = std::max<int>(0, static_cast<int>(submapId) - 1);
	Submap newSubmap(submapId, submapParentId);
	newSubmap.setMapToSubmapOrigin(mapToSubmap);
	newSubmap.setParameters(params_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
	numScansMergedInActiveSubmap_ = 0;
	std::cout << "Created submap: " << activeSubmapIdx_ << " with parent " << submapParentId << std::endl;
	if (submaps_.size() > 1) {
		const auto c = buildOdometryConstraint(activeSubmapIdx_ - 1, activeSubmapIdx_);
		odometryConstraints_.push_back(c);
	}
}

size_t SubmapCollection::findClosestSubmap(const Transform &mapToRangeSensor) const {

	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	auto lessThan = [this, &mapToRangeSensor](size_t idxa, size_t idxb) {
		const auto p0 = mapToRangeSensor.translation();
		const auto pa = submaps_.at(idxa).getMapToSubmapCenter();
		const auto pb = submaps_.at(idxb).getMapToSubmapCenter();
		return (p0 - pa).norm() < (p0 - pb).norm();
	};
	return *(std::min_element(idxs.begin(), idxs.end(), lessThan));
}

const Submap& SubmapCollection::getActiveSubmap() const {
	return submaps_.at(activeSubmapIdx_);
}

bool SubmapCollection::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &timestamp) {

	mapToRangeSensor_ = mapToRangeSensor;
	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		++numScansMergedInActiveSubmap_;
		return true;
	}
	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap(mapToRangeSensor);
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if (isActiveSubmapChanged) {
		std::lock_guard<std::mutex> lck(featureComputationMutex_);
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		submaps_.at(prevActiveSubmapIdx).computeSubmapCenter();
		std::cout << "Active submap changed from " << prevActiveSubmapIdx << " to " << activeSubmapIdx_ << "\n";
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
		TimestampedSubmapId timestampedId { prevActiveSubmapIdx, timestamp };
		finishedSubmapsIdxs_.push(timestampedId);
		numScansMergedInActiveSubmap_ = 0;
		const auto id1 = submaps_.at(prevActiveSubmapIdx).getId();
		const auto id2 = submaps_.at(activeSubmapIdx_).getId();
		adjacencyMatrix_.addEdge(id1, id2);
		std::cout << "Adding edge between " << id1 << " and " << id2 << std::endl;
	}
	submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
	// todo possibly here we need to update the active map
	// that update needs to be thread safe

	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
	placeRecognition_.setParameters(p);
}

void SubmapCollection::computeFeatures(const TimestampedSubmapIds &finishedSubmapIds) {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	isComputingFeatures_ = true;
	Timer t("feature computation");
	for (const auto &id : finishedSubmapIds) {
		std::cout << "computing features for submap: " << id.submapId_ << std::endl;
		submaps_.at(id.submapId_).computeFeatures();
		loopClosureCandidatesIdxs_.push(id);
	}
	// if the features in those maps are computed, they can be considered for a loop closure
	const double d = informationMatrixMaxCorrespondenceDistance(params_.mapBuilder_.mapVoxelSize_);
	computeInformationMatrixOdometryConstraints(*this, d, &odometryConstraints_);
	isComputingFeatures_ = false;
}

bool SubmapCollection::isComputingFeatures() const {
	return isComputingFeatures_;
}

const Constraints& SubmapCollection::getOdometryConstraints() const {
	return odometryConstraints_;
}
const Constraints& SubmapCollection::getLoopClosureConstraints() const {
	return loopClosureConstraints_;
}
void SubmapCollection::addLoopClosureConstraints(const Constraints &lccs) {
	loopClosureConstraints_.insert(loopClosureConstraints_.end(), lccs.begin(), lccs.end());
}

Constraints SubmapCollection::buildLoopClosureConstraints(const TimestampedSubmapIds &loopClosureCandidatesIdxs) const {
	Constraints retVal;
	for (const auto &id : loopClosureCandidatesIdxs) {
		const auto constraints = placeRecognition_.buildLoopClosureConstraints(mapToRangeSensor_, *this,
				adjacencyMatrix_, id.submapId_, activeSubmapIdx_, id.time_);
		if (!constraints.empty()) {
			for (int i = 0; i < constraints.size(); ++i) {
				retVal.push_back(constraints.at(i));
			}
			std::cout << " building loop closure constraints for submap: " << id.submapId_ << " resulted in: "
					<< constraints.size() << " new constraints \n";
		}
	}
	return retVal;
}

Constraint SubmapCollection::buildOdometryConstraint(size_t sourceSubmapIdx, size_t targetSubmapIdx) const {
	Constraint c;
	c.sourceSubmapIdx_ = sourceSubmapIdx;
	c.targetSubmapIdx_ = targetSubmapIdx;
	const Transform &mapToSource = submaps_.at(sourceSubmapIdx).getMapToSubmapOrigin();
	const Transform &mapToTarget = submaps_.at(targetSubmapIdx).getMapToSubmapOrigin();
	c.sourceToTarget_ = mapToSource.inverse() * mapToTarget;
//	std::cout << "odom constraints: \n";
//	std::cout << " source: " << asString(mapToSource) << std::endl;
//	std::cout << " target: " << asString(mapToTarget) << std::endl;
//	std::cout << " source to target: " << asString(c.sourceToTarget_) << std::endl;
//	std::cout << " source transformed into target: \n";
//	std::cout << asString(mapToSource * c.sourceToTarget_) << "\n \n";
	return std::move(c);
}

void SubmapCollection::dumpToFile(const std::string &folderPath, const std::string &filename) const {
	for (size_t i = 0; i < submaps_.size(); ++i) {
		auto copy = submaps_.at(i).getMap();
		const std::string fullPath = folderPath + "/" + filename + "_" + std::to_string(i) + ".pcd";
		open3d::io::WritePointCloudToPCD(fullPath, copy, open3d::io::WritePointCloudOption());
	}
}

void SubmapCollection::transform(const OptimizedTransforms &transformIncrements) {
	const size_t nTransforms = transformIncrements.size();
	std::vector<size_t> optimizedIdxs;
	for (size_t i = 0; i < nTransforms; ++i) {
		const auto &update = transformIncrements.at(i);
		submaps_.at(update.submapId_).transform(update.dT_);
		optimizedIdxs.push_back(update.submapId_);
	}
	std::sort(optimizedIdxs.begin(), optimizedIdxs.end());
	std::vector<size_t> allIdxs = getAllSubmapIdxs();

	std::vector<size_t> submapIdxsToUpdate;
	std::set_difference(allIdxs.begin(), allIdxs.end(), optimizedIdxs.begin(), optimizedIdxs.end(),
			std::inserter(submapIdxsToUpdate, submapIdxsToUpdate.begin()));

	for (auto idx : submapIdxsToUpdate) {
		//look at the node parent
		// if the parent is not in the list of nodes to update
		// then you are done, take the transform of the parent node
		// otherwise, set the current node to be the parent node
		size_t currentNode;
		currentNode = idx;
		while (true) {
			currentNode = submaps_.at(currentNode).getParentId();
			if (std::find(submapIdxsToUpdate.begin(), submapIdxsToUpdate.end(), currentNode)
					== submapIdxsToUpdate.end()) {
				/* parent is in the pose graph */
				const auto &update = transformIncrements.at(currentNode);
				submaps_.at(update.submapId_).transform(update.dT_);
				break;
			}
			if (currentNode == submaps_.at(currentNode).getParentId()) {
				throw std::runtime_error("Stuck in a loop, this should not happen");
			}
		}
	}

}

std::vector<size_t> SubmapCollection::getAllSubmapIdxs() const {
	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	return idxs;
}

////////////////////////////////////////////////////////////////////
/// NON MEMBER ////////
////////////////////////////////////////////////////////////////////
void computeInformationMatrixOdometryConstraints(const SubmapCollection &submaps, double maxCorrespondenceDistance,
		Constraints *constraints) {
	const size_t nConstraints = constraints->size();
	for (size_t i = 0; i < nConstraints; ++i) {
		Constraint &c = constraints->at(i);
		if (!c.isInformationMatrixValid_ && c.isOdometryConstraint_) {
			const size_t sourceIdx = c.sourceSubmapIdx_;
			const size_t targetIdx = c.targetSubmapIdx_;
			const bool isSourceMapFinished = submaps.getSubmaps().at(sourceIdx).getFeaturePtr() != nullptr;
			const bool istargetMapFinished = submaps.getSubmaps().at(targetIdx).getFeaturePtr() != nullptr;
			if (isSourceMapFinished && istargetMapFinished) {
				const auto source = submaps.getSubmaps().at(sourceIdx).getMap();
				const auto target = submaps.getSubmaps().at(targetIdx).getMap();
				std::cout << "computing the information matrix for the edge between submaps: " << sourceIdx << " and "
						<< targetIdx << "\n";
				std::shared_ptr<open3d::geometry::PointCloud> sourceOverlap, targetOverlap;
				{
					Timer("information_matrix");
					// at this voxel size about half a points are dropped
					// the compute intersection function is quite fast less than 0.1 msec
					const double voxelSize = 0.25;
					const size_t minNumPointsVoxel = 3;
					std::vector<size_t> idxsSource, idxsTarget;
					computeIndicesOfOverlappingPoints(source, target, Transform::Identity(), voxelSize,
							minNumPointsVoxel, &idxsSource, &idxsTarget);
//					std::cout << "in total there is " << idxsSource.size() << "/"<< source.points_.size() << " points in source and "
//							<< idxsTarget.size() << "/"<<target.points_.size() << " points in target that overlap \n";
					sourceOverlap = source.SelectByIndex(idxsSource);
					targetOverlap = target.SelectByIndex(idxsTarget);
					c.informationMatrix_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
							source, target, maxCorrespondenceDistance, Eigen::Matrix4d::Identity());
//					std::cout << "information mat: \n" << c.informationMatrix_  << "\n";
					c.isInformationMatrixValid_ = true;
				}
			}
		}
	}
}

} // namespace m545_mapping
