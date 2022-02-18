/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/magic.hpp"
#include <open3d/io/PointCloudIO.h>
#include <open3d/pipelines/registration/Registration.h>

#include <algorithm>
#include <numeric>
#include <utility>
#include <set>
#include <thread>

namespace o3d_slam {

SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
	createNewSubmap(mapToRangeSensor_);
	overlapScansBuffer_.set_size_limit(5);
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

Submap* SubmapCollection::getSubmapPtr(size_t idx) {
	return &(submaps_.at(idx));
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popFinishedSubmapIds() {
	return finishedSubmapsIdxs_.popAllElements();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popLoopClosureCandidates() {
	return loopClosureCandidatesIdxs_.popAllElements();
}

size_t SubmapCollection::numFinishedSubmaps() const {
	return finishedSubmapsIdxs_.size();
}

size_t SubmapCollection::numLoopClosureCandidates() const {
	return loopClosureCandidatesIdxs_.size();
}

size_t SubmapCollection::getTotalNumPoints() const {
	const int nSubmaps = submaps_.size();
	return std::accumulate(submaps_.begin(), submaps_.end(), 0, [](size_t sum, const Submap &s) {
		return sum + s.getMap().points_.size();
	});
}

void SubmapCollection::updateAdjacencyMatrix(const Constraints &loopClosureConstraints){
	for (const auto &c : loopClosureConstraints){
		adjacencyMatrix_.addEdge(c.sourceSubmapIdx_, c.targetSubmapIdx_);
	}
}

void SubmapCollection::addScanToBuffer(const PointCloud &scan, const Transform &mapToRangeSensor,
		const Time &timestamp) {
	overlapScansBuffer_.push(ScanTimeTransform { scan, timestamp, mapToRangeSensor });
}

void SubmapCollection::insertBufferedScans(Submap *submap) {
	while (!overlapScansBuffer_.empty()) {
		auto scan = overlapScansBuffer_.pop();
		submap->insertScan(scan.cloud_, scan.cloud_, scan.mapToRangeSensor_, scan.timestamp_, false);
	}
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
	const size_t submapParentId = activeSubmapIdx_;
	Submap newSubmap(submapId, submapParentId);
	newSubmap.setMapToSubmapOrigin(mapToSubmap);
	newSubmap.setParameters(params_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
	numScansMergedInActiveSubmap_ = 0;
	std::cout << "Created submap: " << activeSubmapIdx_ << " with parent " << submapParentId << std::endl;
	std::cout << "Submap " << activeSubmapIdx_ << " pose: " << asString(newSubmap.getMapToSubmapOrigin())
			<< std::endl;
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
	addScanToBuffer(preProcessedScan, mapToRangeSensor, timestamp);
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
		insertBufferedScans(&submaps_.at(activeSubmapIdx_));
	} else {
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
	}
	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
	placeRecognition_.setParameters(p);
	assert_gt<size_t>(params_.numScansOverlap_, 0, "Num scan overlap has to be > 0");
	overlapScansBuffer_.set_size_limit(params_.numScansOverlap_);
}

void SubmapCollection::computeFeatures(const TimestampedSubmapIds &finishedSubmapIds) {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	isComputingFeatures_ = true;
	Timer timer("submap_finishing_total");

	auto featureComputation = [&]() {
		Timer t("feature computation");
		for (const auto &id : finishedSubmapIds) {
			std::cout << "computing features for submap: " << id.submapId_ << std::endl;
			submaps_.at(id.submapId_).computeFeatures();
			loopClosureCandidatesIdxs_.push(id);
		}
	};

	std::thread t(featureComputation);

	{
		Timer t("odometry_constraint_computation");
		computeOdometryConstraints(*this, finishedSubmapIds, &odometryConstraints_);
	}

	t.join();
	isComputingFeatures_ = false;
}

bool SubmapCollection::isComputingFeatures() const {
	return isComputingFeatures_;
}

const Constraints& SubmapCollection::getOdometryConstraints() const {
	return odometryConstraints_;
}

Constraints SubmapCollection::buildLoopClosureConstraints(
		const TimestampedSubmapIds &loopClosureCandidatesIdxs) const {
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
	//by convention the source is map k and target k+1
	Constraint c;
	c.sourceSubmapIdx_ = sourceSubmapIdx;
	c.targetSubmapIdx_ = targetSubmapIdx;
	const Transform &mapToSource = submaps_.at(sourceSubmapIdx).getMapToSubmapOrigin();
	const Transform &mapToTarget = submaps_.at(targetSubmapIdx).getMapToSubmapOrigin();
	c.sourceToTarget_ = mapToSource.inverse() * mapToTarget;
	c.isOdometryConstraint_ = true;
	c.isInformationMatrixValid_ = false;
	std::cout << "added an odom constraint: \n";
	std::cout << " map " << sourceSubmapIdx << " to " << targetSubmapIdx << ": " << asString(c.sourceToTarget_)
			<< std::endl;

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
	std::cout << "Num transforms: " << transformIncrements.size() << std::endl;
	std::cout << "Num submaps: " << submaps_.size() << std::endl;
	std::cout << "Transforms of updated submaps:: \n";
	for (size_t i = 0; i < nTransforms; ++i) {
		const auto &update = transformIncrements.at(i);
		if (update.submapId_ < submaps_.size()) {
			submaps_.at(update.submapId_).transform(update.dT_);
			optimizedIdxs.push_back(update.submapId_);
			std::cout << "Submap " << update.submapId_ << " " << asString(update.dT_) << "\n";
		} else {
			std::cout << "tying to update submap: " << update.submapId_ << " but the there are only: "
					<< submaps_.size() << "submaps!!!! This should not happen! \n";
		}
	}
	std::sort(optimizedIdxs.begin(), optimizedIdxs.end());
	std::vector<size_t> allIdxs = getAllSubmapIdxs();

	std::vector<size_t> submapIdxsToUpdate;
	std::set_difference(allIdxs.begin(), allIdxs.end(), optimizedIdxs.begin(), optimizedIdxs.end(),
			std::inserter(submapIdxsToUpdate, submapIdxsToUpdate.begin()));
	std::cout << "\n num maps: " << submaps_.size() << "\n";
	std::cout << "num maps missing: " << submapIdxsToUpdate.size() << "\n";
	std::cout << " maps that are missing: \n";
	for (auto idx : submapIdxsToUpdate) {
		//look at the node parent
		// if the parent is not in the list of nodes to update
		// then you are done, take the transform of the parent node
		// otherwise, set the current node to be the parent node
		size_t currentNode;
		currentNode = idx;
		std::cout << currentNode << " with parent: ";
		while (true && !transformIncrements.empty()) {
			currentNode = submaps_.at(currentNode).getParentId();
			if (std::find(submapIdxsToUpdate.begin(), submapIdxsToUpdate.end(), currentNode)
					== submapIdxsToUpdate.end()) {
				/* parent is in the pose graph */
				const auto &update = transformIncrements.at(currentNode);
				submaps_.at(idx).transform(update.dT_);
				std::cout << update.submapId_ << "\n";
				break;
			}
			if (currentNode == submaps_.at(currentNode).getParentId()) {
				throw std::runtime_error("Stuck in a loop, this should not happen");
			}
		}
	}

	//need to flush the buffered scans
	overlapScansBuffer_.clear();

}

std::vector<size_t> SubmapCollection::getAllSubmapIdxs() const {
	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	return idxs;
}

const MapperParameters& SubmapCollection::getParameters() const {
	return params_;
}

void SubmapCollection::setFolderPath(const std::string &folderPath) {
	savingDataFolderPath_ = folderPath;
	placeRecognition_.setFolderPath(folderPath);
}

////////////////////////////////////////////////////////////////////
/// NON MEMBER ////////
////////////////////////////////////////////////////////////////////
namespace {
bool hasConstraint(size_t sourceIdx, size_t targetIdx, const Constraints &constraints) {
	for (const auto &c : constraints) {
		if (c.sourceSubmapIdx_ == sourceIdx && c.targetSubmapIdx_ == targetIdx) {
			return true;
		}
	}
	return false;
}
} // namespace

Constraint buildOdometryConstraint(size_t sourceIdx, size_t targetIdx, const SubmapCollection &submaps) {
	const double mapVoxelSize = getMapVoxelSize(submaps.getParameters().mapBuilder_,
			voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);
	Constraint c = buildConstraint(sourceIdx, targetIdx, submaps, true, 1.5 * mapVoxelSize,
			20.0 * mapVoxelSize,true);
	c.isOdometryConstraint_ = true;
	return c;
}

Constraint buildConstraint(size_t sourceIdx, size_t targetIdx, const SubmapCollection &submaps,
		bool isComputeOverlap, double icpMaxCorrespondenceDistance, double voxelSizeOverlapCompute,
		bool isEstimateInformationMatrix) {

	PointCloud source = submaps.getSubmaps().at(sourceIdx).getMap();
	PointCloud target = submaps.getSubmaps().at(targetIdx).getMap();
	const double mapVoxelSize = getMapVoxelSize(submaps.getParameters().mapBuilder_,
			voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);

	if (isComputeOverlap) {
		std::vector<size_t> sourceIdxs, targetIdxs;
		const size_t minNumPointsPerVoxel = 1;
		computeIndicesOfOverlappingPoints(source, target, Transform::Identity(), voxelSizeOverlapCompute,
				minNumPointsPerVoxel, &sourceIdxs, &targetIdxs);
		source = *source.SelectByIndex(sourceIdxs);
		target = *target.SelectByIndex(targetIdxs);
	}

	open3d::pipelines::registration::ICPConvergenceCriteria criteria;
	criteria.max_iteration_ = icpRunUntilConvergenceNumberOfIterations; // i.e. run until convergence
	const auto icpResult = open3d::pipelines::registration::RegistrationICP(source, target,
			icpMaxCorrespondenceDistance, Eigen::Matrix4d::Identity(),
			open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);

	Eigen::Matrix6d informationMatrix = Eigen::Matrix6d::Identity();
	if (isEstimateInformationMatrix) {
		informationMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target,
				icpMaxCorrespondenceDistance, icpResult.transformation_);
	}

	Constraint c;
	c.sourceSubmapIdx_ = sourceIdx;
	c.targetSubmapIdx_ = targetIdx;
	c.isOdometryConstraint_ = true;
	c.isInformationMatrixValid_ = isEstimateInformationMatrix;
	c.sourceToTarget_ = Transform(icpResult.transformation_);
//	c.sourceToTarget_ = Transform::Identity();
	c.informationMatrix_ = informationMatrix;

//	printf("submap %ld size: %ld \n", sourceIdx, sourceFull.points_.size());
//	printf("submap %ld overlap size: %ld \n", sourceIdx, source.points_.size());
//	printf("submap %ld size: %ld \n", targetIdx, targetFull.points_.size());
//	printf("submap %ld overlap size: %ld \n", targetIdx, target.points_.size());
//	printf("Adding odometry constraint from submap %ld to submap %ld with transformation \n: ",
//			sourceIdx, targetIdx);
//	std::cout << asString(c.sourceToTarget_) << std::endl;
	return c;

}

void computeOdometryConstraints(const SubmapCollection &submaps,
		const SubmapCollection::TimestampedSubmapIds &candidates, Constraints *constraints) {
	const size_t nSubmaps = submaps.getSubmaps().size();
	const size_t activeSubmapIdx = submaps.getActiveSubmap().getId();
	for (const auto candidate : candidates) {
		if (candidate.submapId_ < 1) {
			continue;
		}
		const size_t targetCandidate = candidate.submapId_;
		const size_t sourceCandidate = submaps.getSubmaps().at(targetCandidate).getParentId();
		if (!hasConstraint(sourceCandidate, targetCandidate, *constraints)) {
			const Constraint c = buildOdometryConstraint(sourceCandidate, targetCandidate, submaps);
			constraints->emplace_back(std::move(c));
		}

	}

}

void computeOdometryConstraints(const SubmapCollection &submaps, Constraints *constraints) {
	const size_t nSubmaps = submaps.getSubmaps().size();
	const size_t activeSubmapIdx = submaps.getActiveSubmap().getId();
	for (size_t submapIdx = 1; submapIdx < nSubmaps; ++submapIdx) {
		const size_t targetIdx = submapIdx;
		const size_t sourceIdx = submaps.getSubmaps().at(targetIdx).getParentId();
		if (!hasConstraint(sourceIdx, targetIdx, *constraints) && sourceIdx != activeSubmapIdx
				&& targetIdx != activeSubmapIdx) {
			const Constraint c = buildOdometryConstraint(sourceIdx, targetIdx, submaps);
			constraints->emplace_back(std::move(c));
		}
	}

}

} // namespace o3d_slam
