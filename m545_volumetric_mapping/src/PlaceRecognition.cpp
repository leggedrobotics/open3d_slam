/*
 * PlaceRecognition.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/PlaceRecognition.hpp"
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include "m545_volumetric_mapping/magic.hpp"

#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include "m545_volumetric_mapping/helpers.hpp"

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
} // namespace

PlaceRecognition::PlaceRecognition() {
	icpObjective = icpObjectiveFactory(IcpObjective::PointToPlane);
}

void PlaceRecognition::setParameters(const MapperParameters &p) {
	params_ = p;

}
Constraints PlaceRecognition::buildLoopClosureConstraints(const Transform &mapToRangeSensor,
		const SubmapCollection &submapCollection, const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx,
		size_t activeSubmapIdx, const Time &timestamp) const {

	using namespace open3d::pipelines::registration;
	Constraints constraints;
	const auto &cfg = params_.placeRecognition_;
	const auto edgeLengthChecker = CorrespondenceCheckerBasedOnEdgeLength(cfg.correspondenceCheckerEdgeLength_);
	const auto distanceChecker = CorrespondenceCheckerBasedOnDistance(cfg.correspondenceCheckerDistance_);
	const auto &submaps = submapCollection.getSubmaps();
	const auto &sourceSubmap = submaps.at(lastFinishedSubmapIdx);
	const auto closeSubmapsIdxs = std::move(
			getLoopClosureCandidatesIdxs(mapToRangeSensor, submapCollection, adjMatrix, lastFinishedSubmapIdx,
					activeSubmapIdx));
	std::cout << "considering submap " << lastFinishedSubmapIdx << " for loop closure, num candidate submaps: "
			<< closeSubmapsIdxs.size() << std::endl;
	using namespace open3d::pipelines::registration;
	if (closeSubmapsIdxs.empty()) {
		return constraints;
	}
	const auto sourceSparse = sourceSubmap.getSparseMap();
	const auto source = sourceSubmap.getMap();
	const auto sourceFeature = sourceSubmap.getFeatures();
	for (const auto id : closeSubmapsIdxs) {
		const bool isAdjacent = std::abs<int>(id - lastFinishedSubmapIdx) == 1;
		if (!isAdjacent){
			std::cout << "matching submap: " << lastFinishedSubmapIdx << " with submap: " << id << "\n";
		} else {
			std::cout << "Skipping the loop closure of: " << lastFinishedSubmapIdx << " with submap: " << id << " since they are adjacent \n";
			continue;
		}
		const auto &targetSubmap = submaps.at(id);
		const auto targetSparse = targetSubmap.getSparseMap();
		const auto targetFeature = targetSubmap.getFeatures();
		RegistrationResult ransacResult;
		{
			Timer t("ransac matching");
			ransacResult = RegistrationRANSACBasedOnFeatureMatching(sourceSparse, targetSparse, sourceFeature,
					targetFeature, true, cfg.ransacMaxCorrespondenceDistance_,
					TransformationEstimationPointToPoint(false), cfg.ransacModelSize_, { distanceChecker,
							edgeLengthChecker }, RANSACConvergenceCriteria(cfg.ransacNumIter_, cfg.ransacProbability_));
		}
		if (ransacResult.correspondence_set_.size() < cfg.ransacMinCorrespondenceSetSize_) {
			std::cout << "skipping place recognition with: " << ransacResult.correspondence_set_.size()
					<< " correspondences. \n";
			continue;
		}

		const auto target = targetSubmap.getMap();
		const double mapVoxelSize = getMapVoxelSize(params_.mapBuilder_,
				voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);
		const double voxelSizeForOverlap = 3.0 * mapVoxelSize;
		const size_t minNumPointsPerVoxel = 1;
		std::vector<size_t> sourceIdxs, targetIdxs;
		computeIndicesOfOverlappingPoints(source, target, Transform(ransacResult.transformation_),
				voxelSizeForOverlap, minNumPointsPerVoxel, &sourceIdxs, &targetIdxs);
		const auto sourceOverlap = *source.SelectByIndex(sourceIdxs);
		const auto targetOverlap = *target.SelectByIndex(targetIdxs);

		open3d::pipelines::registration::ICPConvergenceCriteria criteria;
		criteria.max_iteration_ = icpRunUntilConvergenceNumberOfIterations; // i.e. run until convergence
		const auto icpResult = open3d::pipelines::registration::RegistrationICP(sourceOverlap, targetOverlap,
				cfg.maxIcpCorrespondenceDistance_, ransacResult.transformation_,
				open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);
//		printf("submap %ld size: %ld \n", id, source.points_.size());
//			printf("submap %ld overlap size: %ld \n", id, source.points_.size());
//			printf("submap %ld size: %ld \n", lastFinishedSubmapIdx, target.points_.size());
//			printf("submap %ld overlap size: %ld \n", lastFinishedSubmapIdx, target.points_.size());

		const auto &cfg = params_.placeRecognition_;
		if (icpResult.fitness_ < cfg.minRefinementFitness_) {
			std::cout << "skipping place recognition with refinement score: " << icpResult.fitness_ << " \n";
			continue;
		}

		std::cout << "source features num: " << sourceSubmap.getFeatures().Num() << "\n";
		std::cout << "target features num: " << sourceSubmap.getFeatures().Num() << "\n";
		std::cout << "registered num correspondences: " << ransacResult.correspondence_set_.size() << std::endl;
		std::cout << "registered with fitness: " << ransacResult.fitness_ << std::endl;
		std::cout << "registered with rmse: " << ransacResult.inlier_rmse_ << std::endl;
		std::cout << "registered with transformation: \n" << asString(Transform(ransacResult.transformation_))
				<< std::endl;

		std::cout << "refined with fitness: " << icpResult.fitness_ << std::endl;
		std::cout << "refined with rmse: " << icpResult.inlier_rmse_ << std::endl;
		std::cout << "refined with transformation: \n" << asString(Transform(icpResult.transformation_))
				<< std::endl;

		Constraint c;
		c.sourceToTarget_ = Transform(icpResult.transformation_);
		c.sourceSubmapIdx_ = lastFinishedSubmapIdx;
		c.targetSubmapIdx_ = id;
		c.informationMatrix_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(sourceOverlap,
				targetOverlap, mapVoxelSize * 1.5, icpResult.transformation_);
		c.isInformationMatrixValid_ = true;
		c.isOdometryConstraint_ = false;
		c.timestamp_ = timestamp;
		constraints.emplace_back(std::move(c));

	} // end for loop
	return constraints;
}

std::vector<size_t> PlaceRecognition::getLoopClosureCandidatesIdxs(const Transform &mapToRangeSensor,
		const SubmapCollection &submapCollection, const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx,
		size_t activeSubmapIdx) const {
	std::vector<size_t> idxs;
	const auto &submaps = submapCollection.getSubmaps();
	const size_t nSubmaps = submaps.size();
	idxs.reserve(nSubmaps);
	for (size_t i = 0; i < nSubmaps; ++i) {
		if (i == activeSubmapIdx) {
			continue;
		}
		const auto id1 = submaps.at(i).getId();
		const auto id2 = submaps.at(activeSubmapIdx).getId();
		if (adjMatrix.isAdjacent(id1, id2)) {
			continue;
		}

		const double distance = (mapToRangeSensor.translation() - submaps.at(i).getMapToSubmapCenter()).norm();
		const bool isTooFar = distance > params_.submaps_.radius_;
//		std::cout << "distance submap to submap " << i << " : " << distance << std::endl;
		if (isTooFar) {
			continue;
		}

		idxs.push_back(i);
	}
	return idxs;
}

} //namespace m545_mapping

