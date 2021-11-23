/*
 * PlaceRecognition.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/PlaceRecognition.hpp"
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/SubmapCollection.hpp"
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
		size_t activeSubmapIdx) const {

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

	const auto source = sourceSubmap.getSparseMap();
	const auto sourceFeature = sourceSubmap.getFeatures();
	for (const auto id : closeSubmapsIdxs) {
		std::cout << "matching submap: " << lastFinishedSubmapIdx << " with submap: " << id << "\n";
		const auto &targetSubmap = submaps.at(id);
		const auto target = targetSubmap.getSparseMap();
		const auto targetFeature = targetSubmap.getFeatures();
		RegistrationResult ransacResult = RegistrationRANSACBasedOnFeatureMatching(source, target, sourceFeature,
				targetFeature, true, cfg.ransacMaxCorrespondenceDistance_, TransformationEstimationPointToPoint(false),
				cfg.ransacModelSize_, { distanceChecker, edgeLengthChecker },
				RANSACConvergenceCriteria(cfg.ransacNumIter_, cfg.ransacProbability_));

		if (ransacResult.correspondence_set_.size() < cfg.ransacMinCorrespondenceSetSize_) {
			std::cout << "skipping place recognition with: " << ransacResult.correspondence_set_.size()
					<< " correspondences. \n";
			continue;
		}

		ICPConvergenceCriteria icpCriteria;
		icpCriteria.max_iteration_ = cfg.icp_.maxNumIter_;
		const auto icpResult = open3d::pipelines::registration::RegistrationICP(source, target,
				cfg.featureVoxelSize_ * 3.0, ransacResult.transformation_, *icpObjective, icpCriteria);
		if (icpResult.fitness_ < cfg.minRefinementFitness_) {
			std::cout << "skipping place recognition with refinement score: " << icpResult.fitness_ << " \n";
			continue;
		}

		std::cout << "source features num: " << sourceFeature.Num() << "\n";
		std::cout << "target features num: " << targetFeature.Num() << "\n";
		std::cout << "num points source: " << source.points_.size() << "\n";
		std::cout << "num points target: " << target.points_.size() << "\n";
		std::cout << "registered num correspondences: " << ransacResult.correspondence_set_.size() << std::endl;
		std::cout << "registered with fitness: " << ransacResult.fitness_ << std::endl;
		std::cout << "registered with rmse: " << ransacResult.inlier_rmse_ << std::endl;
		std::cout << "registered with transformation: \n" << asString(Transform(ransacResult.transformation_))
				<< std::endl;
		std::cout << "refined with fitness: " << icpResult.fitness_ << std::endl;
		std::cout << "refined with rmse: " << icpResult.inlier_rmse_ << std::endl;
		std::cout << "refined with transformation: \n" << asString(Transform(icpResult.transformation_)) << std::endl;

		const Transform sourceToTarget(icpResult.transformation_);
		Constraint c;
		c.sourceToTarget_ = computeLoopClosingTransform(sourceSubmap, targetSubmap, sourceToTarget);
		c.sourceSubmapIdx_ = lastFinishedSubmapIdx;
		c.targetSubmapIdx_ = id;
		c.informationMatrix_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source,
				target, cfg.featureVoxelSize_ * 3.0 , icpResult.transformation_);
		c.isInformationMatrixValid_ = true;
		c.isOdometryConstraint_ = false;
		constraints.emplace_back(std::move(c));
	} // end for loop
	return constraints;
}

Transform PlaceRecognition::computeLoopClosingTransform(const Submap &sourceSubmap, const Submap &targetSubmap,
		const Transform &sourceToTarget) const {

//	std::vector<size_t> sourceIdxs, targetIdxs;
//	{
//		Timer t("overlap compute");
//	computeIndicesOfOverlappingPoints(source, target, sourceToTarget,
//			params_.placeRecognition_.featureVoxelSize_, &sourceIdxs, &targetIdxs);
//	}
//
//	Transform mapToSource = sourceSubmap.getMapToSubmapOrigin();
//	mapToSource.translation() = computeCenter(source, sourceIdxs);
//	Transform mapTotarget = targetSubmap.getMapToSubmapOrigin();
//	mapTotarget.translation() = computeCenter(target, targetIdxs);

	const auto mapToSource = sourceSubmap.getMapToSubmapOrigin() * sourceToTarget;
	const auto mapToTarget = targetSubmap.getMapToSubmapOrigin();
	const Transform T = mapToSource.inverse() * mapToTarget;

//	std::cout << "loop closing constraints: \n";
//	std::cout << " source: " << asString(mapToSource) << std::endl;
//	std::cout << " target: " << asString(mapToTarget) << std::endl;
//	std::cout << " source to target: " << asString(T) << std::endl;
//	std::cout << " source transformed into target: \n";
//	std::cout << asString(mapToSource * T) << "\n \n";

	return std::move(T);

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

