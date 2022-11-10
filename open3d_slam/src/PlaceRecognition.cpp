/*
 * PlaceRecognition.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "open3d_slam/PlaceRecognition.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/assert.hpp"

#include "open3d_slam/CloudRegistration.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"

#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include "open3d_slam/helpers.hpp"

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<CloudRegistration> cloudRegistration;
} // namespace

PlaceRecognition::PlaceRecognition() {
	updateRegistrationAlgorithm(params_);
}

void PlaceRecognition::setParameters(const MapperParameters &p) {
	params_ = p;
	updateRegistrationAlgorithm(params_);
}

void PlaceRecognition::updateRegistrationAlgorithm(const MapperParameters &p){
	params_.scanMatcher_.icp_.maxNumIter_ = magic::icpRunUntilConvergenceNumberOfIterations;
	params_.scanMatcher_.icp_.maxCorrespondenceDistance_ = params_.placeRecognition_.maxIcpCorrespondenceDistance_;
	cloudRegistration = cloudRegistrationFactory(toCloudRegistrationType(params_.scanMatcher_));
}

Constraints PlaceRecognition::buildLoopClosureConstraints(const Transform &mapToRangeSensor,
		const SubmapCollection &submapCollection, const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx,
		size_t activeSubmapIdx, const Time &timestamp) const {

	using namespace open3d::pipelines::registration;
	Constraints constraints;
	const PlaceRecognitionParameters &cfg = params_.placeRecognition_;
	const auto edgeLengthChecker = CorrespondenceCheckerBasedOnEdgeLength(cfg.correspondenceCheckerEdgeLength_);
	const auto distanceChecker = CorrespondenceCheckerBasedOnDistance(cfg.correspondenceCheckerDistance_);
	const Submap &sourceSubmap = submapCollection.getSubmap(lastFinishedSubmapIdx);
	const std::vector<size_t> closeSubmapsIdxs = std::move(
			getLoopClosureCandidatesIdxs(mapToRangeSensor, submapCollection, adjMatrix, lastFinishedSubmapIdx,
					activeSubmapIdx));
	std::cout << "considering submap " << lastFinishedSubmapIdx << " for loop closure, num candidate submaps: "
			<< closeSubmapsIdxs.size() << std::endl;
	using namespace open3d::pipelines::registration;
	if (closeSubmapsIdxs.empty()) {
		return constraints;
	}
	const PointCloud sourceSparse = sourceSubmap.getSparseMapPointCloud();
	const PointCloud source = sourceSubmap.getMapPointCloudCopy();
	const Submap::Feature sourceFeature = sourceSubmap.getFeatures();
//#pragma omp parallel for
	for (int i = 0; i < closeSubmapsIdxs.size(); ++i) {
		const int id = closeSubmapsIdxs.at(i);
		const std::string matchingSubmapsString = " submap: " + std::to_string(lastFinishedSubmapIdx) + " with submap " + std::to_string(id);

		const Submap &targetSubmap = submapCollection.getSubmap(id);
		const PointCloud targetSparse = targetSubmap.getSparseMapPointCloud();
		const Submap::Feature targetFeature = targetSubmap.getFeatures();
		RegistrationResult ransacResult;
		{
			Timer t("ransac matching");
			ransacResult = RegistrationRANSACBasedOnFeatureMatching(sourceSparse, targetSparse, sourceFeature,
					targetFeature, true, cfg.ransacMaxCorrespondenceDistance_,
					TransformationEstimationPointToPoint(false), cfg.ransacModelSize_, { distanceChecker,
							edgeLengthChecker }, RANSACConvergenceCriteria(cfg.ransacNumIter_, cfg.ransacProbability_));
		}
		if (ransacResult.correspondence_set_.size() < cfg.ransacMinCorrespondenceSetSize_) {
			std::cout << "REJECTED loop closure, " << ransacResult.correspondence_set_.size()
					<< " correspondences. " << matchingSubmapsString << "\n";
			continue;
		}

		if (!isRegistrationConsistent(ransacResult.transformation_)) {
			std::cout << "REJECTED loop closure, with ransac inconsistant " << matchingSubmapsString << "\n";
			continue;
		}

		const PointCloud target = targetSubmap.getMapPointCloudCopy();
		const double mapVoxelSize = getMapVoxelSize(params_.mapBuilder_,
				magic::voxelSizeCorrespondenceSearchIfMapVoxelSizeIsZero);

		const double voxelSizeForOverlap = magic::voxelExpansionFactorOverlapComputation * mapVoxelSize;
		const size_t minNumPointsPerVoxel = 1;
		std::vector<size_t> sourceIdxs, targetIdxs;
		computeIndicesOfOverlappingPoints(source, target, Transform(ransacResult.transformation_),
				voxelSizeForOverlap, minNumPointsPerVoxel, &sourceIdxs, &targetIdxs);
		const PointCloud sourceOverlap = *source.SelectByIndex(sourceIdxs);
		const PointCloud targetOverlap = *target.SelectByIndex(targetIdxs);

//		const auto &sourceOverlap = source;
//		const auto &targetOverlap = target;

		const auto icpResult = cloudRegistration->registerClouds(sourceOverlap, targetOverlap,Transform(ransacResult.transformation_));
//		printf("submap %ld size: %ld \n", id, source.points_.size());
//			printf("submap %ld overlap size: %ld \n", id, source.points_.size());
//			printf("submap %ld size: %ld \n", lastFinishedSubmapIdx, target.points_.size());
//			printf("submap %ld overlap size: %ld \n", lastFinishedSubmapIdx, target.points_.size());

		const auto &cfg = params_.placeRecognition_;
		if (icpResult.fitness_ < cfg.minRefinementFitness_) {
			std::cout << "REJECTED loop closure, refinement score: " << icpResult.fitness_ << ", " << matchingSubmapsString << "\n";;
			continue;
		}

		if (!isRegistrationConsistent(icpResult.transformation_)) {
			std::cout << "REJECTED loop closure, icp reg inconsistent, " << matchingSubmapsString << "\n";;
			continue;
		}

//#pragma omp critical
		{
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
		}

		Constraint c;
		c.sourceToTarget_ = Transform(icpResult.transformation_);
		c.sourceSubmapIdx_ = lastFinishedSubmapIdx;
		c.targetSubmapIdx_ = id;
		c.informationMatrix_ = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(sourceOverlap,
				targetOverlap, cfg.maxIcpCorrespondenceDistance_, icpResult.transformation_);
		c.isInformationMatrixValid_ = true;
		c.isOdometryConstraint_ = false;
		c.timestamp_ = timestamp;
//#pragma omp critical
		{
			constraints.emplace_back(std::move(c));
		}
		assert_eq<int>(lastFinishedSubmapIdx,sourceSubmap.getId(), "oops source submap");
		assert_eq<int>(id,targetSubmap.getId(), "oops target submap");

		if (params_.placeRecognition_.isDumpPlaceRecognitionAlignmentsToFile_) {
			std::string lcName = std::to_string(recognitionCounter_) + "_"+ std::to_string(sourceSubmap.getId())+"_"+std::to_string(targetSubmap.getId());
			PointCloud sourceOverlapCopy = sourceOverlap;
			PointCloud sourceCopy = source;
			sourceCopy.Transform(icpResult.transformation_);
			sourceOverlapCopy.Transform(icpResult.transformation_);
			saveToFile(folderPath_ + "/overlap_source_" + lcName, sourceOverlapCopy);
			saveToFile(folderPath_ + "/full_source_" + lcName, sourceCopy);
			saveToFile(folderPath_ + "/full_target_" + lcName, target);
			saveToFile(folderPath_ + "/overlap_target_" + lcName, targetOverlap);
//			std::cout << "Dumped place recognition to file \n";
		}
		std::cout << "ACCEPTED loop closure: " << matchingSubmapsString <<", " << asStringXYZRPY(c.sourceToTarget_) << "\n";;

	} // end for loop
	return constraints;
}

void PlaceRecognition::setFolderPath(const std::string &folderPath) {
	folderPath_ = folderPath;
}

bool PlaceRecognition::isRegistrationConsistent(const Eigen::Matrix4d &mat) const {
	const double kRadToDeg = 180.0 / M_PI;
	const Transform T(mat);
	const Eigen::Vector3d rpy = toRPY(Eigen::Quaterniond(T.rotation()));
	const double roll = rpy.x();
	const double pitch = rpy.y();
	const double yaw = rpy.z();
	bool result = true;
	const PlaceRecognitionConsistencyCheckParameters &p = params_.placeRecognition_.consistencyCheck_;
	if (std::fabs(roll) > p.maxDriftRoll_) {
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The roll drift is: " << roll * kRadToDeg
				<< " [deg] which is > than " << p.maxDriftRoll_ * kRadToDeg << "\n";
	}
	if (std::fabs(pitch) > p.maxDriftPitch_) {
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The pitch drift is: " << pitch * kRadToDeg
				<< " [deg] which is > than " << p.maxDriftPitch_ * kRadToDeg << "\n";
	}
	if (std::fabs(yaw) > p.maxDriftYaw_) {
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The yaw drift is: " << yaw * kRadToDeg
				<< " [deg] which is > than " << p.maxDriftYaw_ * kRadToDeg << "\n";
	}
	if (std::fabs(T.translation().x()) > p.maxDriftX_){
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The x drift is: " << T.translation().x()
				<< " [m] which is > than " << p.maxDriftX_ << "\n";
	}
	if (std::fabs(T.translation().y()) > p.maxDriftY_){
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The y drift is: " << T.translation().y()
				<< " [m] which is > than " << p.maxDriftY_ << "\n";
	}
	if (std::fabs(T.translation().z()) > p.maxDriftZ_){
		result = false;
		std::cout << "  PlaceRecognition::isRegistrationConsistent The z drift is: " << T.translation().z()
				<< " [m] which is > than " << p.maxDriftZ_ << "\n";
	}

	if (!result) {
		std::cout << "   It is very unlikely that lidar odometry has drifted that much. Most likely, "
				"the place recognition module has fallen prey to spatial aliasing. If you are sure that this is "
				"not the case, feel free to disable this check! \n";
	}

	return result;
}

std::vector<size_t> PlaceRecognition::getLoopClosureCandidatesIdxs(const Transform &mapToRangeSensor,
		const SubmapCollection &submapCollection, const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx,
		size_t activeSubmapIdx) const {
	std::vector<size_t> idxs;
	const size_t nSubmaps = submapCollection.getNumSubmaps();
	idxs.reserve(nSubmaps);
	const Eigen::Vector3d lastFinishedSubmabCenter = submapCollection.getSubmap(lastFinishedSubmapIdx).getMapToSubmapCenter();
	for (size_t i = 0; i < nSubmaps; ++i) {
		if (i == activeSubmapIdx) {
			continue;
		}
		const std::string matchingSubmapsString = " submap: " + std::to_string(lastFinishedSubmapIdx) + " with submap " + std::to_string(i);
		const auto id1 = submapCollection.getSubmap(i).getId();
		const auto id2 = submapCollection.getSubmap(activeSubmapIdx).getId();
		if (adjMatrix.isAdjacent(id1, id2)) {
			continue;
		}

		const bool isAdjacent = std::abs<int>(i - lastFinishedSubmapIdx) == 1
				|| adjMatrix.isAdjacent(i, lastFinishedSubmapIdx);
		if (isAdjacent){
//			std::cout << "Skipping the loop closure of " << matchingSubmapsString
//					<< " since they are adjacent \n";
			continue;
		}

		const Eigen::Vector3d submapCenter = submapCollection.getSubmap(i).getMapToSubmapCenter();
		const double maxDistance = params_.placeRecognition_.loopClosureSearchRadius_;
		const double distance = (lastFinishedSubmabCenter-submapCenter).norm();
//		const double distance = (mapToRangeSensor.translation() - submapCollection.getSubmap(i).getMapToSubmapCenter()).norm();
		const bool isTooFar = distance > maxDistance;
//		std::cout << "distance submap to submap " << i << " : " << distance << std::endl;
		if (isTooFar) {
			continue;
		}

		const int consecutiveThreshold = (int) std::ceil(maxDistance / params_.submaps_.radius_);
		const bool isConsecutive = std::abs<int>(i - lastFinishedSubmapIdx) <= consecutiveThreshold;
		if (isConsecutive) {
			continue;
		}

		const int loopClosingDistance = adjMatrix.getDistanceToNearestLoopClosureSubmap(lastFinishedSubmapIdx);
//		std::cout << "submap " << lastFinishedSubmapIdx<<" has lc dist of: " << loopClosingDistance << "\n";
		if (loopClosingDistance < params_.placeRecognition_.minSubmapsBetweenLoopClosures_){
			std::cout << "Skipping the loop closure of " << matchingSubmapsString << " since there are fewer than "<<  params_.placeRecognition_.minSubmapsBetweenLoopClosures_ << " submaps inbetween \n";
			continue;
		}

		idxs.push_back(i);
	}
	return idxs;
}

} //namespace o3d_slam

