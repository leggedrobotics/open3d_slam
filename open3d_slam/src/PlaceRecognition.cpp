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
	const PlaceRecognitionParameters &cfg = params_.placeRecognition_;
	const auto edgeLengthChecker = CorrespondenceCheckerBasedOnEdgeLength(cfg.correspondenceCheckerEdgeLength_);
	const auto distanceChecker = CorrespondenceCheckerBasedOnDistance(cfg.correspondenceCheckerDistance_);
	const SubmapCollection::Submaps &submaps = submapCollection.getSubmaps();
	const Submap &sourceSubmap = submaps.at(lastFinishedSubmapIdx);
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
	const PointCloud source = sourceSubmap.getMapPointCloud();
	const Submap::Feature sourceFeature = sourceSubmap.getFeatures();
//#pragma omp parallel for
	for (int i = 0; i < closeSubmapsIdxs.size(); ++i) {
		const int id = closeSubmapsIdxs.at(i);
		const bool isAdjacent = std::abs<int>(id - lastFinishedSubmapIdx) == 1
				|| adjMatrix.isAdjacent(id, lastFinishedSubmapIdx);
		if (!isAdjacent) {
			std::cout << "matching submap: " << lastFinishedSubmapIdx << " with submap: " << id << "\n";
		} else {
			std::cout << "Skipping the loop closure of: " << lastFinishedSubmapIdx << " with submap: " << id
					<< " since they are adjacent \n";
			continue;
		}
		const Submap &targetSubmap = submaps.at(id);
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
			std::cout << "skipping place recognition with: " << ransacResult.correspondence_set_.size()
					<< " correspondences. \n";
			continue;
		}

		if (!isRegistrationConsistent(ransacResult.transformation_)) {
			std::cout << "skipping place recognition \n";
			continue;
		}

		const PointCloud target = targetSubmap.getMapPointCloud();
		const double mapVoxelSize = getMapVoxelSize(params_.mapBuilder_,
				voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);

		const double voxelSizeForOverlap = voxelExpansionFactorOverlapComputation * mapVoxelSize;
		const size_t minNumPointsPerVoxel = 1;
		std::vector<size_t> sourceIdxs, targetIdxs;
		computeIndicesOfOverlappingPoints(source, target, Transform(ransacResult.transformation_),
				voxelSizeForOverlap, minNumPointsPerVoxel, &sourceIdxs, &targetIdxs);
		const PointCloud sourceOverlap = *source.SelectByIndex(sourceIdxs);
		const PointCloud targetOverlap = *target.SelectByIndex(targetIdxs);

//		const auto &sourceOverlap = source;
//		const auto &targetOverlap = target;

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

		if (!isRegistrationConsistent(icpResult.transformation_)) {
			std::cout << "skipping place recognition \n";
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
		if (params_.placeRecognition_.isDumpPlaceRecognitionAlignmentsToFile_) {
			PointCloud sourceOverlapCopy = sourceOverlap;
			PointCloud sourceCopy = source;
			sourceCopy.Transform(icpResult.transformation_);
			sourceOverlapCopy.Transform(icpResult.transformation_);
			saveToFile(folderPath_ + "/source_" + std::to_string(recognitionCounter_), sourceOverlapCopy);
			saveToFile(folderPath_ + "/sourceFull_" + std::to_string(recognitionCounter_), sourceCopy);
			saveToFile(folderPath_ + "/targetFull_" + std::to_string(recognitionCounter_), target);
			saveToFile(folderPath_ + "/target_" + std::to_string(recognitionCounter_++), targetOverlap);
			std::cout << "Dumped place recognition to file \n";
		}

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
	const PlaceRecognitionConsistancyCheckParameters &p = params_.placeRecognition_.consistencyCheck_;
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

} //namespace o3d_slam

