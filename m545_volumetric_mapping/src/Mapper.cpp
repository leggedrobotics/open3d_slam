/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mapper.hpp"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/Voxel.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "open3d/utility/Eigen.h"
#include "open3d/utility/Helper.h"

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
} // namespace

Mapper::Mapper(const TransformInterpolationBuffer &odomToRangeSensorBuffer) :
		odomToRangeSensorBuffer_(odomToRangeSensorBuffer) {
	update(params_);
}

void Mapper::setParameters(const MapperParameters &p) {
	params_ = p;
	update(p);
}

void Mapper::update(const MapperParameters &p) {
	icpCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective = icpObjectiveFactory(p.scanMatcher_.icpObjective_);
	scanMatcherCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.scanProcessing_.croppingRadius_);
	mapBuilderCropper_ = std::make_shared<MaxRadiusCroppingVolume>(p.mapBuilder_.scanCroppingRadius_);
	submaps_.setParameters(p);
}

bool Mapper::isMatchingInProgress() const {
	return isMatchingInProgress_;
}

Transform Mapper::getMapToOdom( const Time &timestamp) const {
	const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
	const Transform mapToRangeSensor = getTransform(timestamp, mapToRangeSensorBuffer_);
	return mapToRangeSensor * odomToRangeSensor.inverse();
//	return getTransform(timestamp, mapToOdomBuffer_);
}
Transform Mapper::getMapToRangeSensor( const Time &timestamp) const {
	return getTransform(timestamp, mapToRangeSensorBuffer_);
}

const SubmapCollection& Mapper::getSubmaps() const {
	return submaps_;
}

void Mapper::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (!pcl->HasNormals() && params_.scanMatcher_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, pcl);
		pcl->NormalizeNormals(); //todo, dunno if I need this
	}
}

void Mapper::addRangeMeasurement(const Mapper::PointCloud &rawScan, const Time &timestamp) {
	isMatchingInProgress_ = true;
	lastMeasurementTimestamp_ = timestamp;
	scanMatcherCropper_->setPose(Transform::Identity());
	submaps_.setMapToRangeSensor(mapToRangeSensor_);

	//insert first scan
	if (submaps_.getActiveSubmap().isEmpty()) {
		auto wideCroppedCloud = preProcessScan(rawScan);
		submaps_.insertScan(rawScan, *wideCroppedCloud, Transform::Identity(),timestamp);
		mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
//		mapToOdomBuffer_.push(timestamp,mapToOdom_);
		isMatchingInProgress_ = false;
		return;
	}


	if(!odomToRangeSensorBuffer_.has(timestamp)){
		std::cerr <<"WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
		isMatchingInProgress_ = false;
		return;
	}

	Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
	const auto odometryMotion = odomToRangeSensorPrev_.inverse() * odomToRangeSensor;
	const auto mapToRangeSensorEstimate = getMapToOdom(timestamp) * odomToRangeSensor;
	const auto &activeSubmap = submaps_.getActiveSubmap().getMap();
	std::shared_ptr<PointCloud> narrowCropped, wideCroppedCloud, mapPatch;
	{
		static double avgTime = 0;
		static int count = 0;
		Timer timer;
		wideCroppedCloud = preProcessScan(rawScan);
		narrowCropped = scanMatcherCropper_->crop(*wideCroppedCloud);
		scanMatcherCropper_->setPose(mapToRangeSensor_);
		mapPatch = scanMatcherCropper_->crop(activeSubmap);
//		avgTime += timer.elapsedMsec();
//		++count;
//		std::cout << "avg preprocess: " << avgTime / count << "\n";
	}

	const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
			params_.scanMatcher_.maxCorrespondenceDistance_, mapToRangeSensorEstimate.matrix(), *icpObjective,
			icpCriteria_);

	if (result.fitness_ < params_.minRefinementFitness_) {
		std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
		isMatchingInProgress_ = false;
		return;
	}

	// update transforms
	mapToRangeSensor_.matrix() = result.transformation_;
	mapToOdom_ = mapToRangeSensor_ * odomToRangeSensor.inverse();
	mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
//	mapToOdomBuffer_.push(timestamp,mapToOdom_);

	// concatenate registered cloud into map
	submaps_.setMapToRangeSensor(mapToRangeSensor_);
	const bool isMovedTooLittle = odometryMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
	if (!isMovedTooLittle) {
		submaps_.insertScan(rawScan, *wideCroppedCloud, mapToRangeSensor_,timestamp);
		odomToRangeSensorPrev_ = odomToRangeSensor;
	}

	//check the constraints
	// update the mapToRange sensor interpolation buffer
	// update submaps - only do this once pose graph is added
	// update active submaps!
	const auto constraints = submaps_.getAndClearConstraints();
	if(!constraints.empty()){
		std::cout << "loop closing resulted in " << constraints.size() << " constraints \n";
	}
	auto mapToRangeSensorCopy = mapToRangeSensor_;
	for(const auto &c : constraints){
		const auto lastTimeInBuffer= mapToRangeSensorBuffer_.latest_time();
		const auto beginTimeInBuffer = std::max(c.timeBegin_,c.timeFinish_);
		assert_ge(toUniversal(lastTimeInBuffer), toUniversal(beginTimeInBuffer));
		mapToRangeSensorBuffer_.applyToAllElementsInTimeInterval(c.relativeTransformation_, beginTimeInBuffer, lastTimeInBuffer);
		mapToRangeSensor_ = mapToRangeSensorCopy * c.relativeTransformation_;
		std::cout << "updating current pose estimate with transform: \n" <<asString(c.relativeTransformation_) << "\n";
	}
	submaps_.updateActiveSubmap(mapToRangeSensor_);


	isMatchingInProgress_ = false;
}

std::shared_ptr<Mapper::PointCloud> Mapper::preProcessScan(const PointCloud &rawScan) const {
	mapBuilderCropper_->setPose(Transform::Identity());
	auto wideCroppedCloud = mapBuilderCropper_->crop(rawScan);
	m545_mapping::voxelize(params_.scanProcessing_.voxelSize_, wideCroppedCloud.get());
	m545_mapping::randomDownSample(params_.scanProcessing_.downSamplingRatio_, wideCroppedCloud.get());
	estimateNormalsIfNeeded(wideCroppedCloud.get());
	return wideCroppedCloud;
}

const Mapper::PointCloud& Mapper::getMap() const {
	return submaps_.getActiveSubmap().getMap();
}

Mapper::PointCloud Mapper::getAssembledMap() const {
	PointCloud cloud;
	const int nSubmaps = submaps_.getSubmaps().size();
	const int nPoints = submaps_.getTotalNumPoints();
	cloud.points_.reserve(nPoints);
	if (getMap().HasColors()) {
		cloud.colors_.reserve(nPoints);
	}
	if (getMap().HasNormals()) {
		cloud.normals_.reserve(nPoints);
	}

	for (size_t j = 0; j < submaps_.getSubmaps().size(); ++j) {
		const auto &submap = submaps_.getSubmaps().at(j);
		const auto color = Color::getColor(j % (Color::numColors_ - 2) + 2);
		for (size_t i = 0; i < submap.getMap().points_.size(); ++i) {
			cloud.points_.push_back(submap.getMap().points_.at(i));
			if (getMap().HasColors()) {
				cloud.colors_.push_back(submap.getMap().colors_.at(i));
			}
			if (getMap().HasNormals()) {
				cloud.normals_.push_back(submap.getMap().normals_.at(i));
			}
		}
	}
	return cloud;
}

const Submap& Mapper::getActiveSubmap() const {
	return submaps_.getActiveSubmap();
}

const Mapper::PointCloud& Mapper::getDenseMap() const {
	return submaps_.getActiveSubmap().getDenseMap();
}

bool Mapper::isManipulatingMap() const {
	return isManipulatingMap_;
}

void Mapper::attemptLoopClosures() {
	submaps_.computeFeaturesInLastFinishedSubmap();

	if (!submaps_.isBuildingLoopClosureConstraints()) {
		submaps_.buildLoopClosureConstraints();
	}
}

const TransformInterpolationBuffer &Mapper::getMapToRangeSensorBuffer() const{
	return mapToRangeSensorBuffer_;
}

bool Mapper::isReadyForLoopClosure() const {
	return submaps_.isFinishedSubmap();
}

} /* namespace m545_mapping */
