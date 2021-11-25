/*
 * WrapperRos.cpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/WrapperRos.hpp"

#include <chrono>
#include <open3d/Open3D.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"

#include "m545_volumetric_mapping/Odometry.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {

namespace {
using namespace m545_mapping::frames;
const double visualizationUpdatePeriodMsec = 400.0; // msec
const double timingStatsEveryNsec = 15.0;
}

WrapperRos::WrapperRos(ros::NodeHandlePtr nh) :
		nh_(nh) {
	tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());
	odometryBuffer_.set_size_limit(20);
}

WrapperRos::~WrapperRos() {
	if (odometryWorker_.joinable()) {
		odometryWorker_.join();
		std::cout << "Joined odometry worker \n";
	}
	if (mappingWorker_.joinable()) {
		mappingWorker_.join();
		std::cout << "Joined mapping worker \n";
	}
	if (loopClosureWorker_.joinable()) {
		loopClosureWorker_.join();
		std::cout << "Joined the loop closure worker \n";
	}
}

void WrapperRos::addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp) {
	const TimestampedPointCloud timestampedCloud { timestamp, cloud };
	odometryBuffer_.push(timestampedCloud);
}

void WrapperRos::initialize() {

	odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
	assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
	denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);
	meshPub_ = nh_->advertise<m545_volumetric_mapping_msgs::PolygonMesh>("mesh", 1, true);

	submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
	submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	folderPath_ = ros::package::getPath("m545_volumetric_mapping") + "/data/";

	const std::string paramFile = nh_->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";

	m545_mapping::loadParameters(paramFile, &localMapParams_);

	OdometryParameters odometryParams;
	loadParameters(paramFile, &odometryParams);
	odometry_ = std::make_shared<m545_mapping::LidarOdometry>();
	odometry_->setParameters(odometryParams);

	optimizationProblem_ = std::make_shared<m545_mapping::OptimizationProblem>();
	submaps_ = std::make_shared<m545_mapping::SubmapCollection>();
	mapper_ = std::make_shared<m545_mapping::Mapper>(odometry_->getBuffer(), submaps_);
	m545_mapping::loadParameters(paramFile, &mapperParams_);
	mapper_->setParameters(mapperParams_);

	mesher_ = std::make_shared<m545_mapping::Mesher>();
	m545_mapping::loadParameters(paramFile, &mesherParams_);
	mesher_->setParameters(mesherParams_);
}

void WrapperRos::start() {
	odometryWorker_ = std::thread([this]() {
		odometryWorker();
	});
	mappingWorker_ = std::thread([this]() {
		mappingWorker();
	});
	loopClosureWorker_ = std::thread([this]() {
		loopClosureWorker();
	});
}

void WrapperRos::odometryWorker() {
	while (ros::ok()) {
		if (odometryBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		odometryStatisticsTimer_.startStopwatch();
		const TimestampedPointCloud measurement = odometryBuffer_.pop();
		if (!odometry_->addRangeScan(measurement.cloud_, measurement.time_)) {
			std::cerr << "WARNING: odometry has failed!!!! \n";
			continue;
		}
		// this ensures that the odom is always ahead of the mapping
		// so then we can look stuff up in the interpolation buffer
		mappingBuffer_.push(measurement);
		const auto timestamp = toRos(measurement.time_);
		m545_mapping::publishTfTransform(odometry_->getOdomToRangeSensor(measurement.time_).matrix(), timestamp,
				odomFrame, rangeSensorFrame, tfBroadcaster_.get());

		if (odometryInputPub_.getNumSubscribers() > 0) {
			auto odomInput = odometry_->getPreProcessedCloud();
			std::thread t(
					[this, timestamp, odomInput]() {
						m545_mapping::publishCloud(odomInput, m545_mapping::frames::rangeSensorFrame, timestamp,
								odometryInputPub_);
					});
			t.detach();
		}
		const double timeMeasurement = odometryStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		odometryStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (odometryStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Odometry timing stats: Avg execution time: "
					<< odometryStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / odometryStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			odometryStatisticsTimer_.reset();
		}

	} // end while

}
void WrapperRos::mappingWorker() {
	while (ros::ok()) {
		if (mappingBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		mappingStatisticsTimer_.startStopwatch();
		const TimestampedPointCloud measurement = mappingBuffer_.pop();

		if (!odometry_->getBuffer().has(measurement.time_)) {
			std::cout << "Weird, the odom buffer does not seem to have the transform!!! \n";
			std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/"
					<< odometry_->getBuffer().size_limit() << std::endl;
			const auto &b = odometry_->getBuffer();
			std::cout << "earliest: " << readable(b.earliest_time()) << std::endl;
			std::cout << "latest: " << readable(b.latest_time()) << std::endl;
			std::cout << "requested: " << readable(measurement.time_) << std::endl;
		}
		mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);
		publishMapToOdomTf(measurement.time_);
		//mesherBufffer_.push(measurement.time_);

		computeFeaturesIfReady();
		attemptLoopClosuresIfReady();

		if (isOptimizedGraphAvailable_) {
			isOptimizedGraphAvailable_ = false;
			updateSubmapsAndTrajectory();
		}

		// publish visualizatinos
		publishMaps(measurement.time_);

		//just get the stats
		const double timeMeasurement = mappingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		mappingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mappingStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Mapper timing stats: Avg execution time: " << mappingStatisticsTimer_.getAvgMeasurementMsec()
					<< " msec , frequency: " << 1e3 / mappingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			mappingStatisticsTimer_.reset();
		}

	}
}

void WrapperRos::publishMapToOdomTf(const Time &time) {
	const auto timestamp = toRos(time);
	m545_mapping::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster_.get());
}

void WrapperRos::computeFeaturesIfReady() {
	if (submaps_->numFinishedSubmaps() > 0 && !submaps_->isComputingFeatures()) {
		computeFeaturesResult_ = std::async(std::launch::async, [this]() {
			const auto finishedSubmapIds = submaps_->popFinishedSubmapIds();
			submaps_->computeFeatures(finishedSubmapIds);
		});
	}
}
void WrapperRos::attemptLoopClosuresIfReady() {
	const auto timeout = std::chrono::milliseconds(0);
	if (computeFeaturesResult_.valid() && computeFeaturesResult_.wait_for(timeout) == std::future_status::ready) {
		computeFeaturesResult_.get(); // consume the future
		if (submaps_->numLoopClosureCandidates() > 0) {
			const auto lcc = submaps_->popLoopClosureCandidates();
			loopClosureCandidates_.insert(lcc.begin(), lcc.end());
		}
	}
}
void WrapperRos::loopClosureWorker() {

	while (ros::ok()) {
		if (loopClosureCandidates_.empty() || isOptimizedGraphAvailable_) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}

		Constraints loopClosureConstraints;
		{
			Timer t("loop_closing_attempt");
			const auto lcc = loopClosureCandidates_.popAllElements();
			loopClosureConstraints = submaps_->buildLoopClosureConstraints(lcc);
		}

		if (loopClosureConstraints.empty()){
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}

		{
			Timer t("optimization_problem");
			auto odometryConstraints = submaps_->getOdometryConstraints();
			const double d = informationMatrixMaxCorrespondenceDistance(mapperParams_.mapBuilder_.mapVoxelSize_);
			computeInformationMatrixOdometryConstraints(*submaps_, d, &odometryConstraints);
			submaps_->addLoopClosureConstraints(loopClosureConstraints);

			optimizationProblem_->clearLoopClosureConstraints();
			optimizationProblem_->clearOdometryConstraints();
			optimizationProblem_->insertLoopClosureConstraints(loopClosureConstraints);
			optimizationProblem_->insertOdometryConstraints(odometryConstraints);
			optimizationProblem_->buildOptimizationProblem(*submaps_);

			//			optimizationProblem_->print();
			submaps_->dumpToFile(folderPath_, "before");
			optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
			optimizationProblem_->solve();
			lastLoopClosureConstraints_ = loopClosureConstraints;
			isOptimizedGraphAvailable_ = true;
		}

	} // end while

}

void WrapperRos::updateSubmapsAndTrajectory() {

	std::cout << "Updating the maps: \n";
	const Timer t("submaps_update");
	const auto optimizedTransformations = optimizationProblem_->getOptimizedTransformIncrements();
	submaps_->transform(optimizedTransformations);

	//get The correct time
	const Constraint latestLoopClosureConstraint = *std::max_element(lastLoopClosureConstraints_.begin(),
			lastLoopClosureConstraints_.end(), [](const Constraint &c1, const Constraint &c2) {
				return c1.timestamp_ < c2.timestamp_;
			});

	// loop closing constraints are built such that the source node is always the ne being transformed
	// hence the source node should be the one whose transform we should apply
	assert_gt(latestLoopClosureConstraint.sourceSubmapIdx_, latestLoopClosureConstraint.targetSubmapIdx_);
	const auto dT = optimizedTransformations.at(latestLoopClosureConstraint.sourceSubmapIdx_);
	const Time latestTime = mapper_->getMapToRangeSensorBuffer().latest_time();
//	const Time lastLoopClosureTime = latestLoopClosureConstraint.timestamp_;
	const Time lastLoopClosureTime = mapper_->getMapToRangeSensorBuffer().earliest_time();

	std::cout << "Applying the delta T from submap " << latestLoopClosureConstraint.sourceSubmapIdx_ << "\n";
	std::cout << "the transform is: " << asString(dT.dT_) << std::endl;

	mapper_->getMapToRangeSensorBufferPtr()->applyToAllElementsInTimeInterval(dT.dT_,lastLoopClosureTime , latestTime);

}

void WrapperRos::mesherWorker() {
	if (mesherParams_.isComputeMesh_ && !mesherBufffer_.empty() && !mapper_->getMap().points_.empty()) {
		auto map = mapper_->getMap();
		m545_mapping::MaxRadiusCroppingVolume cropper(localMapParams_.croppingRadius_);
		const auto time = mesherBufffer_.pop();
		const auto timestamp = toRos(time);
		cropper.setPose(mapper_->getMapToRangeSensor(time));
		cropper.crop(&map);
		auto downSampledMap = map.VoxelDownSample(mesherParams_.voxelSize_);
		mesher_->setCurrentPose(mapper_->getMapToRangeSensor(time));
		mesher_->buildMeshFromCloud(*downSampledMap);
		m545_mapping::publishMesh(mesher_->getMesh(), mapFrame, timestamp, meshPub_);
	}

}

void WrapperRos::publishMaps(const Time &time) {
	if (visualizationUpdateTimer_.elapsedMsec() < visualizationUpdatePeriodMsec && !isVisualizationFirstTime_) {
		return;
	}

	const auto timestamp = toRos(time);
	std::thread t(
			[this, timestamp]() {
				m545_mapping::publishCloud(mapper_->getAssembledMap(), m545_mapping::frames::mapFrame, timestamp,
						assembledMapPub_);

				m545_mapping::publishCloud(mapper_->getDenseMap(), m545_mapping::frames::mapFrame, timestamp,
						denseMapPub_);
				m545_mapping::publishSubmapCoordinateAxes(mapper_->getSubmaps(), m545_mapping::frames::mapFrame,
						timestamp, submapOriginsPub_);
				if (submapsPub_.getNumSubscribers() > 0) {
					open3d::geometry::PointCloud cloud;
					m545_mapping::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
					auto decimated = *(cloud.VoxelDownSample(0.5));
					m545_mapping::publishCloud(decimated, m545_mapping::frames::mapFrame, timestamp, submapsPub_);
				}
			});
	t.detach();
	visualizationUpdateTimer_.reset();
	isVisualizationFirstTime_ = false;
}

} // namespace m545_mapping

