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
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"

#include "m545_volumetric_mapping/Odometry.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {

namespace {
using namespace m545_mapping::frames;
const double visualizationUpdatePeriodMsec = 400.0; // msec
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
	submaps_ = std::make_shared<m545_mapping::SubmapCollection>(optimizationProblem_);
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
		if (odometryStatisticsTimer_.elapsedSec() > 10.0) {
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

//		mesherBufffer_.push(measurement.time_);
		publishMapToOdomTf(measurement.time_);
		publishMaps(measurement.time_);
		const double timeMeasurement = mappingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		mappingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mappingStatisticsTimer_.elapsedSec() > 10.0) {
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

void WrapperRos::featureComputationWorker() {
	if (!submaps_->getFinishedSubmapIds().empty() && !submaps_->isComputingFeatures()) {
		submaps_->computeFeatures();
	}
}
void WrapperRos::loopClosureWorker() {
	if (!submaps_->getLoopClosureCandidateIds().empty() && !submaps_->isBuildingLoopClosureConstraints()) {
		submaps_->buildLoopClosureConstraints();
	}
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
void WrapperRos::globalOptimizationWorker() {
	if (optimizationProblem_->isReadyToOptimize() && !optimizationProblem_->isRunningOptimization()) {
		std::cout << "before optimization: \n";
		computeInformationMatrixOdometryConstraints(*submaps_, mapperParams_.placeRecognition_.featureVoxelSize_ * 3.0,
				optimizationProblem_->getOdometryConstraintsPtr());
		optimizationProblem_->buildOptimizationProblem(*submaps_);
		optimizationProblem_->print();
		submaps_->dumpToFile(folderPath_, "before");
		optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
		optimizationProblem_->solve();
		//			submaps_->dumpToFile(folderPath, "after");
		optimizationProblem_->print();
		//now here update the submaps
	}
}

void WrapperRos::publishMaps(const Time &time) {

	if (visualizationUpdateTimer_.elapsedMsec() < visualizationUpdatePeriodMsec && !isVisualizationFirstTime_) {
		return;
	}

	const auto timestamp = toRos(time);
	std::thread t(
			[this,timestamp]() {
				m545_mapping::publishCloud(mapper_->getAssembledMap(), m545_mapping::frames::mapFrame, timestamp,
						assembledMapPub_);

				m545_mapping::publishCloud(mapper_->getDenseMap(), m545_mapping::frames::mapFrame, timestamp,
						denseMapPub_);
				m545_mapping::publishSubmapCoordinateAxes(mapper_->getSubmaps(), m545_mapping::frames::mapFrame,
						timestamp, submapOriginsPub_);
				if (submapsPub_.getNumSubscribers() > 0) {
					open3d::geometry::PointCloud cloud;
					m545_mapping::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
					auto decimated = *(cloud.VoxelDownSample(0.2));
					m545_mapping::publishCloud(decimated, m545_mapping::frames::mapFrame, timestamp, submapsPub_);
				}
			});
	t.detach();
	visualizationUpdateTimer_.reset();
	isVisualizationFirstTime_ = false;
}

} // namespace m545_mapping

