/*
 * WrapperRos.hpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <thread>
#include <future>
#include <Eigen/Dense>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/CircularBuffer.hpp"
#include "open3d_slam/ThreadSafeBuffer.hpp"
#include "open3d_slam/Constraint.hpp"


namespace o3d_slam {

class LidarOdometry;
class Mapper;
class Mesher;
class SubmapCollection;
class OptimizationProblem;


class WrapperRos {
	struct TimestampedPointCloud {
		Time time_;
		PointCloud cloud_;
	};

	struct RegisteredPointCloud{
		TimestampedPointCloud raw_;
		Transform transform_;
		std::string sourceFrame_, targetFrame_;
		size_t submapId_;
	};

public:
	WrapperRos(ros::NodeHandlePtr nh);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	~WrapperRos();

	void addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp);
	void initialize();
	void start();
	size_t getOdometryBufferSize() const;
	size_t getMappingBufferSize() const;
	size_t getOdometryBufferSizeLimit() const;
	size_t getMappingBufferSizeLimit() const;
	std::pair<PointCloud,Time> getLatestRegisteredCloudTimestampPair() const;

private:

	void odometryWorker();
	void mappingWorker();
	void loopClosureWorker();
	void computeFeaturesIfReady();
	void attemptLoopClosuresIfReady();
	void updateSubmapsAndTrajectory();
	void mesherWorker();
	void denseMapWorker();
	void publishMaps(const Time &time);
	void publishMapToOdomTf(const Time &time);


	ros::NodeHandlePtr nh_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
	ros::Publisher odometryInputPub_,mappingInputPub_,submapOriginsPub_, assembledMapPub_, denseMapPub_, submapsPub_, meshPub_;


	// non ros types
	CircularBuffer<RegisteredPointCloud> registeredCloudBuffer_;
	CircularBuffer<TimestampedPointCloud> odometryBuffer_, mappingBuffer_;
	CircularBuffer<Time> mesherBufffer_;
	ThreadSafeBuffer<TimestampedSubmapId> loopClosureCandidates_;
	MapperParameters mapperParams_;
	LocalMapParameters localMapParams_;
	MesherParameters mesherParams_;
	VisualizationParameters visualizationParameters_;
	PointCloud rawCloudPrev_;
	std::shared_ptr<Mesher> mesher_;
	std::shared_ptr<LidarOdometry> odometry_;
	std::shared_ptr<Mapper> mapper_;
	std::shared_ptr<SubmapCollection> submaps_;
	std::shared_ptr<OptimizationProblem> optimizationProblem_;
	std::string folderPath_;
	std::thread odometryWorker_, mappingWorker_, loopClosureWorker_, denseMapWorker_;
	Timer mappingStatisticsTimer_,odometryStatisticsTimer_, visualizationUpdateTimer_, denseMapVisualizationUpdateTimer_, denseMapStatiscticsTimer_;
	bool isVisualizationFirstTime_ = true;
	std::future<void> computeFeaturesResult_;
	bool isOptimizedGraphAvailable_ = false;
	Constraints lastLoopClosureConstraints_;
	bool isPublishMapsThreadRunning_ = false;

};

} // namespace o3d_slam
