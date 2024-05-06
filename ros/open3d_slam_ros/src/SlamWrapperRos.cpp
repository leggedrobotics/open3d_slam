/*
 * SlamWrapperRosRos.cpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <open3d/Open3D.h>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/odometry.h"
#include "open3d_conversions/open3d_conversions.hpp"
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

#ifdef open3d_slam_ros_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
using namespace o3d_slam::frames;
}

SlamWrapperRos::SlamWrapperRos(rclcpp::Node* nh, rclcpp::executors::SingleThreadedExecutor* executor) : BASE(), nh_(nh), executor_(executor) {
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
  prevPublishedTimeScanToScan_ = fromUniversal(0);
  prevPublishedTimeScanToMap_ = fromUniversal(0);
}

SlamWrapperRos::~SlamWrapperRos() {
  if (tfWorker_.joinable()) {
    tfWorker_.join();
    std::cout << "Joined tf worker \n";
  }
  if (visualizationWorker_.joinable()) {
    visualizationWorker_.join();
    std::cout << "Joined visualization worker \n";
  }
  if (params_.odometry_.isPublishOdometryMsgs_ && odomPublisherWorker_.joinable()) {
    odomPublisherWorker_.join();
    std::cout << "Joined odom publisher worker \n";
  }
}

void SlamWrapperRos::startWorkers() {
  tfWorker_ = std::thread([this]() { tfWorker(); });
  visualizationWorker_ = std::thread([this]() { visualizationWorker(); });
  if (params_.odometry_.isPublishOdometryMsgs_) {
    odomPublisherWorker_ = std::thread([this]() { odomPublisherWorker(); });
  }

  BASE::startWorkers();
}

void SlamWrapperRos::odomPublisherWorker() {
  rclcpp::Rate r(500.0);
  while (rclcpp::ok()) {
    auto getTransformMsg = [](const Transform& T, const Time& t) {
      rclcpp::Time timestamp = toRos(t);
      geometry_msgs::msg::TransformStamped transformMsg = o3d_slam::toRos(T.matrix(), timestamp, mapFrame, rangeSensorFrame);
      return transformMsg;
    };

    auto getOdomMsg = [](const geometry_msgs::msg::TransformStamped& transformMsg) {
      nav_msgs::msg::Odometry odomMsg;
      odomMsg.header = transformMsg.header;
      odomMsg.child_frame_id = transformMsg.child_frame_id;
      odomMsg.pose.pose.orientation = transformMsg.transform.rotation;
      odomMsg.pose.pose.position.x = transformMsg.transform.translation.x;
      odomMsg.pose.pose.position.y = transformMsg.transform.translation.y;
      odomMsg.pose.pose.position.z = transformMsg.transform.translation.z;
      return odomMsg;
    };

    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScanOdom_;
    if (!isAlreadyPublished && odometry_->hasProcessedMeasurements()) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      geometry_msgs::msg::TransformStamped transformMsg = getTransformMsg(T, latestScanToScan);
      nav_msgs::msg::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2scanTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2scanOdomPublisher_);
      prevPublishedTimeScanToScanOdom_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMapOdom_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      const Transform T = mapper_->getMapToRangeSensor(latestScanToMap);
      geometry_msgs::msg::TransformStamped transformMsg = getTransformMsg(T, latestScanToMap);
      nav_msgs::msg::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2mapTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2mapOdomPublisher_);
      prevPublishedTimeScanToMapOdom_ = latestScanToMap;
    }

    //executor_->spin_some();
    r.sleep();
  }
}

void SlamWrapperRos::tfWorker() {
  rclcpp::WallRate r(20.0);
  while (rclcpp::ok()) {
    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
    if (!isAlreadyPublished && odometry_->hasProcessedMeasurements()) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      rclcpp::Time timestamp = toRos(latestScanToScan);
      o3d_slam::publishTfTransform(T.matrix(), timestamp, odomFrame, rangeSensorFrame, tfBroadcaster_.get());
      o3d_slam::publishTfTransform(T.matrix(), timestamp, mapFrame, "raw_odom_o3d", tfBroadcaster_.get());
      prevPublishedTimeScanToScan_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      publishMapToOdomTf(latestScanToMap);
      prevPublishedTimeScanToMap_ = latestScanToMap;
    }

    //executor_->spin_some();
    r.sleep();
  }
}
void SlamWrapperRos::visualizationWorker() {
  rclcpp::WallRate r(20.0);
  while (rclcpp::ok()) {
    const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
    if (odometryInputPub_->get_subscription_count() > 0 && isTimeValid(scanToScanTimestamp)) {
      const PointCloud odomInput = odometry_->getPreProcessedCloud();
      o3d_slam::publishCloud(odomInput, o3d_slam::frames::rangeSensorFrame, toRos(scanToScanTimestamp), odometryInputPub_);
    }

    const Time scanToMapTimestamp = latestScanToMapRefinementTimestamp_;
    if (isTimeValid(scanToMapTimestamp)) {
      publishDenseMap(scanToMapTimestamp);
      publishMaps(scanToMapTimestamp);
    }

    //executor_->spin_some();
    r.sleep();
  }
}

void SlamWrapperRos::loadParametersAndInitialize() {
  odometryInputPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("odom_input", 1);
  mappingInputPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("mapping_input", 1);
  assembledMapPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("assembled_map", 1);
  denseMapPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("dense_map", 1);

  submapsPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("submaps", 1);
  submapOriginsPub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("submap_origins", 1);

  saveMapSrv_ = nh_->create_service<open3d_slam_msgs::srv::SaveMap>(
    "save_map",
    std::bind(&SlamWrapperRos::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  saveSubmapsSrv_ = nh_->create_service<open3d_slam_msgs::srv::SaveSubmaps>(
    "save_submaps",
    std::bind(&SlamWrapperRos::saveSubmapsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  scan2scanTransformPublisher_ = nh_->create_publisher<geometry_msgs::msg::TransformStamped>("scan2scan_transform", 1);
  scan2scanOdomPublisher_ = nh_->create_publisher<nav_msgs::msg::Odometry>("scan2scan_odometry", 1);
  scan2mapTransformPublisher_ = nh_->create_publisher<geometry_msgs::msg::TransformStamped>("scan2map_transform", 1);
  scan2mapOdomPublisher_ = nh_->create_publisher<nav_msgs::msg::Odometry>("scan2map_odometry", 1);

  folderPath_ = ament_index_cpp::get_package_share_directory("open3d_slam_ros") + "/data/";
  nh_->declare_parameter("map_saving_folder", folderPath_);
  mapSavingFolderPath_ = nh_->get_parameter("map_saving_folder").as_string();

  //nh_->declare_parameter("parameter_folder_path", "");
  const std::string paramFolderPath = nh_->get_parameter("parameter_folder_path").as_string();
  //nh_->declare_parameter("parameter_filename", "");
  const std::string paramFilename = nh_->get_parameter("parameter_filename").as_string();
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params_);

  BASE::loadParametersAndInitialize();
}

void SlamWrapperRos::saveMapCallback(const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Request> req, const std::shared_ptr<open3d_slam_msgs::srv::SaveMap::Response> res) {
  const bool savingResult = saveMap(mapSavingFolderPath_);
  res->status_message = savingResult ? "Map saved to: " + mapSavingFolderPath_ : "Error while saving map";
}
void SlamWrapperRos::saveSubmapsCallback(const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Request> req, const std::shared_ptr<open3d_slam_msgs::srv::SaveSubmaps::Response> res) {
  const bool savingResult = saveSubmaps(mapSavingFolderPath_);
  res->status_message = savingResult ? "Submaps saved to: " + mapSavingFolderPath_ : "Error while saving submaps";
}

void SlamWrapperRos::publishMapToOdomTf(const Time& time) {
  const rclcpp::Time timestamp = toRos(time);
  o3d_slam::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame, tfBroadcaster_.get());
  o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp, mapFrame, "raw_rs_o3d", tfBroadcaster_.get());
}

void SlamWrapperRos::publishDenseMap(const Time& time) {
  if (denseMapVisualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_) {
    return;
  }
  const auto denseMap = mapper_->getActiveSubmap().getDenseMapCopy();  // copy
  const rclcpp::Time timestamp = toRos(time);
  o3d_slam::publishCloud(denseMap.toPointCloud(), o3d_slam::frames::mapFrame, timestamp, denseMapPub_);
}

void SlamWrapperRos::publishMaps(const Time& time) {
  if (visualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_ && !isVisualizationFirstTime_) {
    return;
  }

  const rclcpp::Time timestamp = toRos(time);
  {
    PointCloud map = mapper_->getAssembledMapPointCloud();
    voxelize(params_.visualization_.assembledMapVoxelSize_, &map);
    o3d_slam::publishCloud(map, o3d_slam::frames::mapFrame, timestamp, assembledMapPub_);
  }
  o3d_slam::publishCloud(mapper_->getPreprocessedScan(), o3d_slam::frames::rangeSensorFrame, timestamp, mappingInputPub_);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), o3d_slam::frames::mapFrame, timestamp, submapOriginsPub_);
  if (submapsPub_->get_subscription_count() > 0) {
    open3d::geometry::PointCloud cloud;
    o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
    voxelize(params_.visualization_.submapVoxelSize_, &cloud);
    o3d_slam::publishCloud(cloud, o3d_slam::frames::mapFrame, timestamp, submapsPub_);
  }

  visualizationUpdateTimer_.reset();
  isVisualizationFirstTime_ = false;
}

}  // namespace o3d_slam
