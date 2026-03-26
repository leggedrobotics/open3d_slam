/*
 * SlamMapInitializer.cpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#include "open3d_slam_ros/SlamMapInitializer.hpp"

#include <open3d/io/PointCloudIO.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

namespace {

const double sqrt2 = std::sqrt(2.0);

rclcpp::QoS makeCloudSubscriptionQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth), rmw_qos_profile_sensor_data);
}

}  // namespace

SlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, rclcpp::Node::SharedPtr node)
    : server_("initialization_pose", node), slamPtr_(std::move(slamPtr)), node_(std::move(node)) {}

SlamMapInitializer::~SlamMapInitializer() {
  if (initWorker_.joinable()) {
    initWorker_.join();
    std::cout << "Joined mapInitializer worker \n";
  }
}

void SlamMapInitializer::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  Eigen::Isometry3d init_transform;
  tf2::fromMsg(msg->pose.pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}

void SlamMapInitializer::initSlamCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  (void)req;
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
  res->success = true;
  res->message = "Map initialized";
}

void SlamMapInitializer::initialize(const MapInitializingParameters& params) {
  mapInitializerParams_ = params;
  PointCloud raw_map;
  initialized_.store(false);

  std::cout << "Loading pointloud from: " << mapInitializerParams_.pcdFilePath_ << "\n";
  if (!open3d::io::ReadPointCloud(mapInitializerParams_.pcdFilePath_, raw_map)) {
    throw std::runtime_error("Initialization pointcloud not loaded: " + mapInitializerParams_.pcdFilePath_);
  }

  slamPtr_->setInitialMap(raw_map);
  slamPtr_->setInitialTransform(params.initialPose_.matrix());
  std::cout << "init pose: " << asString(params.initialPose_) << std::endl;
  if (!params.isInitializeInteractively_) {
    std::cout << "Finished setting initial map! \n";
    return;
  }

  initInteractiveMarker();
  initPoseSub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1), std::bind(&SlamMapInitializer::initialPoseCallback, this, std::placeholders::_1));
  initializeSlamSrv_ = node_->create_service<std_srvs::srv::Trigger>(
      "initialize_slam", std::bind(&SlamMapInitializer::initSlamCallback, this, std::placeholders::_1, std::placeholders::_2));
  cloudPub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud_preview", rclcpp::QoS(1));
  const std::string cloudTopic = tryGetParam<std::string>("cloud_topic", *node_);
  std::cout << "Initializer subscribing to " << cloudTopic << std::endl;
  cloudSub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloudTopic, makeCloudSubscriptionQos(1), std::bind(&SlamMapInitializer::pointcloudCallback, this, std::placeholders::_1));
  initWorker_ = std::thread([this]() { initializeWorker(); });
  std::cout << "started interactive marker worker \n";
}

void SlamMapInitializer::initializeWorker() {
  rclcpp::Rate rate(20.0);
  const bool isMergeScansIntoMap = slamPtr_->getMapperParameters().isMergeScansIntoMap_;
  slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = false;
  slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = true;
  while (rclcpp::ok() && !initialized_.load()) {
    rate.sleep();
  }
  slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = isMergeScansIntoMap;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = false;
  std::cout << "Finished setting initial map! \n";
}

void SlamMapInitializer::initInteractiveMarker() {
  menuHandler_.insert("Initialize SLAM map",
                      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) { initMapCallback(msg); });
  menuHandler_.insert("Set Pose",
                      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) { setPoseCallback(msg); });

  auto interactiveMarker = createInteractiveMarker();
  interactiveMarkerName_ = interactiveMarker.name;
  server_.insert(interactiveMarker);
  menuHandler_.apply(server_, interactiveMarker.name);
  server_.applyChanges();
}

void SlamMapInitializer::setPoseCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) {
  Eigen::Isometry3d init_transform;
  tf2::fromMsg(msg->pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}

void SlamMapInitializer::initMapCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& msg) {
  (void)msg;
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
}

void SlamMapInitializer::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  visualization_msgs::msg::InteractiveMarker marker;
  server_.get(interactiveMarkerName_, marker);
  Eigen::Isometry3d markerPose;
  tf2::fromMsg(marker.pose, markerPose);
  open3d::geometry::PointCloud cloud;
  open3d_conversions::rosToOpen3d(msg, cloud, false);
  cloud.Transform(markerPose.matrix());
  o3d_slam::publishCloud(cloud, slamPtr_->getFrames().mapFrame, rclcpp::Time(marker.header.stamp), cloudPub_);
}

visualization_msgs::msg::InteractiveMarker SlamMapInitializer::createInteractiveMarker() const {
  visualization_msgs::msg::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = mapInitializerParams_.frameId_;
  const int64_t now_nanoseconds = node_->now().nanoseconds();
  interactiveMarker.header.stamp.sec = static_cast<int32_t>(now_nanoseconds / 1000000000ll);
  interactiveMarker.header.stamp.nanosec = static_cast<uint32_t>(now_nanoseconds % 1000000000ll);
  interactiveMarker.name = "Initial Pose";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to see options";
  interactiveMarker.pose = tf2::toMsg(mapInitializerParams_.initialPose_);

  const auto arrowMarker = []() {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.pose.position.x = -0.25;
    return marker;
  }();

  visualization_msgs::msg::InteractiveMarkerControl boxControl;
  boxControl.always_visible = true;
  boxControl.markers.push_back(arrowMarker);
  boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  interactiveMarker.controls.push_back(boxControl);

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 1.0 / sqrt2;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0 / sqrt2;
  control.orientation.z = 0.0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 1.0 / sqrt2;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

}  // namespace o3d_slam
