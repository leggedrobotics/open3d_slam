//
// Created by peyschen on 08/06/23.
//

#include "realtime_meshing_ros/RealtimeMeshingRos.h"
#include <eigen_conversions/eigen_msg.h>
#include <open3d_conversions/open3d_conversions.h>
#include "realtime_meshing/helpers.h"

void RealtimeMeshingRos::initializeRos() {
  auto parameterFile = nh_->param<std::string>("parameter_file", "");
  if (!parameterFile.empty()) {
    loadParameters(parameterFile, params_);
  }
  auto odometryTopic = nh_->param<std::string>("odometry_topic", "odom_input");
  auto pointcloudTopic = nh_->param<std::string>("pointcloud_topic", "pointcloud_input");
  odomSub_ = nh_->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &RealtimeMeshingRos::odometryCallback, this);
  pointcloudSub_ = nh_->subscribe<sensor_msgs::PointCloud2>(pointcloudTopic, 1, &RealtimeMeshingRos::pointCloudCallback, this);
  mapPub_ = nh_->advertise<open3d_slam_msgs::PolygonMesh>("mesh", 1, true);
  mesher_ = std::make_unique<Mesher>();
  mesher_->setParameters(params_);
  meshingThread_ = std::thread([&]() { meshingWorker(); });
  publishThread_ = std::thread([&]() { publishWorker(); });

  ROS_INFO("Initialized!");
}

void RealtimeMeshingRos::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  Eigen::Isometry3d tf;
  tf::poseMsgToEigen(msg->pose.pose, tf);
  tfBuffer_.push(o3d_slam::fromUnix(msg->header.stamp.toNSec()), tf);
}
void RealtimeMeshingRos::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  open3d::geometry::PointCloud pointCloudRaw;
  open3d_conversions::rosToOpen3d(msg, pointCloudRaw);
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(params_.preRotateRoll_, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(params_.preRotatePitch_, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(params_.preRotateYaw_, Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d tfMatrix = Eigen::Matrix4d::Identity();
  tfMatrix.block<3, 3>(0, 0) = rot;
  pointCloudRaw = pointCloudRaw.Transform(tfMatrix);
  pointCloudRaw = pointCloudRaw.RemoveNonFinitePoints();
  pointCloudBuffer_->push({o3d_slam::fromUnix(msg->header.stamp.toNSec()), pointCloudRaw});
}

void RealtimeMeshingRos::meshingWorker() {
  size_t numPointcloudsAdded = 0;
  while (workersShouldRun_) {
    if (pointCloudBuffer_->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      continue;
    }
    if (tfBuffer_.empty()) {
      continue;
    }
    auto stampedCloud = pointCloudBuffer_->pop();
    o3d_slam::Transform rangeSensorToMap;
    if (!tfBuffer_.has(stampedCloud.time_)) {
      ROS_WARN("tfBuffer did not have transform!");
      rangeSensorToMap = tfBuffer_.latest_measurement().transform_;
    } else {
      rangeSensorToMap = tfBuffer_.lookup(stampedCloud.time_);
    }

    mesher_->addNewPointCloud(stampedCloud.cloud_, rangeSensorToMap);
    numPointcloudsAdded++;
    if (numPointcloudsAdded % 2 == 0) {
      mesher_->mesh();
    }
    if (numPointcloudsAdded == 10) {
      numPointcloudsAdded = 0;
    }
  }
}

void RealtimeMeshingRos::publishWorker() {
  auto nextExecution = std::chrono::steady_clock::now();
  while (workersShouldRun_) {
    std::this_thread::sleep_until(nextExecution);
    open3d_slam_msgs::PolygonMesh meshMsg;
    auto mesh = mesher_->getMesh();
    if (!mesh.IsEmpty()) {
      Eigen::Isometry3d rangeSensor = mesher_->getActiveMeshMap()->getMapToRange();
      Eigen::Vector3d rangeTranslation = rangeSensor.translation();
      auto top = rangeTranslation.z() + params_.meshCropHeight_;
      auto bottom = top - 100;
      auto center = (top + bottom)/2.0;

      Eigen::Vector3d bBoxCenter{rangeTranslation.x(), rangeTranslation.y(), center};
      Eigen::Vector3d extent{100,100,100};
      open3d::geometry::OrientedBoundingBox cropBox(bBoxCenter,rangeSensor.rotation(), extent);
      mesh = *mesh.Crop(cropBox);
      if(mesh.IsEmpty()){
          return;
      }
      open3d_conversions::open3dToRos(mesh, "map", meshMsg);
      meshMsg.header.frame_id = "map";
      mapPub_.publish(meshMsg);
    }
    nextExecution += std::chrono::milliseconds(200);  // Publish mesh at 5Hz
  }
}

RealtimeMeshingRos::~RealtimeMeshingRos() {
  workersShouldRun_ = false;
  if (meshingThread_.joinable()) {
    meshingThread_.join();
    ROS_INFO("Joined meshing thread!");
  }

  if (publishThread_.joinable()) {
    publishThread_.join();
    ROS_INFO("Joined publishing thread!");
  }
}
