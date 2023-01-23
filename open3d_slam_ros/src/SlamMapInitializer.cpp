/*
 * SlamMapInitializer.cpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include "open3d/io/PointCloudIO.h"

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"

#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"


namespace o3d_slam {

const double sqrt2= std::sqrt(2.0);

SlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros::NodeHandlePtr nh) :
  server_("initialization_pose"),
  slamPtr_(slamPtr),
  nh_(nh){
}

SlamMapInitializer::~SlamMapInitializer(){
	if (initWorker_.joinable()) {
		initWorker_.join();
		std::cout << "Joined mapInitializer worker \n";
	}
}

void SlamMapInitializer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
  Eigen::Isometry3d init_transform;
	tf::poseMsgToEigen(msg.pose.pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}
bool SlamMapInitializer::initSlamCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
  return true;
}

void SlamMapInitializer::initialize(const MapInitializingParameters &params) {
  mapInitializerParams_ = params;
  PointCloud raw_map;
  initialized_.store(false);

  std::cout << "Loading pointloud from: " << mapInitializerParams_.pcdFilePath_ << "\n";
  if (!open3d::io::ReadPointCloud(mapInitializerParams_.pcdFilePath_, raw_map))
	{
		std::cerr << "[Error] Initialization pointcloud not loaded" << std::endl;
  }

  Transform initPose = params.initialPose_;
  slamPtr_->setInitialMap(raw_map);
  slamPtr_->setInitialTransform(initPose.matrix());
  std::cout << "init pose: " << asString(initPose) << std::endl;
  if (params.isInitializeInteractively_){
    initInteractiveMarker();
    initPoseSub_ = nh_->subscribe("/initialpose", 1, &SlamMapInitializer::initialPoseCallback, this);
    initializeSlamSrv_ = nh_->advertiseService("initialize_slam", &SlamMapInitializer::initSlamCallback,this);
    cloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("aligned_cloud_preview",1);
		std::string cloudTopic = nh_->param<std::string>("cloud_topic", "");
		std::cout << "Initializer subscribing to " << cloudTopic << std::endl;
    cloudSub_ = nh_->subscribe(cloudTopic, 1, &SlamMapInitializer::pointcloudCallback, this);
    initWorker_ = std::thread([this]() {
  		initializeWorker();
  	});
    std::cout << "started interactive marker worker \n";
  } else {
		std::cout << "Finished setting initial map! \n";
  }

}
void SlamMapInitializer::initializeWorker() {
	ros::Rate r(20);
	const bool isMergeScansIntoMap = slamPtr_->getMapperParameters().isMergeScansIntoMap_;
	slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = false;
	slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = true;
	while (ros::ok() && !initialized_.load()) {
		ros::spinOnce();
		r.sleep();
	}
  slamPtr_->getMapperParametersPtr()->isMergeScansIntoMap_ = isMergeScansIntoMap;
  // TODO: this is a hack allows to merge scans into map that is significantly smaller than a scan but requires refinement.
  usleep(1000000);
  slamPtr_->getMapperParametersPtr()->isIgnoreMinRefinementFitness_ = false;
  std::cout << "Finished setting initial map! \n";
}

void SlamMapInitializer::initInteractiveMarker() {
  menuHandler_.insert("Initialize SLAM map", boost::bind(&SlamMapInitializer::initMapCallback, this, _1));
  menuHandler_.insert("Set Pose", boost::bind(&SlamMapInitializer::setPoseCallback, this, _1));

  auto interactiveMarker = createInteractiveMarker();
  interactiveMarkerName_ = interactiveMarker.name;
  server_.insert(interactiveMarker);
  menuHandler_.apply(server_, interactiveMarker.name);
  server_.applyChanges();
}

void SlamMapInitializer::setPoseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
  Eigen::Isometry3d init_transform;
	tf::poseMsgToEigen(msg->pose, init_transform);
  std::cout << "Initial Pose \n" << asString(init_transform) << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
}

void SlamMapInitializer::initMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
  std::cout << "Map initialized" << std::endl;
  initialized_.store(true);
}

void SlamMapInitializer::pointcloudCallback(const sensor_msgs::PointCloud2 &msg){
	visualization_msgs::InteractiveMarker marker;
	server_.get(interactiveMarkerName_, marker);
  Eigen::Isometry3d markerPose;
	tf::poseMsgToEigen(	marker.pose, markerPose);
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	cloud.Transform(markerPose.matrix());
  o3d_slam::publishCloud(cloud, o3d_slam::frames::mapFrame, marker.header.stamp, cloudPub_);
}

visualization_msgs::InteractiveMarker SlamMapInitializer::createInteractiveMarker() const {
	visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = mapInitializerParams_.frameId_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Initial Pose";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to see options";
	tf::poseEigenToMsg(mapInitializerParams_.initialPose_, interactiveMarker.pose);

  // create a mesh marker
  const auto arrowMarker = []() {
    visualization_msgs::Marker marker;
		marker.type = visualization_msgs::Marker::ARROW;
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

  // create a non-interactive control which contains the mesh
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(arrowMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the mesh
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 1.0 / sqrt2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0;
  control.orientation.y = 1.0 / sqrt2;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1.0 / sqrt2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1.0 / sqrt2;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);
  
  return interactiveMarker;
}

} // namespace o3d_slam

