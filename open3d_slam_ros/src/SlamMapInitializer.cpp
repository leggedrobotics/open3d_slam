/*
 * SlamMapInitializer.cpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>



namespace o3d_slam {

SlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros::NodeHandlePtr nh) :
  server_("initialization_pose"),
  slamPtr_(slamPtr),
  nh_(nh){
}

void SlamMapInitializer::initialize() {
  PointCloud raw_map;

  
	frameId_ = nh_->param<std::string>("map_frame_id", "");
	meshResourcePath_ = nh_->param<std::string>("map_mesh_path", "");
	pcdFilePath_ = nh_->param<std::string>("map_pointcloud_path", "");

  initialized_.store(false);

  initInterectiveMarker();
  
  if (!open3d::io::ReadPointCloud(pcdFilePath_, raw_map))
	{
		std::cerr << "[Error] Initialization pointcloud not loaded" << std::endl;
  }
  
  slamPtr_->setInitialMap(raw_map);
  
  while (!initialized_.load())
  {
    ros::spinOnce();
  }
  
}

void SlamMapInitializer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  slamPtr_->addRangeScan(cloud, timestamp);
}

void SlamMapInitializer::initInterectiveMarker() {
  menuHandler_.insert("Initialize SLAM", boost::bind(&SlamMapInitializer::interactiveMarkerCallback, this, _1));

  auto interactiveMarker = createInteractiveMarker();

  server_.insert(interactiveMarker);
  menuHandler_.apply(server_, interactiveMarker.name);
  server_.applyChanges();
}

void SlamMapInitializer::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
  Eigen::Affine3d init_transform;
	geometry_msgs::Pose pose = msg->pose;
	tf::poseMsgToEigen(pose, init_transform);
	
  slamPtr_->setInitialTransform(init_transform.matrix());
  initialized_.store(true);
}

visualization_msgs::InteractiveMarker SlamMapInitializer::createInteractiveMarker() const {
	visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = frameId_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Initial Pose";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = -1.0;
  interactiveMarker.pose.position.y = -3.0;
  interactiveMarker.pose.position.z = -1.0;
  interactiveMarker.pose.orientation.x = 0.0;
  interactiveMarker.pose.orientation.y = 0.0;
  interactiveMarker.pose.orientation.z = 0.0;
  interactiveMarker.pose.orientation.w = 1.0;

  // create a grey box marker
  const auto meshMarker = [&meshResourcePath_ = meshResourcePath_]() {
    visualization_msgs::Marker marker;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.mesh_resource = meshResourcePath_;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(meshMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);
  
  return interactiveMarker;
}


} // namespace o3d_slam

