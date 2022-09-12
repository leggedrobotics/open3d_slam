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

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"

#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam_ros/SlamMapInitializer.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"


namespace o3d_slam {

SlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros::NodeHandlePtr nh) :
  server_("initialization_pose"),
  slamPtr_(slamPtr),
  nh_(nh){
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

  {
    std::cout << "Calculating normals for the initial map!" << std::endl;
    const auto &p = slamPtr_->getMapperParameters();
    const int knn = p.scanMatcher_.kNNnormalEstimation_;
    Timer t("initial map normal estimation");
    if (!raw_map.HasNormals() && p.scanMatcher_.icpObjective_ == o3d_slam::IcpObjective::PointToPlane) {
    		estimateNormals(p.scanMatcher_.kNNnormalEstimation_, &raw_map);
    		raw_map.NormalizeNormals(); //todo, dunno if I need this
    	}
    std::cout << "Normals estimated! \n" << std::endl;
  }
  Transform initPose;
	tf::poseMsgToEigen(params.initialMarkerPose_, initPose);
  slamPtr_->setInitialTransform(initPose.matrix());
  std::cout << "init pose: " << asString(initPose) << std::endl;
  if (params.isUseInteractiveMarker_){
    initInterectiveMarker();
    ros::Rate r(100);
    while (!initialized_.load())
    {
      ros::spinOnce();
      r.sleep();
    }
  }
  slamPtr_->setInitialMap(raw_map);
  
}

void SlamMapInitializer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
  const Time timestamp = fromRos(msg->header.stamp);
  slamPtr_->addRangeScan(cloud, timestamp);
}

void SlamMapInitializer::initInterectiveMarker() {
  menuHandler_.insert("Initialize SLAM map", boost::bind(&SlamMapInitializer::interactiveMarkerCallback, this, _1));

  auto interactiveMarker = createInteractiveMarker();

  server_.insert(interactiveMarker);
  menuHandler_.apply(server_, interactiveMarker.name);
  server_.applyChanges();
}

void SlamMapInitializer::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
  Eigen::Affine3d init_transform;
	tf::poseMsgToEigen(msg->pose, init_transform);
  std::cout << "Initial Pose \n" << msg->pose << std::endl;
  slamPtr_->setInitialTransform(init_transform.matrix());
  initialized_.store(true);
}

visualization_msgs::InteractiveMarker SlamMapInitializer::createInteractiveMarker() const {
	visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = mapInitializerParams_.frameId_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Initial Pose";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to initialize slam map";
  interactiveMarker.pose = mapInitializerParams_.initialMarkerPose_;

  // create a mesh marker
  const auto meshMarker = [&meshFilePath_ = mapInitializerParams_.meshFilePath_]() {
    visualization_msgs::Marker marker;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.mesh_resource = meshFilePath_;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the mesh
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(meshMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the mesh
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

