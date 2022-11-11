/*
 * SlamMapInitializer.hpp
 *
 *  Created on: Jun 16, 2022
 *      Author: lukaszpi
 */

#pragma once

#include <memory>
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/SlamWrapper.hpp"
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_srvs/Trigger.h>

namespace o3d_slam {

class SlamMapInitializer{

public:
	SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros::NodeHandlePtr nh);
	~SlamMapInitializer();
	
	void initialize(const MapInitializingParameters &params);

private:
	void initInteractiveMarker();
	void setPoseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
	void initMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
	void initializeWorker();
	void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
	bool initSlamCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	visualization_msgs::InteractiveMarker createInteractiveMarker() const;
	void pointcloudCallback(const sensor_msgs::PointCloud2 &msg);
  
	interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;
	std::shared_ptr<SlamWrapper> slamPtr_;
	std::atomic_bool initialized_;
	MapInitializingParameters mapInitializerParams_;
	ros::NodeHandlePtr nh_;
	std::thread initWorker_;
	ros::ServiceServer initializeSlamSrv_;
	ros::Subscriber initPoseSub_;
	std::string interactiveMarkerName_;
	ros::Subscriber cloudSub_;
	ros::Publisher cloudPub_;

};

} // namespace o3d_slam
