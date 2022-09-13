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
#include "open3d_slam_ros/Parameters.hpp"
#include "open3d_slam/SlamWrapper.hpp"
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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
	visualization_msgs::InteractiveMarker createInteractiveMarker() const;
  
	interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;
	std::shared_ptr<SlamWrapper> slamPtr_;
	std::atomic_bool initialized_;
	MapInitializingParameters mapInitializerParams_;
	ros::NodeHandlePtr nh_;
	std::thread initWorker_;

};

} // namespace o3d_slam
