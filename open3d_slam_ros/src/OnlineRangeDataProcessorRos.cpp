/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) :
		BASE(nh), tfListener_(tfBuffer_){

}

void OnlineRangeDataProcessorRos::initialize() {
	initCommonRosStuff();
	slam_ = std::make_shared<SlamWrapperRos>(nh_);
	slam_->loadParametersAndInitialize();
	
}

void OnlineRangeDataProcessorRos::startProcessing() {
	slam_->startWorkers();
	cloudSubscriber_ = nh_->subscribe(cloudTopic_, 1, &OnlineRangeDataProcessorRos::cloudCallback,this);
	
	posePublishingTimer_ = nh_->createTimer(ros::Duration(0.01), &OnlineRangeDataProcessorRos::posePublisherTimerCallback, this);


	scan2scanTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("tt_scan2scan_transform", 1, true);
	scan2scanOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2scan_odometry", 1, true);
	scan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("tt_scan2map_transform", 1, true);
	scan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry", 1, true);
	scan2mapOdomPriorPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry_prior", 1, true);

	consolidatedScan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("cons_tt_scan2map_odometry", 1, true);

	ros::spin();
	slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {

	if(slam_->params_.odometry_.overwriteWithTf){
	// Get and set the point cloud registration prior from the tf.
	getAndSetTfPrior(timestamp);
	}

	// Add the scan measurement
	slam_->addRangeScan(cloud, timestamp);
	
  // Stop publishing the same cloud again,
  //o3d_slam::publishCloud(cloud, o3d_slam::frames::rangeSensorFrame, toRos(timestamp), rawCloudPub_);

}

geometry_msgs::TransformStamped OnlineRangeDataProcessorRos::getTransformMsg(const Transform &T, const ros::Time &timestamp, const std::string parent, const std::string child){
		geometry_msgs::TransformStamped transformMsg = o3d_slam::toRos(T.matrix(), timestamp,  parent,  child);
		return transformMsg;
}

nav_msgs::Odometry OnlineRangeDataProcessorRos::getOdomMsg(const geometry_msgs::TransformStamped &transformMsg){
	nav_msgs::Odometry odomMsg;
	odomMsg.header = transformMsg.header;
	odomMsg.child_frame_id = transformMsg.child_frame_id;
	odomMsg.pose.pose.orientation = transformMsg.transform.rotation;
	odomMsg.pose.pose.position.x = transformMsg.transform.translation.x;
	odomMsg.pose.pose.position.y = transformMsg.transform.translation.y;
	odomMsg.pose.pose.position.z = transformMsg.transform.translation.z;
	return odomMsg;
}

void OnlineRangeDataProcessorRos::posePublisherTimerCallback(const ros::TimerEvent&){

	if(!slam_->isNewScan2MapRefinementAvailable_ || !slam_->params_.odometry_.overwriteWithTf){
		return;
	}


	// Runs 100hz
	Time latestTransformTime; 
	Transform mapToRangeSensorTransform;

	// Thread safety, memory access

	{
		std::lock_guard<std::mutex> methodLock(posePublishingMutex_);

		mapToRangeSensorTransform = slam_->latestScanToMapRefinedPose_;
		latestTransformTime = slam_->latestScanToMapRefinementTimestamp_;
		slam_->isNewScan2MapRefinementAvailable_=false;
	}



	Transform tfQueriedodometry;


	if(o3d_slam::lookupTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, ros::Time::now(), tfBuffer_, tfQueriedodometry))
	{
		// The odometry prior at the time of scan2map refinement
		Transform latestOdomToRangeSensorInBuffer_ = slam_->getLatestOdometryPose();

		// The odometry motion calculated between the odometry prior at the time opf Scan2map and the current odometry pose.
		// Hypothesis: if scan2map refinement takes ages, this difference might be big.
		// Otherwise expected to be near identical.
		const Transform odometryMotion = latestOdomToRangeSensorInBuffer_.inverse()*tfQueriedodometry;
		
		// We propagate the map2lidar refined pose. This is no longer the registered pose. Shouldnt be used for point cloud accumulation.
		Transform consolidatedMapToRangeSensor = mapToRangeSensorTransform * odometryMotion ;

		// The non-consolidated mapToOdom calculation. Expected to be delayed.
		Transform mapToOdom = mapToRangeSensorTransform * tfQueriedodometry.inverse();

		// The consolidated mapToOdom calculation. Expected to be more accurate if scan2map registration takes long.
		Transform mapToOdomConsolidated = consolidatedMapToRangeSensor * tfQueriedodometry.inverse();

		// Publish the slow but accurate map2odom.  Odom is the parent, map is the child.
		o3d_slam::publishTfTransform(mapToOdom.matrix().inverse(), ros::Time::now(), o3d_slam::frames::odomFrame, o3d_slam::frames::mapFrame,
				tfBroadcaster_.get());

		// Publish the fast but propagated map2odom.  Odom is the parent, map_consolidated is the child.
		o3d_slam::publishTfTransform(mapToOdomConsolidated.matrix().inverse(), ros::Time::now(), o3d_slam::frames::odomFrame, "map_consolidated",
				tfBroadcaster_.get());

		geometry_msgs::TransformStamped consolidatedScan2maptransformMsg = getTransformMsg(consolidatedMapToRangeSensor, ros::Time::now(), "map_consolidated",  o3d_slam::frames::rangeSensorFrame);
		nav_msgs::Odometry consolidatedScan2mapodomMsg = getOdomMsg(consolidatedScan2maptransformMsg);
		//publishIfSubscriberExists(consolidatedScan2maptransformMsg, consolidatedScan2mapTransformPublisher_);
		publishIfSubscriberExists(consolidatedScan2mapodomMsg, consolidatedScan2mapOdomPublisher_);

	}else{
		ROS_WARN("TF did not provide prior. Tf based transforms not published.");
	}


	// The getTransformMsg function has hard coded frame care.
	geometry_msgs::TransformStamped scan2maptransformMsg = getTransformMsg(mapToRangeSensorTransform, ros::Time::now(), o3d_slam::frames::mapFrame,  o3d_slam::frames::rangeSensorFrame);
	nav_msgs::Odometry scan2mapodomMsg = getOdomMsg(scan2maptransformMsg);
	publishIfSubscriberExists(scan2maptransformMsg, scan2mapTransformPublisher_);
	publishIfSubscriberExists(scan2mapodomMsg, scan2mapOdomPublisher_);

};


void OnlineRangeDataProcessorRos::getAndSetTfPrior(const Time &queryTime){

	if(!isTimeValid(queryTime)){
		return;
	}
		
	if(slam_->checkIfodomToRangeSensorBufferHasTime(queryTime)){
		return;
	}

	if (!tfBuffer_.canTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, o3d_slam::toRos(queryTime), ros::Duration(0.2))) {
		ROS_WARN("Requested transform from frames ODOM to Lidar with target timestamp lidar cannot be found.");
	}else{
		// Goal: Set the initial guess of the registration.
		Transform tfQueriedodometry;
		if(o3d_slam::lookupTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, o3d_slam::toRos(queryTime), tfBuffer_, tfQueriedodometry))
		{
			//todo
			// Possibly put a exception block and revert to LiDAR odometry pose as prior.
		}
		std::cout << "Latest TF query time: " << o3d_slam::toSecondsSinceFirstMeasurement(queryTime) << std::endl;
		
		slam_->addOdometryPrior(queryTime, tfQueriedodometry);
		}
	

}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	const Time timestamp = fromRos(msg->header.stamp);
	accumulateAndProcessRangeData(cloud, timestamp);
}



} // namespace o3d_slam

