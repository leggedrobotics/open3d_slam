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
#include "open3d_slam/output.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) :
		BASE(nh), tfListener_(tfBuffer_){
	tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());

}

void OnlineRangeDataProcessorRos::initialize() {
	initCommonRosStuff();
	slam_ = std::make_shared<SlamWrapperRos>(nh_);
	slam_->loadParametersAndInitialize();
	trajectoryAlignmentHandlerPtr_ = std::make_shared<o3d_slam::TrajectoryAlignmentHandler>();
	trajectoryAlignmentHandlerPtr_->initHandler();
	
}

void OnlineRangeDataProcessorRos::startProcessing() {
	slam_->startWorkers();
	// Multithreaded spinner.
	//ros::MultiThreadedSpinner spinner(12);
	
	// Main topic subscriber
	cloudSubscriber_ = nh_->subscribe(cloudTopic_, 1, &OnlineRangeDataProcessorRos::cloudCallback,this);

	// Relavent pose subscriber
	priorPoseSubscriber_ = nh_->subscribe<nav_msgs::Odometry>("/state_estimator/odometry", 1000, &OnlineRangeDataProcessorRos::poseStampedPriorCallback, this, ros::TransportHints().tcpNoDelay());
	
	// Timers to publish data
	posePublishingTimer_ = nh_->createTimer(ros::Duration(0.01), &OnlineRangeDataProcessorRos::posePublisherTimerCallback, this);
	registeredEnuCloudPublisherTimer_ = nh_->createTimer(ros::Duration(0.025), &OnlineRangeDataProcessorRos::registeredCloudPublisherCallback, this);

	// Service to call to get the transformation.
	alignWithWorld_ = nh_->advertiseService("align_world", &OnlineRangeDataProcessorRos::alignWithWorld, this);

	// Path publishers (When GNSS is available)
	lidarPathPublisher_ = nh_->advertise<nav_msgs::Path>("/lidarPath", 1);
	gnssPathPublisher_ = nh_->advertise<nav_msgs::Path>("/gnssPath", 1);

	lidarPoseInMapPathPublisher_ = nh_->advertise<nav_msgs::Path>("/lidarPoseInMapPath", 1);
	consolidatedLidarPoseInMapPathPublisher_ = nh_->advertise<nav_msgs::Path>("/consolidatedLidarPoseInMapPath", 1);

	// Publishers for the poses
	consolidatedScan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("const_tt_scan2map_transform", 1, true);
	scan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("tt_scan2map_transform", 1, true);
	scan2mapOdometryPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry", 1, true);
	scan2mapOdometryPriorPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry_prior", 1, true);

	// Experimental
	consolidatedScan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("cons_tt_scan2map_odometry", 1, true);

	// Registered cloud
	registeredCloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("registered_cloud", 1, true);

	//spinner.spin();

	ros::spin();
	slam_->stopWorkers();
}

bool OnlineRangeDataProcessorRos::alignWithWorld(std_srvs::EmptyRequest& /*req*/, std_srvs::EmptyResponse& /*res*/) {

	auto tLidar = trajectoryAlignmentHandlerPtr_->getLidarTrajectory();
	auto tGnss = trajectoryAlignmentHandlerPtr_->getGnssTrajectory();

	if((tGnss.size()<5) || (tLidar.size()<5)){

		std::cout << " NOT ENOUGH MEASUREMENTS TO ALIGN with world "<< std::endl;
		return false;
	}

	if(!trajectoryAlignmentHandlerPtr_->alignTrajectories(mapToWorld_)){

		std::cout << " AlignTrajectoriesFailed "<< std::endl;
		return false;
	}

	nav_msgs::Path path;
	geometry_msgs::PoseStamped poseStamped;
	path.header.frame_id = "enu";
	poseStamped.header.frame_id = "enu";

	for (auto pose : tLidar) {
		Eigen::Vector4d transformablePoint;
		transformablePoint.row(0).col(0) = pose.second.row(0).col(0);
		transformablePoint.row(1).col(0) = pose.second.row(1).col(0);
		transformablePoint.row(2).col(0) = pose.second.row(2).col(0);
		transformablePoint.row(3).col(0) << 1.0;

		// For timo: this is where you might need to optimize. Currently super naive, point-wise multiplication. Single threaded.
		transformablePoint =  mapToWorld_.matrix() * transformablePoint;

		poseStamped.header.stamp.sec = int(int(pose.first) / 1e9);
		poseStamped.header.stamp.nsec = int(int(pose.first) % int(1e9));
		poseStamped.pose.position.x = transformablePoint.row(0).col(0).value();
		poseStamped.pose.position.y = transformablePoint.row(1).col(0).value();
		poseStamped.pose.position.z = transformablePoint.row(2).col(0).value();

		path.poses.push_back(poseStamped);
	}

	lidarPathPublisher_.publish(path);

	path.poses.clear();
	path.header.frame_id = "enu";
	poseStamped.header.frame_id = "enu";
	for (auto pose : tGnss) {
		poseStamped.header.stamp.sec = int(int(pose.first) / 1e9);
		poseStamped.header.stamp.nsec = int(int(pose.first) % int(1e9));
		poseStamped.pose.position.x = pose.second(0);
		poseStamped.pose.position.y = pose.second(1);
		poseStamped.pose.position.z = pose.second(2);
		path.poses.push_back(poseStamped);
	}
	gnssPathPublisher_.publish(path);

	slam_->setMapToEnu(mapToWorld_);
	std::string userName = getenv("USERNAME");
	std::string folderPath = "/home/" + userName +"/";
	const bool savingResult = slam_->transformSaveMap(folderPath, mapToWorld_);
	
  return true;
}

void OnlineRangeDataProcessorRos::poseStampedPriorCallback(const nav_msgs::Odometry::ConstPtr& odometryPose) {

	if((slam_->params_.odometry_.overwriteWithTf_) || !(slam_->params_.odometry_.listenPriorFromTopic_)){
		ROS_DEBUG("Prior from topic is NOT used.");
		return;
	}

	geometry_msgs::TransformStamped transformStamped;

	// Meta data.
	transformStamped.child_frame_id = o3d_slam::frames::rangeSensorFrame;
	transformStamped.header.frame_id = o3d_slam::frames::odomFrame;
	transformStamped.header.stamp = odometryPose->header.stamp;

	// Position.
	transformStamped.transform.translation.x = odometryPose->pose.pose.position.x;
	transformStamped.transform.translation.y = odometryPose->pose.pose.position.y;
	transformStamped.transform.translation.z = odometryPose->pose.pose.position.z;

	// Orientation.
	transformStamped.transform.rotation.x = odometryPose->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = odometryPose->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = odometryPose->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = odometryPose->pose.pose. orientation.w;

	
	const Time timestamp = fromRos(odometryPose->header.stamp);
	slam_->addOdometryPrior(timestamp, tf2::transformToEigen(transformStamped));
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {


	callbackLatestTimestamp_ = timestamp;

	if(slam_->params_.odometry_.overwriteWithTf_){
	//Get and set the point cloud registration prior from the tf.
		if(!getAndSetTfPrior(timestamp)){
			std::cout << " No prior found for timestamp " << timestamp << std::endl; 
			return;
		}
	}
	// Add the scan measurement
	slam_->addRangeScan(cloud, timestamp);
	pcArriveTime_ = std::chrono:: high_resolution_clock::now();
	rosTimeAtTheTimeOfTheLastCallback_ = ros::Time::now();
	// Publish the raw point cloud, immediately.
  	o3d_slam::publishCloud(cloud , o3d_slam::frames::rangeSensorFrame, toRos(timestamp), rawCloudPub_);
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

void OnlineRangeDataProcessorRos::registeredCloudPublisherCallback(const ros::TimerEvent&){

	// Get the latest registered cloud. If none exist, returns the constuctor initialized cloud.
	o3d_slam::SlamWrapper::RegisteredPointCloud registeredCloud = slam_->getLatestRegisteredCloud();

	// Check if the cloud actually arrived and valid.
	if(!isTimeValid(registeredCloud.raw_.time_)){
		ROS_WARN_THROTTLE(1, "Invalid time to publish registered cloud. (Throttled to 1 Hz)");
		return;
	}

	if(!isRegisteredCloudAlreadyPublished(registeredCloud.raw_.time_)){
		//std::cerr << "Registered cloud already published." << std::endl;
		return;
	}

	// Transform the cloud to the map_new frame.
	registeredCloud.raw_.cloud_ = registeredCloud.raw_.cloud_.Transform(registeredCloud.transform_.matrix());

	// Publish the registered cloud.
	o3d_slam::publishCloud(registeredCloud.raw_.cloud_ , o3d_slam::frames::mapFrame, toRos(registeredCloud.raw_.time_), registeredCloudPub_);
	return;
}

bool OnlineRangeDataProcessorRos::isRegisteredCloudAlreadyPublished(const Time &timestamp){
	if(timestamp == lastPublishedRegisteredCloudTime_){
		return true;
	}
	lastPublishedRegisteredCloudTime_ = timestamp;
	return false;
}

void OnlineRangeDataProcessorRos::posePublisherTimerCallback(const ros::TimerEvent&){
	

	if(!slam_->isNewScan2MapRefinementAvailable_){
		//std::cout << "Scan to map registration has not yet started." << std::endl;
		return;
	}

	willPublish_ = std::chrono:: high_resolution_clock::now();


	std::chrono::duration<double> time_span =  std::chrono::duration_cast< std::chrono::duration<double>>(willPublish_ - pcArriveTime_);
	// std::cout << "Point cloud arrival to enabled publishing: " << time_span.count() * 1e+3 << " ms" << std::endl;

	Time latestTransformTime; 
	Transform mapToRangeSensorTransform;
	Transform latestOdomToRangeSensorInBuffer_ = slam_->getLatestOdometryPose();
	{
		// Thread safety, memory access
		std::lock_guard<std::mutex> methodLock(slam_->posePublishingMutex_);

		mapToRangeSensorTransform = slam_->latestScanToMapRefinedPose_;
		latestTransformTime = slam_->latestScanToMapRefinementTimestamp_;
		slam_->isNewScan2MapRefinementAvailable_=false;
	}

	if(slam_->params_.odometry_.publishMapToOdomTfTransform_){

		Transform mapToOdomtt_fast = mapToRangeSensorTransform * latestOdomToRangeSensorInBuffer_.inverse();
		o3d_slam::publishTfTransform(mapToOdomtt_fast.matrix().inverse(), ros::Time::now(), o3d_slam::frames::odomFrame, o3d_slam::frames::mapFrame, tfBroadcaster_.get());
	}

	const geometry_msgs::TransformStamped scan2maptransformMsg = getTransformMsg(mapToRangeSensorTransform, o3d_slam::toRos(latestTransformTime), o3d_slam::frames::mapFrame,  o3d_slam::frames::rangeSensorFrame);
	const nav_msgs::Odometry scan2mapOdometryMsg = getOdomMsg(scan2maptransformMsg);
	publishIfSubscriberExists(scan2maptransformMsg, scan2mapTransformPublisher_);
	publishIfSubscriberExists(scan2mapOdometryMsg, scan2mapOdometryPublisher_);

	// Publish scan2map initial guess
	const Transform T_prior = slam_->getMapToRangeSensorPrior(latestTransformTime);
				geometry_msgs::TransformStamped transformMsgPrior = getTransformMsg(T_prior, o3d_slam::toRos(latestTransformTime), o3d_slam::frames::mapFrame,  o3d_slam::frames::rangeSensorFrame);
	nav_msgs::Odometry odomMsg_prior = getOdomMsg(transformMsgPrior);
	publishIfSubscriberExists(odomMsg_prior, scan2mapOdometryPriorPublisher_);

	{
		geometry_msgs::PoseStamped poseStamped;
		lidarPathInMap.header.frame_id = o3d_slam::frames::mapFrame;
		poseStamped.header.frame_id = o3d_slam::frames::mapFrame;

		poseStamped.header.stamp = o3d_slam::toRos(latestTransformTime);
		poseStamped.pose.position.x = scan2mapOdometryMsg.pose.pose.position.x;
		poseStamped.pose.position.y = scan2mapOdometryMsg.pose.pose.position.y;
		poseStamped.pose.position.z = scan2mapOdometryMsg.pose.pose.position.z;

		poseStamped.pose.orientation.x = scan2mapOdometryMsg.pose.pose.orientation.x;
		poseStamped.pose.orientation.y = scan2mapOdometryMsg.pose.pose.orientation.y;
		poseStamped.pose.orientation.z = scan2mapOdometryMsg.pose.pose.orientation.z;
		poseStamped.pose.orientation.w = scan2mapOdometryMsg.pose.pose.orientation.w;

		lidarPathInMap.poses.push_back(poseStamped);
	}

	lidarPoseInMapPathPublisher_.publish(lidarPathInMap);

	// Append pose to path.
	trajectoryAlignmentHandlerPtr_->addLidarPose(mapToRangeSensorTransform.translation(), o3d_slam::toRos(latestTransformTime).toSec());

	// Experimental section. The latest mapToRangeSensorTransform is propagated to the current timestamp through the tf.
	Transform tfQueriedLatestOdometry;

	// The odometry prior at the time of scan2map refinement
	ros::Time currentTime = ros::Time::now();
	if(o3d_slam::lookupTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, currentTime, tfBuffer_, tfQueriedLatestOdometry))
	{
		// The odometry motion calculated between the odometry prior at the time opf Scan2map and the current odometry pose.
		// Hypothesis: if scan2map refinement takes ages, this difference might be big.
		// Otherwise expected to be near identical.
		const Transform odometryMotion = latestOdomToRangeSensorInBuffer_.inverse()*tfQueriedLatestOdometry;
		
		// We propagate the map2lidar refined pose. This is no longer the registered pose. Shouldnt be used for point cloud accumulation.
		Transform consolidatedMapToRangeSensor = mapToRangeSensorTransform * odometryMotion ;

		// The consolidated mapToOdom calculation. Expected to be more accurate if scan2map registration takes long.
		Transform mapToOdomConsolidated = consolidatedMapToRangeSensor * tfQueriedLatestOdometry.inverse();

		//std::cout << "Consolidated mapToOdom: " << o3d_slam::asString(mapToOdomConsolidated) << "\n";
		ros::Duration timeDifference = currentTime - o3d_slam::toRos(latestTransformTime);
		// ROS_INFO_STREAM("Time difference Current Time - latestTransformTime: " << timeDifference.toNSec() * 1e-6 << " ms");

		ros::Duration timeDifference3 = currentTime - o3d_slam::toRos(callbackLatestTimestamp_);
		// ROS_INFO_STREAM("Time difference Current Time - timeStampOfTheLastArrivedCloud: " << timeDifference3.toNSec() * 1e-6 << " ms");

		ros::Duration timeDifferenceBeforeRegister = slam_->getCurrentTimeBeforeRegistration() - rosTimeAtTheTimeOfTheLastCallback_;
		// ROS_INFO_STREAM("Time difference Ros time before registration - rosTimeAtTheTimeOfTheLastPointCloud_: " << timeDifferenceBeforeRegister.toNSec() * 1e-6 << " ms");

		ros::Duration timeDifferenceAfterRegister = slam_->getCurrentTimeAfterRegistration() - rosTimeAtTheTimeOfTheLastCallback_;
		// ROS_INFO_STREAM("Time difference Ros time after registration - rosTimeAtTheTimeOfTheLastPointCloud_: " << timeDifferenceAfterRegister.toNSec() * 1e-6 << " ms");

		std::chrono::duration<double> time_spanBefore =  std::chrono::duration_cast< std::chrono::duration<double>>(slam_->getBeforeRegistrationTime() - pcArriveTime_);
		// std::cout << "Before Registration (HighRes) - time when last cloud arrived (highRes): " << time_spanBefore.count() * 1e+3 << " ms" << std::endl;

		std::chrono::duration<double> time_spanAfter =  std::chrono::duration_cast< std::chrono::duration<double>>(slam_->getAfterRegistrationTime() - pcArriveTime_);
		// std::cout << "After Registration (HighRes) - time when last cloud arrived (highRes): " << time_spanAfter.count() * 1e+3 << " ms" << std::endl;

		ros::Duration timeDifferenceAfterRegisterDiff =currentTime - rosTimeAtTheTimeOfTheLastCallback_;
		// ROS_INFO_STREAM("Time difference currentTime - rosTimeAtTheTimeOfTheLastCallback_: " << timeDifferenceAfterRegisterDiff.toNSec() * 1e-6 << " ms");

		// Difference of above 2
		//ros::Duration timeDifference2 = o3d_slam::toRos(callbackLatestTimestamp_) - o3d_slam::toRos(latestTransformTime);
		//ROS_INFO_STREAM("Time difference timeStampOfTheLastArrivedCloud - latestTransformTime " << timeDifference2.toNSec() * 1e-6 << " ms");

		
		// Publish the fast but propagated map2odom.  Odom is the parent, map_consolidated is the child.
		o3d_slam::publishTfTransform(mapToOdomConsolidated.matrix().inverse(), currentTime, o3d_slam::frames::odomFrame, "map_consolidated",
				tfBroadcaster_.get());
		
		geometry_msgs::TransformStamped consolidatedScan2maptransformMsg = getTransformMsg(consolidatedMapToRangeSensor, currentTime, o3d_slam::frames::mapFrame,  o3d_slam::frames::rangeSensorFrame);
		nav_msgs::Odometry consolidatedScan2mapodomMsg = getOdomMsg(consolidatedScan2maptransformMsg);
		publishIfSubscriberExists(consolidatedScan2maptransformMsg, consolidatedScan2mapTransformPublisher_);
		publishIfSubscriberExists(consolidatedScan2mapodomMsg, consolidatedScan2mapOdomPublisher_);

		geometry_msgs::PoseStamped poseStamped;
		consolidatedLiDARpathInMap.header.frame_id = o3d_slam::frames::mapFrame;
		poseStamped.header.frame_id = o3d_slam::frames::mapFrame;

		poseStamped.header.stamp = currentTime;
		poseStamped.pose.position.x = consolidatedScan2mapodomMsg.pose.pose.position.x;
		poseStamped.pose.position.y = consolidatedScan2mapodomMsg.pose.pose.position.y;
		poseStamped.pose.position.z = consolidatedScan2mapodomMsg.pose.pose.position.z;

		poseStamped.pose.orientation.x = consolidatedScan2mapodomMsg.pose.pose.orientation.x;
		poseStamped.pose.orientation.y = consolidatedScan2mapodomMsg.pose.pose.orientation.y;
		poseStamped.pose.orientation.z = consolidatedScan2mapodomMsg.pose.pose.orientation.z;
		poseStamped.pose.orientation.w = consolidatedScan2mapodomMsg.pose.pose.orientation.w;

		consolidatedLiDARpathInMap.poses.push_back(poseStamped);
		consolidatedLidarPoseInMapPathPublisher_.publish(consolidatedLiDARpathInMap);

	}else{
		ROS_WARN("TF did not provide prior. Tf based transforms not published.");
		return;
	}

	// if the GNSS is available and the ENU to map handler is ported we could also add the GNSS pose to the path.
	//trajectoryAlignmentHandlerPtr_->addGnssPose(latestOdomToRangeSensorInBuffer_.translation(), o3d_slam::toRos(latestTransformTime).toSec());	
};

bool OnlineRangeDataProcessorRos::getAndSetTfPrior(const Time &queryTime){

	// If the query time is not valid, we cannot do anything. Early return.
	if(!isTimeValid(queryTime)){
		return false;
	}

	// Check if we can transform.
	if (!tfBuffer_.canTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, o3d_slam::toRos(queryTime), ros::Duration(0.05))) {
		ROS_WARN_STREAM("Requested transform from frames " <<  o3d_slam::frames::odomFrame << " to " << o3d_slam::frames::rangeSensorFrame << " with target timestamp lidar cannot be found.");

		// We return and later lidar-to-lidar odometry will replace the prior.
		return false;
	}

	// Goal: Set the initial guess of the registration.
	Transform odometryQuery;
	if(!o3d_slam::lookupTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, o3d_slam::toRos(queryTime), tfBuffer_, odometryQuery))
	{
		// We already have an exception block inside.
		return false;
	}

	// Enable for debugging.
	//std::cout << "Latest TF query time: " << o3d_slam::toSecondsSinceFirstMeasurement(queryTime) << std::endl;

	// Add the found odometry prior to the buffer.
	//ROS_INFO_STREAM("Adding odometry prior " << asString(odometryQuery) << " at time: " << o3d_slam::toRos(queryTime).toSec());
	slam_->addOdometryPrior(queryTime, odometryQuery);
	return true;
	
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, false);
	const Time timestamp = fromRos(msg->header.stamp);
	ros::Time currentTime = ros::Time::now();

	//accumulateAndProcessRangeData(cloud, timestamp);

	if (cloud.IsEmpty()) {
		std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
		return;
	}

	ros::Duration timeDifferenceAfterRegister = ros::Time::now() - msg->header.stamp;
	// ROS_INFO_STREAM("Time difference Current time - pcArrivalStamp: " << timeDifferenceAfterRegister.toNSec() * 1e-6 << " ms");

	processMeasurement(cloud, timestamp);

}



} // namespace o3d_slam

