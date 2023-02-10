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
	cloudSubscriber_ = nh_->subscribe(cloudTopic_, 1, &OnlineRangeDataProcessorRos::cloudCallback,this);

	// Relavent pose subscriber
	priorPoseSubscriber_ = nh_->subscribe("/rowesys/estimator/pose_fused", 1, &OnlineRangeDataProcessorRos::poseStampedPriorCallback,this);
	
	posePublishingTimer_ = nh_->createTimer(ros::Duration(0.01), &OnlineRangeDataProcessorRos::posePublisherTimerCallback, this);

	// Service to call to get the transformation.
	alignWithWorld_ = nh_->advertiseService("align_world", &OnlineRangeDataProcessorRos::alignWithWorld, this);

	//registeredEnuCloud_ = nh_->advertise<sensor_msgs::PointCloud2>("registered_enu_cloud", 1, true);

	lidarPath_ = nh_->advertise<nav_msgs::Path>("/lidarPath", 1);
	gnssPath_ = nh_->advertise<nav_msgs::Path>("/gnssPath", 1);

	scan2scanTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("tt_scan2scan_transform", 1, true);
	scan2scanOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2scan_odometry", 1, true);
	scan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("tt_scan2map_transform", 1, true);
	scan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry", 1, true);
	scan2mapOdomPriorPublisher_ = nh_->advertise<nav_msgs::Odometry>("tt_scan2map_odometry_prior", 1, true);

	consolidatedScan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("cons_tt_scan2map_odometry", 1, true);

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

	lidarPath_.publish(path);

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
	gnssPath_.publish(path);

	slam_->setMapToEnu(mapToWorld_);
	std::string userName = getenv("USERNAME");
	std::string folderPath = "/home/" + userName +"/data/fieldMap/";
	const bool savingResult = slam_->transformSaveMap(folderPath, mapToWorld_);
	
  return true;
}

void OnlineRangeDataProcessorRos::poseStampedPriorCallback(const geometry_msgs::PoseStampedConstPtr& odometryPose) {

	if(!slam_->params_.odometry_.overwriteWithTf){
		return;
	}

	Transform transform;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.child_frame_id = o3d_slam::frames::rangeSensorFrame;
	transformStamped.header.frame_id = o3d_slam::frames::odomFrame;
	transformStamped.header.stamp = odometryPose->header.stamp;
	transformStamped.transform.translation.x = odometryPose->pose.position.x;
	transformStamped.transform.translation.y = odometryPose->pose.position.y;
	transformStamped.transform.translation.z = odometryPose->pose.position.z;

	transformStamped.transform.rotation.x = odometryPose->pose.orientation.x;
	transformStamped.transform.rotation.y = odometryPose->pose.orientation.y;
	transformStamped.transform.rotation.z = odometryPose->pose.orientation.z;
	transformStamped.transform.rotation.w = odometryPose->pose.orientation.w;

	transform = tf2::transformToEigen(transformStamped);

	slam_->addOdometryPrior(fromRos(odometryPose->header.stamp), transform);

}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud &cloud, const Time &timestamp) {

	if(slam_->params_.odometry_.overwriteWithTf){
	// Get and set the point cloud registration prior from the tf.
	//getAndSetTfPrior(timestamp);
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

void OnlineRangeDataProcessorRos::registeredCloudPublisherCallback(const ros::TimerEvent&){

	/*
	if(!scanToMapStarted_){
		std::cout << "Scan to map not yet started." << std::endl;
		return;
	}

	if((mapToWorld_.matrix() == Eigen::Matrix4d::Identity(4,4))){
		std::cout << "Return since map to enu not calculated yet." << std::endl;
		return;
	}

	auto cloudAndTimePair = slam_->getLatestRegisteredCloudTimestampPair();
	cloudAndTimePair.first = cloudAndTimePair.first.Transform(mapToWorld_.matrix());
	o3d_slam::publishCloud(cloudAndTimePair.first, "enu", toRos(cloudAndTimePair.second), registeredEnuCloud_);
	*/

}

void OnlineRangeDataProcessorRos::posePublisherTimerCallback(const ros::TimerEvent&){
	
	if(!slam_->isNewScan2MapRefinementAvailable_){
		return;
	}

	scanToMapStarted_ = true;

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
	// The odometry prior at the time of scan2map refinement
	Transform latestOdomToRangeSensorInBuffer_ = slam_->getLatestOdometryPose();
	if(o3d_slam::lookupTransform(o3d_slam::frames::odomFrame, o3d_slam::frames::rangeSensorFrame, ros::Time::now(), tfBuffer_, tfQueriedodometry))
	{
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

		std::cout << "Consolidated mapToOdom: " << o3d_slam::asString(mapToOdomConsolidated) << "\n";
		std::cout << "mapToOdom: " << o3d_slam::asString(mapToOdom) << "\n";
		
		// Publish the slow but accurate map2odom.  Odom is the parent, map is the child.
		//o3d_slam::publishTfTransform(mapToOdom.matrix().inverse(),toRos(latestTransformTime), o3d_slam::frames::odomFrame, o3d_slam::frames::mapFrame,
		//		tfBroadcaster2_.get());
		/*
		// Publish the fast but propagated map2odom.  Odom is the parent, map_consolidated is the child.
		o3d_slam::publishTfTransform(mapToOdomConsolidated.matrix().inverse(), ros::Time::now(), o3d_slam::frames::odomFrame, "map_consolidated",
				tfBroadcaster_.get());
		*/
		geometry_msgs::TransformStamped consolidatedScan2maptransformMsg = getTransformMsg(consolidatedMapToRangeSensor, ros::Time::now(), "map_new",  o3d_slam::frames::rangeSensorFrame);
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

	trajectoryAlignmentHandlerPtr_->addLidarPose(mapToRangeSensorTransform.translation(), o3d_slam::toRos(latestTransformTime).toSec());
	trajectoryAlignmentHandlerPtr_->addGnssPose(latestOdomToRangeSensorInBuffer_.translation(), o3d_slam::toRos(latestTransformTime).toSec());
	
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

