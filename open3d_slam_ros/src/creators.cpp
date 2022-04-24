/*
 * creators.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */


#include "open3d_slam_ros/creators.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"


namespace o3d_slam {


std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(ros::NodeHandlePtr nh){
	return std::make_shared<OnlineRangeDataProcessorRos>(nh);
}
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(ros::NodeHandlePtr nh){
	return std::make_shared<RosbagRangeDataProcessorRos>(nh);
}


std::shared_ptr<DataProcessorRos> dataProcessorFactory(ros::NodeHandlePtr nh, bool isProcessAsFastAsPossible){
	if (isProcessAsFastAsPossible){
		return createRosbagDataProcessor(nh);
	} else {
		return createOnlineDataProcessor(nh);
	}
}

} /* namespace o3d_slam */
