/*
 * creators.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */
#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"


namespace o3d_slam {



std::shared_ptr<OnlineRangeDataProcessorRos> createOnlineDataProcessor(ros::NodeHandlePtr nh);
std::shared_ptr<RosbagRangeDataProcessorRos> createRosbagDataProcessor(ros::NodeHandlePtr nh);

std::shared_ptr<DataProcessorRos> dataProcessorFactory(ros::NodeHandlePtr nh, bool isProcessAsFastAsPossible);


} /* namespace o3d_slam */
