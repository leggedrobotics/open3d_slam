#include "ros/ros.h"
#include <sensor_msgs/point_cloud2_iterator.h>

// #include "Lid_Cam/ColorProjection.hpp"
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/RGBDImage.h>
// #include "Lid_Cam/helpers.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <open3d/Open3D.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher pub_goal;
geometry_msgs::PoseStamped goal;

// void cloudCallback(const sensor_msgs::PointCloud2 &msg) {
// 	const auto ros_pc2 = &cloud;
// 	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
// 	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
// 	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
// 	sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
// 	sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
// 	sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
	
// 	for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
// 					++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) 
// 	{
// 	o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
// 	o3d_pc.colors_.push_back(
// 	Eigen::Vector3d(((int) (*ros_pc2_r)) / 255.0, ((int) (*ros_pc2_g)) / 255.0,((int) (*ros_pc2_b)) / 255.0));
// 	}

//     sensor_msgs::PointCloud2 cloud_msg ;
//     cloud_msg.header = msg.header;
//     cloud_msg.header.stamp =ros::Time::now();
//     pub.publish(cloud_msg);

// }
void cloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
	float hum_x,hum_y,hum_z;
	int counter = 0;
	hum_x =0; hum_y =0; hum_z = 0;
	float hum_low, hum_high;
	hum_low = 10;
	hum_high = -10;
	sensor_msgs::PointCloud2ConstIterator<uint8_t> it_r(*msg, "r");
	sensor_msgs::PointCloud2ConstIterator<uint8_t> it_g(*msg, "g");
	sensor_msgs::PointCloud2ConstIterator<uint8_t> it_b(*msg, "b");
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
 			 // ROS_INFO_STREAM(" pt" << it[0] << it[1]<<it[2] );
    		 
 			 // ROS_INFO_STREAM(" col" << (int)(*it_r) <<" "  << (int)(*it_g) <<" "<< (int)(*it_b) );
    		//  if((int)(*it_r) >200 && (int)(*it_g) < 100 && (int)(*it_b) < 100  )
    		 if((int)(*it_b) >200  )
    		 	{	
	    		 	hum_x +=it[0];
	    		 	hum_y +=it[1];
	    		 	hum_z +=it[2];
					if(hum_low > it[0])
					 	hum_low = it[0];
					if(hum_high < it[0])
						hum_high= it[0];
					
						
	    		 	counter ++;
    		 	}
    		 	++it_r;
    			++it_g;
    		 	++it_b;
    		 	 	
    	} 
    		 if(counter)
    		 {
    		 goal.pose.position.x = hum_x/counter;goal.pose.position.y = hum_y/counter ;goal.pose.position.z = hum_z/counter;
			 goal.pose.orientation.x = hum_high - hum_low;
			 goal.pose.orientation.x = goal.pose.orientation.x/10;

   		 goal.header =msg->header;
   		 goal.header.stamp = msg->header.stamp;
   		 pub_goal.publish(goal);
   		 }
						ROS_INFO_STREAM("Number of human pc points "<< counter<<"size"<<goal.pose.orientation.x*10);

    }
int main (int argc, char **argv)
{

	ros::init(argc, argv, "Mask");
   ros::NodeHandle nh;
   ros::Subscriber cloudSub;
   cloudSub = nh.subscribe("/output_scan", 10, &cloudCallback);
	pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/human_goal", 1);
	ros::spin ();
}