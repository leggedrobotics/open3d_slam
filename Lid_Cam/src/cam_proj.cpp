#include <sensor_msgs/point_cloud2_iterator.h>
#include "Lid_Cam/ColorProjection.hpp"
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/RGBDImage.h>
#include "Lid_Cam/helpers.hpp"
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
#include <bits/stdc++.h>
#include <string>
using namespace std;
ros::Publisher pub;
bool get_only_fov = true; 
open3d::geometry::PointCloud cloud;
sensor_msgs::Image::Ptr Image_mask (new sensor_msgs::Image ());
int image_counter = 0;
namespace o3d_slam {

    ColorProjection::ColorProjection() {};

    open3d::geometry::PointCloud ColorProjection::projectionAndColor(open3d::geometry::PointCloud &cloud,
                                                           const sensor_msgs::ImageConstPtr &msg,
                                                           const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K,
                                                           const Eigen::Matrix<double, 5, 1> &D,
                                                           const Eigen::Vector3d &rpy,
                                                           const Eigen::Vector3d &translation,
                                                           const bool &cropFlag) {

    ColorProjection::pos_lidar = cloud.points_;

 	std::vector<Eigen::Vector3d> pos_udimage_aux(pos_lidar.size());     //[u,v,1]
 
 	std::vector<Eigen::Vector2i> pos_dimage(pos_lidar.size());          //with distortion
        std::vector<Eigen::Vector4d> pos_lidar_aux(pos_lidar.size());
        std::vector<double> depth(pos_lidar.size());
        Eigen::Quaterniond quaternion = o3d_slam::fromRPY(rpy);
        const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
        Eigen::MatrixXd T(3, 4);           //[R|T]
        T.leftCols(3) = rotation.inverse();   //its either R|T or R_inv|-R_inv* T
        // T.leftCols(3) = rotation;
        T.col(3) = translation;
        std::vector<bool> corr_mask(pos_lidar.size(),true);

	for (int i = 0; i < pos_lidar.size(); i++) {
            pos_lidar_aux[i].topRows(3) = pos_lidar[i];
            pos_lidar_aux[i](3) = 1.0;
            pos_udimage_aux[i] = T * pos_lidar_aux[i];         //[K*[R|T]*[x_l,y_l,z_l,1] = lamda*[u.v.1]
            depth[i] = pos_lidar[i].z();

            if(pos_udimage_aux[i].z() <= 0) {
                pos_dimage[i].x() = -1.0;
                pos_dimage[i].y() = -1.0;          //get rid of the points with z<0 and color them later with white
                corr_mask[i] = false;
	    }
            else {

                pos_lidar[i] = rotation.inverse() * pos_lidar[i] + translation;
                pos_lidar[i].x() /= pos_lidar[i].z();
                pos_lidar[i].y() /= pos_lidar[i].z();

                //the distortion part
                double r_square = pow(pos_lidar[i].x(), 2) + pow(pos_lidar[i].y(), 2);
                double x = pos_lidar[i].x() * (1 + D(0) * r_square + D(1) * pow(r_square, 2)) + 2 * D(2) * pos_lidar[i].x() * pos_lidar[i].y() + D(3);
                double y = pos_lidar[i].y() * (1 + D(0) * r_square + D(1) * pow(r_square, 2)) + D(2) * (r_square + 2 * pow(pos_lidar[i].y(), 2)) + 2 * D(3) * pos_lidar[i].x() * pos_lidar[i].y();
                pos_dimage[i].x() = round(K(0, 0) * x + K(0, 2));
                pos_dimage[i].y() = round(K(1, 1) * y + K(1, 2));
                

            }
        }

       	cloud.colors_ = imageConversion(msg, pos_dimage);
        open3d::geometry::PointCloud cloud_new;

        if(get_only_fov)
        {
            for (int i =0;i<pos_lidar.size();++i)
            {
                
                    if(cloud.colors_[i][0] != -1 && cloud.colors_[i][1] != -1 && cloud.colors_[i][2] != -1)
                    {
                        cloud_new.points_.push_back(rotation.inverse()*cloud.points_[i]+translation);
                        cloud_new.colors_.push_back(cloud.colors_[i]);
                        i++;
                    // ROS_INFO_STREAM("New color from " << "test"<< pos_dimage[i].x()<< " " << pos_dimage[i].y()); 

                    }
            }
        }
        else
        {
            for (int i =0;i<pos_lidar.size();++i)
            {
                
                   
                        cloud_new.points_.push_back(rotation.inverse()*cloud.points_[i]+translation);
                        cloud_new.colors_.push_back(cloud.colors_[i]);
                        i++;
                    // ROS_INFO_STREAM("New color from " << "test"<< pos_dimage[i].x()<< " " << pos_dimage[i].y()); 

            }
        }
        
            // ROS_INFO_STREAM("Total filterd Points " << i); 

       // depthInfo = getDepth(pos_dimage, depth);
//          open3d.io.write_point_cloud('out.pcd',cloud);
        
        
    return cloud_new;

    }

 

    open3d::geometry::RGBDImage ColorProjection::getRGBDImage(const std::vector<double>& depth, const sensor_msgs::ImageConstPtr &msg) {
        open3d::geometry::Image image_color;
        open3d::geometry::Image image_depth;
        open3d::geometry::RGBDImage image_rgbd;
        image_color.data_ = msg->data;
        //std::vector to cv
        cv::Mat m = cv::Mat(msg->height, msg->width, CV_8UC1);
        std::memcpy(m.data, depth.data(), depth.size() * sizeof(double));
        //cv to o3d
        int bytes_per_channel = (m.depth() / 2 + 1);
        image_depth.Prepare(m.cols, m.rows, m.channels(), bytes_per_channel);
        std::memcpy(image_depth.data_.data(), m.data, m.total() * m.channels() * bytes_per_channel);
        image_rgbd = *open3d::geometry::RGBDImage::CreateFromColorAndDepth(image_color, image_depth, 1000, 0, false);
        return image_rgbd;
    }

    open3d::geometry::PointCloud ColorProjection::filterColor(const open3d::geometry::PointCloud&cloud) {
        std::vector<Eigen::Matrix<double, 3, 1>> posArray;
        std::vector<Eigen::Matrix<double, 3, 1>> colorArray;
        if (cloud.colors_.size() <= 0) {
            return cloud;
        }


        auto isClose = [](double val, double refValue, double tolerance){
        	return std::fabs(val - refValue) <= tolerance;
        };

        for (int i = 0; i < cloud.points_.size(); i++) {
            if (cloud.colors_[i].array().all() >= 0.0) {
                // if (cloud.colors_[i][0] >= 1.0 || cloud.colors_[i][1] >= 1.0 || cloud.colors_[i][2] >= 1.0) {
                //     std::cout << "Color value larger than 1.0 detected. " << cloud.colors_[i] << std::endl;
                //     continue;
                // }

            	const auto &c = cloud.colors_[i];
            	//hack to deal with white points
            	const double eps = 0.1;
						if (c[0] <= eps && c[1] <= eps && c[2] <= eps) {
							continue;
						}


						//hack to deal with yellow points
						if (isClose(c[0],1.0,0.2) && isClose(c[1],1.0,0.2) && isClose(c[2],0.0,0.3)){
							continue;
						}
                posArray.push_back(cloud.points_[i]);
                colorArray.push_back(cloud.colors_[i]);
            }
        }
        open3d::geometry::PointCloud newCloud(posArray);
        newCloud.colors_ = colorArray;
        return newCloud;
    }

    std::vector<Eigen::Matrix<double, 3, 1>> ColorProjection::imageConversion(const sensor_msgs::ImageConstPtr &msg, const std::vector<Eigen::Vector2i> pixels) {
    cv_bridge::CvImagePtr cv_ptr;
   	std_msgs::Header msg_header = msg->header;

   	std::string frame_id = msg_header.frame_id.c_str();
	// ROS_INFO_STREAM("New Image from " << frame_id);	
	   try
  	 {
    		 //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

  	 }
  	 catch (cv_bridge::Exception& e)
  		 {
    	 ROS_ERROR("cv_bridge exception: %s", e.what());
    		
		}

        std::vector<cv::Vec3b> pixelColorscv(pixels.size());
        std::vector<Eigen::Matrix<double, 3, 1>> pixelColors(pixels.size());
        for (int i = 0; i < pixels.size(); i++) {
        // for (int i =0; i<pos_lidar.size();++i)
        // {
            // ROS_INFO_STREAM("New color from " << "test"<< pixels[i].y() << pixels[i].x()); 
        
        // }if()
            if (pixels[i].y() >= 0 && pixels[i].y() < cv_ptr->image.rows && pixels[i].x() >= 0 && pixels[i].x() < cv_ptr->image.cols) {
                pixelColorscv[i] = cv_ptr->image.at<cv::Vec3b>( pixels[i].y(), pixels[i].x());
                if(pixelColorscv[i][0] ==1)
                {
                pixelColors[i].x() =255;
                pixelColors[i].y() =255;
                pixelColors[i].z() =255;
                pixelColors[i] = pixelColors[i]/ 255.0;
                }
                else
                {
                pixelColors[i].x() = 0;
                pixelColors[i].y() = 0;
                pixelColors[i].z() = 0;
                }
                // pixelColors[i].x() = pixelColorscv[i](0);
                // pixelColors[i].y() = pixelColorscv[i](0);
                // pixelColors[i].z() = pixelColorscv[i](0);
                

//                std::cout << "pixelcolor:" << pixelColors[i].transpose() << std::endl;
            }
            else {
                pixelColors[i].x() = -1.0;
                pixelColors[i].y() = -1.0;
                pixelColors[i].z() = -1.0;
            }
        }
        return pixelColors;
    }


}
//std::shared_ptr<o3d_slam::ColorProjection> colorProjectionPtr_;
//colorProjectionPtr_ = std::make_shared<o3d_slam::ColorProjection>();

void open3dToRos(const open3d::geometry::PointCloud &pointcloud, sensor_msgs::PointCloud2 &ros_pc2,
		std::string frame_id) {
	sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
	if (pointcloud.HasColors()) {
		modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	} else {
		modifier.setPointCloud2FieldsByString(1, "xyz");
	}
	modifier.resize(pointcloud.points_.size());
	ros_pc2.header.frame_id = frame_id;
	sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
	sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
	sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
	if (pointcloud.HasColors()) {
		sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
		for (size_t i = 0; i < pointcloud.points_.size();
				i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
			const Eigen::Vector3d &point = pointcloud.points_[i];
			const Eigen::Vector3d &color = pointcloud.colors_[i];
			*ros_pc2_x = point(0);
			*ros_pc2_y = point(1);
			*ros_pc2_z = point(2);
			*ros_pc2_r = (int) (255 * color(0));
			*ros_pc2_g = (int) (255 * color(1));
			*ros_pc2_b = (int) (255 * color(2));
		}
	} else {
		for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
			const Eigen::Vector3d &point = pointcloud.points_[i];
			*ros_pc2_x = point(0);
			*ros_pc2_y = point(1);
			*ros_pc2_z = point(2);
		}
	}
}
open3d::geometry::PointCloud  rosToOpen3d(const sensor_msgs::PointCloud2 &cloud,
		bool skip_colors) {
	const auto ros_pc2 = &cloud;
	open3d::geometry::PointCloud o3d_pc;
	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
	sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
	o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
	if (ros_pc2->fields.size() == 3 || skip_colors == true) {
		for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
			o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
		}
	} else {
		o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
		if (ros_pc2->fields[3].name == "rgb") {
			sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
			sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
			sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

			for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
					++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
				o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
				o3d_pc.colors_.push_back(
						Eigen::Vector3d(((int) (*ros_pc2_r)) / 255.0, ((int) (*ros_pc2_g)) / 255.0,
								((int) (*ros_pc2_b)) / 255.0));
			}
		} else if (ros_pc2->fields[3].name == "intensity") {
			sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
			for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
					++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i) {
				o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
				o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
			}
		}
	}
	return o3d_pc;
}
void cloudCallback(const sensor_msgs::PointCloud2 &msg) {
        // ROS_INFO_STREAM("Incoming Point Cloud" << msg.data.size());
        
        // string header  = msg.header.frame_id;
        string header = "camMainView";
    	cloud = rosToOpen3d(msg, true);
        Eigen::Vector3d rpy;
        rpy << -3.11, -1.36, -1.5;
        Eigen::Vector3d translation;
        translation << 0.207, -0.025, 0.213;
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K;
        K<< 767.239452, 0, 814.414996, 0, 767.168172, 597.978256, 0, 0, 1.0;
        Eigen::Matrix<double, 5, 1> D;
        D <<-0.044303, 0.006917, -0.000472, -0.000009, 0.000000 ;
        std::shared_ptr<o3d_slam::ColorProjection> colorProjectionPtr_;
        colorProjectionPtr_ = std::make_shared<o3d_slam::ColorProjection>();
    if(image_counter) 
    {   open3d::geometry::PointCloud coloredCloud = colorProjectionPtr_->projectionAndColor(cloud, Image_mask, K, D, rpy, translation, true);
    
    sensor_msgs::PointCloud2 cloud_msg ;
    open3dToRos(coloredCloud,cloud_msg,header);
    //cloud_msg.header.stamp = ros::Time::now();
    // ROS_INFO_STREAM("Coloured Point Cloud" << cloud_msg.data.size());
    cloud_msg.header.stamp =ros::Time::now();
    pub.publish(cloud_msg);

}
}
void maskCallback(const sensor_msgs::ImagePtr& msg) {
  image_counter =1;
  Image_mask->data = msg->data;
  Image_mask->encoding = msg->encoding;
  Image_mask->header = msg->header;
  Image_mask->height = msg->height;
  Image_mask->width = msg->width;
  Image_mask->step = msg->step;
  Image_mask->is_bigendian = msg->is_bigendian;
  
    //pub.publish(Image_mask);
 //    Eigen::Vector3d rpy;
 //        rpy << -3.11, -1.36, -1.5;
 //        Eigen::Vector3d translation;
 //        rpy << 0.207, -0.025, 0.213;
 //        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K;
 //        K<< 767.239452, 0, 814.414996, 0, 767.168172, 597.978256, 0, 0, 1.0;
 //        Eigen::Matrix<double, 5, 1> D;
 //        D <<-0.044303, 0.006917, -0.000472, -0.000009, 0.000000 ;
 //        std::shared_ptr<o3d_slam::ColorProjection> colorProjectionPtr_;
 //        colorProjectionPtr_ = std::make_shared<o3d_slam::ColorProjection>();
	// if(cloud.points_.size() >0) 
	// {	open3d::geometry::PointCloud coloredCloud = colorProjectionPtr_->projectionAndColor(cloud, msg, K, D, rpy, translation, true);
	// sensor_msgs::PointCloud2 cloud_msg ;
	// open3dToRos(coloredCloud,cloud_msg,"os_sensor");
	// //cloud_msg.header.stamp = ros::Time::now();
	// ROS_INFO_STREAM("Coloured Point Cloud" << cloud_msg.data.size());

	// pub.publish(cloud_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Cam_Lidar_Project");
    ros::NodeHandle nh;
    ros::Subscriber cloudSub;
    cloudSub = nh.subscribe("/os_cloud_node/points", 10, &cloudCallback);
    ros::Subscriber maskSub;
    maskSub = nh.subscribe("/src/mask", 100, &maskCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/output_scan", 10);
     
      // while (ros::ok())
 // {
  //

   //  ros::spinOnce();
    //     loop_rate.sleep();
 // }
	//Eigen::Vector3d rpy;
        //rpy << -3.11, -1.36, -1.5;
        //Eigen::Vector3d translation;
        //rpy << 0.207, -0.025, 0.213;
        //Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K;
        //K<< 767.239452, 0, 814.414996, 0, 767.168172, 597.978256, 0, 0, 1.0;
        //Eigen::Matrix<double, 5, 1> D;
        //D <<-0.044303, 0.006917, -0.000472, -0.000009, 0.000000 ;
	//o3d_slam::ColorProjection colorProjectionPtr_;
	//colorProjection = o3d_slam::ColorProjection();
//        std::shared_ptr<o3d_slam::ColorProjection> colorProjectionPtr_;
//	colorProjectionPtr_ = std::make_shared<o3d_slam::ColorProjection>();
	//sensor_msgs::ImagePtr Image_ptr;
	//Image_ptr =&Image_mask
	//open3d::geometry::PointCloud coloredCloud = colorProjectionPtr_->projectionAndColor(cloud, Image_mask, K, D, rpy, translation, true);
    
  // K: [767.239452, 0, 814.414996, 0, 767.168172, 597.978256, 0, 0, 1.0]
  // D: [-0.044303, 0.006917, -0.000472, -0.000009, 0.000000]
  // translation: [0.207, -0.025, 0.213]
  // rpy: [-3.11, -1.36, -1.5]
	
      ros::spin();
    }
