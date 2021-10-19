//#include "m545_volumetric_mapping/CvImage.hpp"
//#include <ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/rgb_colors.h>
//#include <sensor_msgs/image_encodings.h>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
//
//namespace m545_mapping {
//
//    CvImage::CvImage() {}
//    CvImage cvimage;
//
//        Eigen::RowVector3d CvImage::imageConversion(const sensor_msgs::ImageConstPtr &msg, int &i, int &j) {
//
//            cv_bridge::CvImagePtr cv_ptr;
//            try {
//                cv_ptr = CvImage::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//            }
//            catch (cv_bridge::Exception &e) {
//                ROS_ERROR("cv_bridge exception: %s", e.what());
//                return Eigen::RowVector3d::Zero();
//            }
//            Eigen::RowVector3d pixelColor = cv_ptr->image.at<Eigen::RowVector3d>(i, j);   //BGR
//            std::swap(pixelColor(0), pixelColor(2));    //RGB
//            return pixelColor;
//        }
//
//}