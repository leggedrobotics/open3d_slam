#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//namespace m545_mapping{
//
//class CvImage{
//
//public:
//    CvImage();
//    ~CvImage() = default;

    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;
//    cv_bridge::CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
//                                   const std::string& encoding = std::string());
    Eigen::RowVector3d imageConversion(const sensor_msgs::ImageConstPtr& msg, int& i, int& j, const std::string& encoding = std::string()) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, encoding);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return Eigen::RowVector3d::Zero();
        }
        Eigen::RowVector3d pixelColor = cv_ptr->image.at<Eigen::RowVector3d>(i, j);   //BGR
//        std::swap(pixelColor(0), pixelColor(2));    //RGB
        return pixelColor;
    }

//};

//typedef boost::shared_ptr<CvImage> CvImagePtr;
//typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

//}