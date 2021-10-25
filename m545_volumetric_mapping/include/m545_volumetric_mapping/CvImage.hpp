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

    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;
//    cv_bridge::CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
//                                   const std::string& encoding = std::string());
    std::vector<Eigen::Matrix<double, 3, 1>> imageConversion(const sensor_msgs::ImageConstPtr& msg, const std::vector<Eigen::Vector2i> pixels, const std::string& encoding = std::string()) {
        cv_bridge::CvImagePtr cv_ptr;
//        try {
        cv_ptr = cv_bridge::toCvCopy(msg, encoding);
//        }
//        catch (cv_bridge::Exception &e) {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return Eigen::RowVector3d::Zero();
//        }
        std::vector<Eigen::Matrix<double, 3, 1>> pixelColors(pixels.size());
        for (int i = 0; i < pixels.size(); i++) {
            int x = pixels[i].x();
            int y = pixels[i].y();
            if (x > 0 && x < msg->width && y > 0 && y < msg->height) {
                pixelColors[i] = cv_ptr->image.at<Eigen::Matrix<double, 3, 1>>(x, y);
            }
            else {
                pixelColors[i] = Eigen::Matrix<double, 3, 1>::Zero();
            }
        }
//        Eigen::RowVector3d pixelColor = cv_ptr->image.at<Eigen::RowVector3d>(i, j);
//        std::swap(pixelColor(0), pixelColor(2));    //RGB
        return pixelColors;
    }

//};
//
//typedef boost::shared_ptr<CvImage> CvImagePtr;
//typedef boost::shared_ptr<CvImage const> CvImageConstPtr;
//
//}