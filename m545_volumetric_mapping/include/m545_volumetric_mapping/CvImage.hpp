#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "opencv2/core/eigen.hpp"

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
//        std::cout << "color" << std::endl;
        cv_bridge::CvImagePtr cv_ptr;
//        try {
        cv_ptr = cv_bridge::toCvCopy(msg, encoding);
//        }
//        catch (cv_bridge::Exception &e) {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return Eigen::RowVector3d::Zero();
//        }

        std::vector<cv::Vec3b> pixelColorscv(pixels.size());
        std::vector<Eigen::Matrix<double, 3, 1>> pixelColors(pixels.size());
        for (int i = 0; i < pixels.size(); i++) {
//            int x = pixels[i].x();
//            int y = pixels[i].y();
//            std::cout << "pixel coordinates " << pixels[i].transpose() << std::endl;
//            std::cout << msg->height << msg->width << std::endl;
            if (pixels[i].x() >= 0 && pixels[i].x() < cv_ptr->image.rows && pixels[i].y() >= 0 && pixels[i].y() < cv_ptr->image.cols) {
                pixelColorscv[i] = cv_ptr->image.at<cv::Vec3b>(pixels[i].x(), pixels[i].y());
//                pixelColors[i] = {0, 0, 0};
//                std::cout << "coordinates" << x << "  " << y << std::endl;

                pixelColors[i].x() = pixelColorscv[i](0);
                pixelColors[i].y() = pixelColorscv[i](1);
                pixelColors[i].z() = pixelColorscv[i](2);
                pixelColors[i] = pixelColors[i] / 255.0;
//                std::cout << "pixelcolor:" << pixelColors[i].transpose() << std::endl;
            }
            else {
                pixelColors[i].x() = 1.0;
                pixelColors[i].y() = 1.0;
                pixelColors[i].z() = 1.0;
                pixelColorscv[i] = cv::Vec3b(1.0, 1.0, 1.0);
            }
//            std::cout << "pixelcolor:" << pixelColors[i].transpose() << std::endl;
        }
//        cv::Mat imgenerate = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3, pixelColorscv.data());
//        cv::imshow("imwindow", imgenerate);
//        cv::waitKey(5);
//        Eigen::RowVector3d pixelColor = cv_ptr->image.at<Eigen::RowVector3d>(i, j);
//        std::swap(pixelColor(0), pixelColor(2));    //RGB
        for (int i = 0; i < pixelColors.size(); i++) {
//        std::cout << "color:" << pixelColors[i].transpose() << std::endl;
        }
        return pixelColors;
    }

//};
//
//typedef boost::shared_ptr<CvImage> CvImagePtr;
//typedef boost::shared_ptr<CvImage const> CvImageConstPtr;
//
//}