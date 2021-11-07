//
// Created by helecomika on 11/3/21.
//
#include "m545_volumetric_mapping/Color.hpp"
#include <open3d/geometry/PointCloud.h>

namespace m545_mapping {

    Color::Color() {};

    open3d::geometry::PointCloud Color::projectionAndColor(open3d::geometry::PointCloud &cloud,
                                                           const sensor_msgs::ImageConstPtr &msg,
                                                           const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K,
                                                           const Eigen::Matrix<double, 5, 1> &D,
                                                           const Eigen::Quaternion<double> &quaternion,
                                                           const Eigen::Vector3d &translation,
                                                           const bool &cropFlag) {

        Color::pos_lidar = cloud.points_;
        std::vector<Eigen::Vector4d> pos_lidar_aux(pos_lidar.size());       //[x_l,y_l,z_l,1]
        std::vector<Eigen::Vector3d> pos_udimage_aux(pos_lidar.size());     //[u,v,1]
        std::vector<Eigen::Vector2i> pos_udimage(pos_lidar.size());         //[u,v]
        std::vector<Eigen::Vector2i> pos_dimage(pos_lidar.size());          //with distortion
        Eigen::Matrix3d rotation;
        Eigen::MatrixXd RT(3, 4);           //[R|T]
        rotation = quaternion.toRotationMatrix();
        RT.leftCols(3) = rotation;
        RT.col(3) = translation;
        for (int i = 0; i < pos_lidar.size(); i++) {
//        pos_lidar_aux[i].topRows(3) = pos_lidar[i];
            pos_lidar_aux[i].x() = -pos_lidar[i].y();
            pos_lidar_aux[i].y() = pos_lidar[i].x();
            pos_lidar_aux[i].z() = pos_lidar[i].z();
            pos_lidar_aux[i](3) = 1.0;
            pos_udimage_aux[i] = RT * pos_lidar_aux[i];         //[K*[R|T]*[x_l,y_l,z_l,1] = lamda*[u.v.1]
            if(pos_udimage_aux[i].z() < 0) {
                pos_udimage[i].x() = -1.0;
                pos_udimage[i].y() = -1.0;          //get rid of the points with z<0 and color them later with white
            }
            else {
                pos_udimage_aux[i] = K * pos_udimage_aux[i];
                pos_udimage_aux[i] = pos_udimage_aux[i] / pos_udimage_aux[i](2);
                pos_udimage[i].x() = ceil(pos_udimage_aux[i].x());            //consider pixel size
                pos_udimage[i].y() = ceil(pos_udimage_aux[i].y());
                //the distortion part
                double u0 = K(0, 2);
                double v0 = K(1, 2);
                double u_differ = pos_udimage_aux[i].x() - u0;
                double v_differ = pos_udimage_aux[i].y() - v0;
                double r_square = pow(u_differ, 2) + pow(v_differ, 2);
                pos_dimage[i].x() = round(pos_udimage_aux[i].x());
                pos_dimage[i].y() = round(pos_udimage_aux[i].y());
                pos_dimage[i].x() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * u_differ +
                                          2 * D(2) * u_differ * v_differ + D(3) * (r_square + 2 * pow(u_differ, 2)) + u0);
                pos_dimage[i].y() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * v_differ +
                                          2 * D(3) * u_differ * v_differ + D(2) * (r_square + 2 * pow(v_differ, 2)) + v0);
            }
        }
        cloud.colors_ = imageConversion(msg, pos_udimage);

        std::vector<Eigen::Matrix<double, 3, 1>> posArray;
        std::vector<Eigen::Matrix<double, 3, 1>> colorArray;
        std::vector<Eigen::Matrix<double, 3, 1>> posArrayCloud2;
        std::vector<Eigen::Matrix<double, 3, 1>> colorArrayCloud2;

        if (cropFlag) {
            int j = 0;
            for (int i = 0; i < cloud.points_.size(); i++) {
                if (cloud.colors_[i] != whitePoint) {
                    posArray.push_back(cloud.points_[i]);
                    colorArray.push_back(cloud.colors_[i]);
                    j++;
                }
                else {
                    posArrayCloud2.push_back(cloud.points_[i]);
                    colorArrayCloud2.push_back(whitePoint);
                }
            }
        }
        open3d::geometry::PointCloud newCloud(posArray);
        newCloud.colors_ = colorArray;
        cloud2.points_ = posArrayCloud2;
        cloud2.colors_ = colorArrayCloud2;

        return newCloud;
//    return cloud;

    }

    std::vector<Eigen::Matrix<double, 3, 1>> Color::imageConversion(const sensor_msgs::ImageConstPtr &msg, const std::vector<Eigen::Vector2i> pixels) {
        cv_bridge::CvImagePtr cv_ptr;
//        try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
//        }
//        catch (cv_bridge::Exception &e) {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//        }

        std::vector<cv::Vec3b> pixelColorscv(pixels.size());
        std::vector<Eigen::Matrix<double, 3, 1>> pixelColors(pixels.size());
        for (int i = 0; i < pixels.size(); i++) {

            if (pixels[i].x() >= 0 && pixels[i].x() < cv_ptr->image.rows && pixels[i].y() >= 0 && pixels[i].y() < cv_ptr->image.cols) {
                pixelColorscv[i] = cv_ptr->image.at<cv::Vec3b>(pixels[i].x(), cv_ptr->image.cols-pixels[i].y());
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
        return pixelColors;
    }

    open3d::geometry::PointCloud Color::getCloud2() {

        return cloud2;
    }
}
