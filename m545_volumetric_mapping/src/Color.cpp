//
// Created by helecomika on 11/3/21.
//
#include "m545_volumetric_mapping/Color.hpp"
#include <open3d/geometry/PointCloud.h>
#include "m545_volumetric_mapping/helpers.hpp"
#include <Eigen/Geometry>

namespace m545_mapping {

    Color::Color() {};

    open3d::geometry::PointCloud Color::projectionAndColor(open3d::geometry::PointCloud &cloud,
                                                           const sensor_msgs::ImageConstPtr &msg,
                                                           const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K,
                                                           const Eigen::Matrix<double, 5, 1> &D,
                                                           const Eigen::Vector3d &rpy,
                                                           const Eigen::Vector3d &translation,
                                                           const bool &cropFlag) {

        Color::pos_lidar = cloud.points_;
        std::vector<Eigen::Vector3d> pos_udimage_aux(pos_lidar.size());     //[u,v,1]
        std::vector<Eigen::Vector2i> pos_dimage(pos_lidar.size());          //with distortion
        std::vector<Eigen::Vector3d> pos_lidar_aux(pos_lidar.size());
        Eigen::Quaterniond quaternion = m545_mapping::fromRPY(rpy);
        const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
        Eigen::MatrixXd RT(3, 4);           //[R|T]
        RT.leftCols(3) = rotation.inverse();
        RT.col(3) = translation;
        for (int i = 0; i < pos_lidar.size(); i++) {
            pos_lidar_aux[i].topRows(3) = pos_lidar[i];
            pos_lidar_aux[i](3) = 1.0;
            pos_udimage_aux[i] = RT * pos_lidar_aux[i];         //[K*[R|T]*[x_l,y_l,z_l,1] = lamda*[u.v.1]
            if(pos_udimage_aux[i].z() < 0) {
                pos_dimage[i].x() = -1.0;
                pos_dimage[i].y() = -1.0;          //get rid of the points with z<0 and color them later with white
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

        return cloud;

    }

    open3d::geometry::PointCloud Color::filterColor(open3d::geometry::PointCloud &cloud) {
        std::vector<Eigen::Matrix<double, 3, 1>> posArray;
        std::vector<Eigen::Matrix<double, 3, 1>> colorArray;
        for (int i = 0; i < cloud.points_.size(); i++) {
            if (cloud.colors_[i] != noColor) {
                posArray.push_back(cloud.points_[i]);
                colorArray.push_back(cloud.colors_[i]);
            }
        }
        open3d::geometry::PointCloud newCloud(posArray);
        newCloud.colors_ = colorArray;
        return newCloud;
    }

    std::vector<Eigen::Matrix<double, 3, 1>> Color::imageConversion(const sensor_msgs::ImageConstPtr &msg, const std::vector<Eigen::Vector2i> pixels) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        std::vector<cv::Vec3b> pixelColorscv(pixels.size());
        std::vector<Eigen::Matrix<double, 3, 1>> pixelColors(pixels.size());
        for (int i = 0; i < pixels.size(); i++) {

            if (pixels[i].y() >= 0 && pixels[i].y() < cv_ptr->image.rows && pixels[i].x() >= 0 && pixels[i].x() < cv_ptr->image.cols) {
                pixelColorscv[i] = cv_ptr->image.at<cv::Vec3b>(pixels[i].y(), pixels[i].x());
                pixelColors[i].x() = pixelColorscv[i](0);
                pixelColors[i].y() = pixelColorscv[i](1);
                pixelColors[i].z() = pixelColorscv[i](2);
                pixelColors[i] = pixelColors[i] / 255.0;
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
