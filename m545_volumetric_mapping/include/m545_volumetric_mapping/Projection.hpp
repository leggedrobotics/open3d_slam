#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
#include <iostream>


//YAML::Node config = YAML::LoadFile("/home/helecomika/sp_ws/src/m545_volumetric_mapping/m545_volumetric_mapping/param/params_projection.yaml");
//const std::vector<double> vK = config["K"].as<std::vector<double> >();
//Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K(vK.data());
//const std::vector<double> vD = config["D"].as<std::vector<double> >();
//Eigen::Matrix<double, 5, 1> D(vD.data());
//const std::vector<double> vqua = config["quaternion"].as<std::vector<double> >();
//Eigen::Quaternion<double> quaternion(vqua.data());
//const std::vector<double> vtran = config["translation"].as<std::vector<double> >();
//Eigen::Vector3d translation(vtran.data());


std::vector<Eigen::Vector2i> projectionLidarToPixel(const std::vector<Eigen::Matrix<double, 3, 1>> &pos_lidar, const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K, const Eigen::Matrix<double, 5, 1> &D, const Eigen::Quaternion<double> &quaternion, const Eigen::Vector3d &translation) {
    std::vector<Eigen::Vector4d> pos_lidar_aux(pos_lidar.size());
    std::vector<Eigen::Vector3d> pos_udimage_aux(pos_lidar.size());
    std::vector<Eigen::Vector2i> pos_udimage(pos_lidar.size());
    std::vector<Eigen::Vector2i> pos_dimage(pos_lidar.size());
    Eigen::Matrix3d rotation;
    Eigen::MatrixXd RT(3, 4);
    rotation = quaternion.toRotationMatrix();
    RT.leftCols(3) = rotation;
    RT.col(3) = translation;
    for (int i = 0; i < pos_lidar.size(); i++) {
        pos_lidar_aux[i].topRows(3) = pos_lidar[i];
        pos_lidar_aux[i](3) = 1.0;
        pos_udimage_aux[i] = K * RT * pos_lidar_aux[i];
        pos_udimage_aux[i] = pos_udimage_aux[i] / pos_udimage_aux[i](2);
        pos_udimage[i].x() = ceil(pos_udimage_aux[i].x() / 2.5);
        pos_udimage[i].y() = ceil(pos_udimage_aux[i].y() / 2.5);

//        double u0 = K(0, 2);
//        double v0 = K(1, 2);
//        double u_differ = pos_udimage_aux[i].x() - u0;
//        double v_differ = pos_udimage_aux[i].y() - v0;
//        double r_square = pow(u_differ, 2) + pow(v_differ, 2);
//        pos_dimage[i].x() = round(pos_udimage_aux[i].x());
//        pos_dimage[i].y() = round(pos_udimage_aux[i].y());
//        pos_dimage[i].x() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * u_differ +
//                              2 * D(2) * u_differ * v_differ + D(3) * (r_square + 2 * pow(u_differ, 2)) + u0);
//        pos_dimage[i].y() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * v_differ +
//                              2 * D(3) * u_differ * v_differ + D(2) * (r_square + 2 * pow(v_differ, 2)) + v0);
//        std::cout << pos_udimage_aux[i].transpose() << std::endl;
    }
    for(int i=0;i<pos_lidar.size();i++)
    {
        std::cout << "before:" << pos_lidar[i].transpose() << "after:" << pos_udimage[i].transpose() << std::endl;
    }
    return pos_udimage;
}
