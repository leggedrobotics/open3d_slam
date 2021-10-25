#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>


YAML::Node config = YAML::LoadFile("/home/helecomika/sp_ws/src/m545_volumetric_mapping/m545_volumetric_mapping/param/params_projection.yaml");
const std::vector<double> vK = config["K"].as<std::vector<double> >();
Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K(vK.data());
const std::vector<double> vD = config["D"].as<std::vector<double> >();
Eigen::Matrix<double, 5, 1> D(vD.data());
const std::vector<double> vqua = config["quaternion"].as<std::vector<double> >();
Eigen::Quaternion<double> quaternion(vqua.data());
const std::vector<double> vtran = config["translation"].as<std::vector<double> >();
Eigen::Vector3d translation(vtran.data());

Eigen::Matrix3d rotation;
Eigen::MatrixXd RT(3, 4);


std::vector<Eigen::Vector2i> projectionLidarToPixel(const std::vector<Eigen::Vector3d> &pos_lidar) {
    std::vector<Eigen::Vector4d> pos_lidar_aux(pos_lidar.size());
    std::vector<Eigen::Vector3d> pos_udimage_aux(pos_lidar.size());
    std::vector<Eigen::Vector2i> pos_dimage(pos_lidar.size());
    rotation = quaternion.toRotationMatrix();
    RT.leftCols(3) = rotation;
    RT.col(3) = translation;
    for (int i = 0; i < pos_lidar.size(); i++) {
        pos_lidar_aux[i].topRows(3) = pos_lidar[i];
        pos_lidar_aux[i](3) = 1.0;
        pos_udimage_aux[i] = K * RT * pos_lidar_aux[i];
        pos_udimage_aux[i] = pos_udimage_aux[i] / pos_udimage_aux[i](2);
        double u0 = K(0, 2);
        double v0 = K(1, 2);
        double u_differ = pos_udimage_aux[i].x() - u0;
        double v_differ = pos_udimage_aux[i].y() - v0;
        double r_square = pow(u_differ, 2) + pow(v_differ, 2);
        pos_dimage[i].x() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * u_differ +
                              2 * D(2) * u_differ * v_differ + D(3) * (r_square + 2 * pow(u_differ, 2)) + u0);
        pos_dimage[i].y() = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * v_differ +
                              2 * D(3) * u_differ * v_differ + D(2) * (r_square + 2 * pow(v_differ, 2)) + v0);
    }
//    pos_lidar_aux.topRows(3) = pos_lidar;
//    pos_lidar_aux(3) = 1.0;
//    pos_udimage_aux = K * RT * pos_lidar_aux;
//    pos_udimage_aux = pos_udimage_aux / pos_udimage_aux(2);
//    double u0 = K(0, 2);
//    double v0 = K(1, 2);
//    double u_differ = pos_udimage_aux(0) - u0;
//    double v_differ = pos_udimage_aux(1) - v0;
//    double r_square = pow(u_differ, 2) + pow(v_differ, 2);
//    pos_dimage(0) = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * u_differ +
//                    2 * D(2) * u_differ * v_differ + D(3) * (r_square + 2 * pow(u_differ, 2)) + u0);
//    pos_dimage(1) = round((1 + D(0) * r_square + D(1) * pow(r_square, 2) + D(4) * pow(r_square, 3)) * v_differ +
//                    2 * D(3) * u_differ * v_differ + D(2) * (r_square + 2 * pow(v_differ, 2)) + v0);
    return pos_dimage;
}
