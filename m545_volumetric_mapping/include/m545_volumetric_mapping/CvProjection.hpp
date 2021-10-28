#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/eigen.hpp"

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

//std::vector<cv::Point3d> point;
//std::vector<cv::Point2d> imagePoint;
//
//YAML::Node config = YAML::LoadFile("/home/helecomika/sp_ws/src/m545_volumetric_mapping/m545_volumetric_mapping/param/params_projection.yaml");
//const std::vector<double> vK = config["K"].as<std::vector<double> >();
//Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K(vK.data());
//const std::vector<double> vD = config["D"].as<std::vector<double> >();
//Eigen::Matrix<double, 5, 1> D(vD.data());
////const std::vector<double> vqua = config["quaternion"].as<std::vector<double> >();
////Eigen::Quaternion<double> quaternion(vqua.data());
//const std::vector<double> vRot = config["rotationVec"].as<std::vector<double> >();
//Eigen::Vector3d rotation(vRot.data());
//const std::vector<double> vtran = config["translation"].as<std::vector<double> >();
//Eigen::Vector3d translation(vtran.data());
//
cv::Mat KMat(3, 3, cv::DataType<double>::type);
cv::Mat rotationMat(3, 1, cv::DataType<double>::type);
cv::Mat translationMat(3, 1, cv::DataType<double>::type);
cv::Mat DMat(5, 1, cv::DataType<double>::type);
//

std::vector<Eigen::Matrix<int, 2, 1>> cvProjection(const std::vector<Eigen::Matrix<double, 3, 1>> &pos_lidar, const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K, const Eigen::Matrix<double, 5, 1> &D, const Eigen::Quaternion<double> &quaternion, const Eigen::Vector3d &translation) {
    std::vector<cv::Point3d> points(pos_lidar.size());
    for (int i = 0; i < pos_lidar.size(); i++) {
        points.push_back(cv::Point3d(pos_lidar[i].x(), pos_lidar[i].y(), pos_lidar[i].z()));
//        std::cout << "point:" << pos_lidar[i] << std::endl;
    }
    cv::eigen2cv(K, KMat);
    cv::eigen2cv(D, DMat);
    Eigen::Vector3d rotvec = {3.084, -1.314, -1.334};
    cv::eigen2cv(rotvec, rotationMat);
    cv::eigen2cv(translation, translationMat);
//    std::cout << "K" << KMat << "   D" << DMat << "  quaternion" << rotationMat << std::endl;
    std::vector<cv::Point2d> imagePointsDouble(pos_lidar.size());
    cv::projectPoints(points, rotationMat, translationMat, KMat, DMat, imagePointsDouble);
    for (int i = 0; i < imagePointsDouble.size(); i++) {
        std::cout << "x:" << imagePointsDouble[i].x << "y:" << imagePointsDouble[i].y << std::endl;
    }
//    std::vector<cv::Point2i> imagePointscv(imagePointsDouble.size());
//    for (int i = 0; i < imagePointsDouble.size(); i++) {
//        imagePointscv.push_back(cv::Point2i(round(imagePointsDouble[i].x / 2.5), round(imagePointsDouble[i].y / 2.5)));
//    }
    std::vector<Eigen::Vector2i> imagePoints(imagePointsDouble.size());
//    cv::cv2eigen(imagePointscv, imagePoints);
    for (int i = 0; i < imagePointsDouble.size(); i++) {
        imagePoints.push_back(Eigen::Vector2i(ceil(imagePointsDouble[i].x / 2.5), ceil(imagePointsDouble[i].y / 2.5)));
//        std::cout << "x:" << imagePoints[i].x() << "y:" << imagePoints[i].y() << std::endl;
    }
    return imagePoints;

}
//
////std::vector<cv::Point3d> Generate3DPoints(){}
