//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/eigen.hpp"
//
//#include <iostream>
//#include <string>
//#include <yaml-cpp/yaml.h>
//
////std::vector<cv::Point3d> point;
////std::vector<cv::Point2d> imagePoint;
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
//cv::Mat KMat(3, 3, cv::DataType<double>::type);
//cv::Mat rotationMat(3, 1, cv::DataType<double>::type);
//cv::Mat translationMat(3, 1, cv::DataType<double>::type);
//cv::Mat DMat(5, 1, cv::DataType<double>::type);
//
//std::vector<cv::Point2i> cvProjection(const std::vector<cv::Point3d> &points) {
//
//    cv::eigen2cv(K, KMat);
//    cv::eigen2cv(D, DMat);
//    cv::eigen2cv(rotation, rotationMat);
//    cv::eigen2cv(translation, translationMat);
//    std::vector<cv::Point2i> imagePoints;
//    cv::projectPoints(points, rotationMat, translationMat, KMat, DMat, imagePoints);
//    return imagePoints;
//
//}
//
////std::vector<cv::Point3d> Generate3DPoints(){}
