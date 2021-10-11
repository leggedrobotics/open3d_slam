#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>


Eigen::Vector4d pos_lidar_aux;
Eigen::Vector3d pos_udimage_aux;

const Eigen::Matrix3d K;
const Eigen::VectorXd D(5);
const Eigen::Quaterniond quaternion;
const Eigen::Vector3d translation;

Eigen::Matrix3d rotation;
Eigen::MatrixXd RT(3,4);

// void lidarToCamera(const Eigen::Vector3d &pos_lidar, const Eigen::Quaterniond &quaternion, const Eigen::Vector3d &translation) {
//     rotation = quaternion.toRotationMatrix();
//     RT.leftCols(3) = rotation;
//     RT.col(3) = translation;
//     pos_lidar_aux.topRows(3) = pos_lidar;
//     pos_lidar_aux(3) = aux;
//     // pos_camera_aux.topRows(3) = pos_camera;
//     // pos_camera_aux[3] = aux;
//     pos_camera_aux = RT * pos_lidar_aux;
// }

// void cameraToPixel(Eigen::Vector4d pos_camera_aux, const Eigen::VectorXd D) {
//     pos_udimage_aux = K * pos_camera_aux;
//     pos_udimage_aux = pos_udimage_aux / pos_udimage_aux(2);
//     pos_udimage = pos_udimage_aux.topRows(2);
// }

// void distortion(const Eigen::Matrix3d K, const Eigen::Matrix3d D) {
//     double u0 = K(0,2);
//     double v0 = K(1,2);
//     double u_differ = pos_udimage(0) - u0;
//     double v_differ = pos_udimage(1) - v0;
//     double r_square = pow(u_differ,2) + pow(v_differ,2);
//     pos_dimage(0) = round((1 + D(0)*r_square + D(1)*pow(r_square,2) + D(4)*pow(r_square,3)) * u_differ + 2*D(2)*u_differ*v_differ + D(3)*(r_square+2*pow(u_differ,2)) + u0);
//     pos_dimage(1) = round((1 + D(0)*r_square + D(1)*pow(r_square,2) + D(4)*pow(r_square,3)) * v_differ + 2*D(3)*u_differ*v_differ + D(2)*(r_square+2*pow(v_differ,2)) + v0);
//     // pos_dimage(0) = round(pos_dimage(0));
//     // pos_dimage(1) = round(pos_dimage(1));
// }

void projectionLidarToPixel(const Eigen::Vector3d &pos_lidar, const Eigen::Quaterniond &quaternion, const Eigen::Vector3d &translation, Eigen::Vector2i &pos_dimage) {
    rotation = quaternion.toRotationMatrix();
    RT.leftCols(3) = rotation;
    RT.col(3) = translation;
    pos_lidar_aux.topRows(3) = pos_lidar;
    pos_lidar_aux(3) = 1.0;
    pos_udimage_aux = K * RT * pos_lidar_aux;
    pos_udimage_aux = pos_udimage_aux / pos_udimage_aux(2);
    double u0 = K(0,2);
    double v0 = K(1,2);
    double u_differ = pos_udimage_aux(0) - u0;
    double v_differ = pos_udimage_aux(1) - v0;
    double r_square = pow(u_differ,2) + pow(v_differ,2);
    pos_dimage(0) = round((1 + D(0)*r_square + D(1)*pow(r_square,2) + D(4)*pow(r_square,3)) * u_differ + 
                    2*D(2)*u_differ*v_differ + D(3)*(r_square+2*pow(u_differ,2)) + u0);
    pos_dimage(1) = round((1 + D(0)*r_square + D(1)*pow(r_square,2) + D(4)*pow(r_square,3)) * v_differ + 
                    2*D(3)*u_differ*v_differ + D(2)*(r_square+2*pow(v_differ,2)) + v0);
}