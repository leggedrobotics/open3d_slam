/*
 * helpers.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/output.hpp"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>

// ros stuff
#include "open3d_conversions/open3d_conversions.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {

    namespace {
        namespace registration = open3d::pipelines::registration;

        class AccumulatedPoint {
        public:
            AccumulatedPoint() :
                    num_of_points_(0), point_(0.0, 0.0, 0.0), normal_(0.0, 0.0, 0.0), color_(0.0, 0.0, 0.0) {
            }

        public:
            void AddPoint(const open3d::geometry::PointCloud &cloud, int index) {
                point_ += cloud.points_[index];
                if (cloud.HasNormals()) {
                    if (!std::isnan(cloud.normals_[index](0)) && !std::isnan(cloud.normals_[index](1))
                        && !std::isnan(cloud.normals_[index](2))) {
                        normal_ += cloud.normals_[index];
                    }
                }
                if (cloud.HasColors()) {
                    color_ += cloud.colors_[index];
                }
                num_of_points_++;
            }

            Eigen::Vector3d GetAveragePoint() const {
                return point_ / double(num_of_points_);
            }

            Eigen::Vector3d GetAverageNormal() const {
                // Call NormalizeNormals() afterwards if necessary
                return normal_ / double(num_of_points_);
            }

            Eigen::Vector3d GetAverageColor() const {
                return color_ / double(num_of_points_);
            }

        public:
            int num_of_points_;
            Eigen::Vector3d point_;
            Eigen::Vector3d normal_;
            Eigen::Vector3d color_;
        };

        class point_cubic_id {
        public:
            size_t point_id;
            int cubic_id;
        };

    } //namespace

    double calcMean(const std::vector<double> &data) {
        if (data.empty()) {
            return 0.0;
        }
        return static_cast<double>(std::accumulate(data.begin(), data.end(), 0.0)) / data.size();
    }

    double calcStandardDeviation(const std::vector<double> &data) {
        const int n = data.size();
        if (n < 2) {
            return 0.0;
        }
        const double mean = calcMean(data);
        double stdDev = 0.0;
        for (const auto d : data) {
            const double e = d - mean;
            stdDev += e * e;
        }

        return std::sqrt(stdDev / (n - 1));
    }

    void publishMesh(const open3d::geometry::MeshBase &mesh, const std::string &frame_id, const ros::Time &timestamp,
                     ros::Publisher &pub) {
        if (pub.getNumSubscribers() > 0) {
            m545_volumetric_mapping_msgs::PolygonMesh meshMsg;
            open3d::geometry::PointCloud pointcloud;
            open3d_conversions::open3dToRos(mesh, frame_id, meshMsg);
            meshMsg.header.frame_id = frame_id;
            meshMsg.header.stamp = timestamp;
            //recolor here
            pointcloud.points_ = mesh.vertices_;
            pointcloud.colors_ = mesh.vertex_colors_;
            std::cout << "points in final cloud" << pointcloud.points_.size() << std::endl;
            open3d_conversions::open3dToRos(pointcloud, meshMsg.cloud, frame_id);
//            auto color_it = std::find_if(meshMsg.cloud.fields.begin(), meshMsg.cloud.fields.end(), [](const sensor_msgs::PointField &field) {
//                return field.name == "rgb";});
//            if (color_it == meshMsg.cloud.fields.end())
//                std::cout << "sorry no color here" << std::endl;
//            else
//                std::cout << "works here" << std::endl;
            pub.publish(meshMsg);
        }
    }

    void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,
                      ros::Publisher &pub) {
        if (pub.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 msg;
            open3d_conversions::open3dToRos(cloud, msg, frame_id);
            msg.header.stamp = timestamp;
            pub.publish(msg);
        }
    }

    geometry_msgs::Pose getPose(const Eigen::MatrixXd &T) {
        geometry_msgs::Pose pose;

        // Fill pose
        Eigen::Affine3d eigenTr;
        eigenTr.matrix() = T;
        tf::poseEigenToMsg(eigenTr, pose);

        return pose;
    }

    geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
                                          const std::string &childFrame) {

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = time;
        transformStamped.header.frame_id = frame;
        transformStamped.child_frame_id = childFrame;
        const auto pose = getPose(Mat);
        transformStamped.transform.translation.x = pose.position.x;
        transformStamped.transform.translation.y = pose.position.y;
        transformStamped.transform.translation.z = pose.position.z;
        transformStamped.transform.rotation = pose.orientation;
        return transformStamped;
    }

    void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl) {
        auto croppedCloud = pcl->Crop(bbox);
        *pcl = *croppedCloud;
    }

    void cropMesh(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::TriangleMesh *mesh) {
        auto croppedMesh = mesh->Crop(bbox);
        *mesh = *croppedMesh;
    }

    std::string asString(const Eigen::Isometry3d &T) {
        const double kRadToDeg = 180.0 / M_PI;
        const auto &t = T.translation();
        const auto &q = Eigen::Quaterniond(T.rotation());
        const std::string trans = string_format("t:[%f, %f, %f]", t.x(), t.y(), t.z());
        const std::string rot = string_format("q:[%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());
        const auto rpy = toRPY(q) * kRadToDeg;
        const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]", rpy.x(), rpy.y(), rpy.z());
        return trans + " ; " + rot + " ; " + rpyString;

    }

    Eigen::Quaterniond fromRPY(const double roll, const double pitch, const double yaw) {

        const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
        return yaw_angle * pitch_angle * roll_angle;
    }

    Eigen::Vector3d toRPY(const Eigen::Quaterniond &_q) {
        Eigen::Quaterniond q(_q);
        q.normalize();
        const double r = getRollFromQuat(q.w(), q.x(), q.y(), q.z());
        const double p = getPitchFromQuat(q.w(), q.x(), q.y(), q.z());
        const double y = getYawFromQuat(q.w(), q.x(), q.y(), q.z());
        return Eigen::Vector3d(r, p, y);
    }

    Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy) {
        return fromRPY(rpy.x(), rpy.y(), rpy.z());
    }

    void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl) {
        open3d::geometry::KDTreeSearchParamKNN param(numNearestNeighbours);
        pcl->EstimateNormals(param);
    }

    std::shared_ptr<registration::TransformationEstimation> icpObjectiveFactory(const m545_mapping::IcpObjective &obj) {

        switch (obj) {
            case m545_mapping::IcpObjective::PointToPoint: {
                auto obj = std::make_shared<registration::TransformationEstimationPointToPoint>(false);
                return obj;
            }

            case m545_mapping::IcpObjective::PointToPlane: {
                auto obj = std::make_shared<registration::TransformationEstimationPointToPlane>();
                return obj;
            }

            default:
                throw std::runtime_error("Unknown icp objective");
        }

    }
    Timer::Timer() :
            Timer(false, "") {

    }

    Timer::Timer(const std::string &name) :
            Timer(true, name) {
    }
    Timer::Timer(bool isPrintInDestructor, const std::string &name) {
        startTime_ = std::chrono::steady_clock::now();
        isPrintInDestructor_ = isPrintInDestructor;
        name_ = name;
    }
    Timer::~Timer() {
        if (isPrintInDestructor_) {
            std::cout << "Timer " << name_ << ": Elapsed time: " << elapsedMsec() << " msec \n";
        }
    }

    open3d::geometry::AxisAlignedBoundingBox boundingBoxAroundPosition(const Eigen::Vector3d &low,
                                                                       const Eigen::Vector3d &high, const Eigen::Vector3d &origin /*= Eigen::Vector3d::Zero()*/) {
        open3d::geometry::AxisAlignedBoundingBox bbox;
        bbox.min_bound_ = origin + low;
        bbox.max_bound_ = origin + high;
        return bbox;
    }

    double Timer::elapsedMsec() const {
        const auto endTime = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count() / 1e3;
    }
    double Timer::elapsedSec() const {
        const auto endTime = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime_).count() / 1e3;
    }

    void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud *pcl) {
        if (downSamplingRatio >= 1.0) {
            return;
        }
        auto downSampled = pcl->RandomDownSample(downSamplingRatio);
        *pcl = *downSampled;
    }
    void voxelize(double voxelSize, open3d::geometry::PointCloud *pcl) {
        if (voxelSize <= 0) {
            return;
        }
        auto voxelized = pcl->VoxelDownSample(voxelSize);
        *pcl = *voxelized;
    }

    void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
                            const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster) {
        geometry_msgs::TransformStamped transformStamped = m545_mapping::toRos(Mat, time, frame, childFrame);
        broadcaster->sendTransform(transformStamped);
    }

    std::shared_ptr<open3d::geometry::PointCloud> voxelizeAroundPosition(double voxel_size,
                                                                         const open3d::geometry::AxisAlignedBoundingBox &bbox, const open3d::geometry::PointCloud &cloud) {
        using namespace open3d::geometry;
        auto output = std::make_shared<PointCloud>();
        if (voxel_size <= 0.0) {
            *output = cloud;
            return output;
//		throw std::runtime_error("[VoxelDownSample] voxel_size <= 0.");
        }
        Eigen::Vector3d voxel_size3 = Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
        Eigen::Vector3d voxel_min_bound = cloud.GetMinBound() - voxel_size3 * 0.5;
        Eigen::Vector3d voxel_max_bound = cloud.GetMaxBound() + voxel_size3 * 0.5;
        if (voxel_size * std::numeric_limits<int>::max() < (voxel_max_bound - voxel_min_bound).maxCoeff()) {
            throw std::runtime_error("[VoxelDownSample] voxel_size is too small.");
        }
        std::unordered_map<Eigen::Vector3i, AccumulatedPoint, open3d::utility::hash_eigen<Eigen::Vector3i>> voxelindex_to_accpoint;

        const bool has_normals = cloud.HasNormals();
        const bool has_colors = cloud.HasColors();
        output->points_.reserve(cloud.points_.size());
        if (has_colors) {
            output->colors_.reserve(cloud.points_.size());
        }
        if (has_normals) {
            output->normals_.reserve(cloud.points_.size());
        }
        voxelindex_to_accpoint.reserve(cloud.points_.size());

        auto isInside = [&bbox](const Eigen::Vector3d &p) {
            return p.x() <= bbox.max_bound_.x() && p.y() <= bbox.max_bound_.y() && p.z() <= bbox.max_bound_.z()
                   && p.x() >= bbox.min_bound_.x() && p.y() >= bbox.min_bound_.y() && p.z() >= bbox.min_bound_.z();
        };

        Eigen::Vector3d ref_coord;
        Eigen::Vector3i voxel_index;
        for (int i = 0; i < (int) cloud.points_.size(); i++) {
            if (isInside(cloud.points_[i])) {
                ref_coord = (cloud.points_[i] - voxel_min_bound) / voxel_size;
                voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))), int(floor(ref_coord(2)));
                voxelindex_to_accpoint[voxel_index].AddPoint(cloud, i);
            } else {
                output->points_.push_back(cloud.points_[i]);
                if (has_normals) {
                    output->normals_.push_back(cloud.normals_[i]);
                }
                if (has_colors) {
                    output->colors_.push_back(cloud.colors_[i]);
                }
            }
        }

        for (auto accpoint : voxelindex_to_accpoint) {
            output->points_.push_back(accpoint.second.GetAveragePoint());
            if (has_normals) {
                output->normals_.push_back(accpoint.second.GetAverageNormal());
            }
            if (has_colors) {
                output->colors_.push_back(accpoint.second.GetAverageColor());
            }
        }

        return output;
    }

} /* namespace m545_mapping */

