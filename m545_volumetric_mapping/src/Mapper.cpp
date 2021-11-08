/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mapper.hpp"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace m545_mapping {

    namespace {
        namespace registration = open3d::pipelines::registration;
        std::shared_ptr<registration::TransformationEstimation> icpObjective;
    } // namespace

    Mapper::Mapper() :
            tfListener_(tfBuffer_) {
        update(params_);
    }

    void Mapper::setParameters(const MapperParameters &p) {
        params_ = p;
        update(p);
    }

    void Mapper::update(const MapperParameters &p) {
        icpCriteria_.max_iteration_ = params_.maxNumIter_;
        icpObjective = icpObjectiveFactory(params_.icpObjective_);
    }

    bool Mapper::isMatchingInProgress() const {
        return isMatchingInProgress_;
    }

    Eigen::Isometry3d Mapper::getMapToOdom() const {
        return mapToOdom_;
    }
    Eigen::Isometry3d Mapper::getMapToRangeSensor() const {
        return mapToRangeSensor_;
    }

    void Mapper::estimateNormalsIfNeeded(PointCloud *pcl) const {
        if (params_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
            estimateNormals(params_.kNNnormalEstimation_, pcl);
            pcl->NormalizeNormals();
        }
    }

    void Mapper::addRangeMeasurement(const Mapper::PointCloud &cloudIn, const ros::Time &timestamp) {
        isMatchingInProgress_ = true;
        lastMeasurementTimestamp_ = timestamp;
        //insert first scan
        if (map_.points_.empty()) {
            auto cloud = cloudIn;
            auto bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_, params_.mapBuilderCropBoxHighBound_);
            auto croppedCloud = cloud.Crop(bbox);
            m545_mapping::voxelize(params_.voxelSize_, croppedCloud.get());

//		auto voxelizedCloud = croppedCloud->VoxelDownSample(params_.mapVoxelSize_);
            estimateNormalsIfNeeded(croppedCloud.get());
            map_ += *croppedCloud;
            denseMap_ += cloud;
            isMatchingInProgress_ = false;
            return;
        }

        Eigen::Isometry3d odomToRangeSensor;
        const bool lookupStatus = lookupTransform(frames::odomFrame, frames::rangeSensorFrame, timestamp,
                                                  &odomToRangeSensor);
        const auto odometryMotion = odomToRangeSensorPrev_.inverse() * odomToRangeSensor;
        //todo check rotation and trans
        if (!lookupStatus) {
            isMatchingInProgress_ = false;
            return;
        }
        auto cloud = cloudIn;
        Timer timer;
        open3d::geometry::AxisAlignedBoundingBox bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_,
                                                                                  params_.mapBuilderCropBoxHighBound_);
        auto wideCroppedCloud = cloud.Crop(bbox);

//	std::cout << "Cropping input cloud finished\n";
//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";
//	auto voxelizedCloud = croppedCloud->VoxelDownSample(params_.voxelSize_);
        {
//		Timer timer("voxelize_input_cloud");
            m545_mapping::voxelize(params_.voxelSize_, wideCroppedCloud.get());

        }

        bbox = boundingBoxAroundPosition(params_.cropBoxLowBound_, params_.cropBoxHighBound_);
        auto narrowCropped = wideCroppedCloud->Crop(bbox);
        m545_mapping::randomDownSample(params_.downSamplingRatio_, narrowCropped.get());

        bbox = boundingBoxAroundPosition(1.4 * params_.cropBoxLowBound_, 1.4 * params_.cropBoxHighBound_,
                                         mapToRangeSensor_.translation());

        auto mapPatch = map_.Crop(bbox);

//	std::cout << "Map and scan pre processing finished\n";
//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";

        Timer timer2;
//	const Eigen::Matrix4d initTransform = (mapToRangeSensor_).matrix();
        const Eigen::Matrix4d initTransform = (mapToOdom_ * odomToRangeSensor).matrix();
        const auto result = open3d::pipelines::registration::RegistrationICP(*narrowCropped, *mapPatch,
                                                                             params_.maxCorrespondenceDistance_, initTransform, *icpObjective, icpCriteria_);

//	std::cout << "Scan to map matching finished \n";
//	std::cout << "Time elapsed: " << timer2.elapsedMsec() << " msec \n";
//	std::cout << "fitness: " << result.fitness_ << std::endl;

        if (result.fitness_ < params_.minRefinementFitness_) {
            std::cout << "Skipping the refinement step, fitness: " << result.fitness_ << std::endl;
            isMatchingInProgress_ = false;
            return;
        }

        // update transforms
        mapToRangeSensor_.matrix() = result.transformation_;
        mapToOdom_ = mapToRangeSensor_ * odomToRangeSensor.inverse();

        // concatenate registered cloud into map
        const bool isMovedTooLittle = odometryMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
        if (!isMovedTooLittle) {
            std::lock_guard<std::mutex> lck(mapManipulationMutex_);
            isManipulatingMap_ = true;
//		Timer timer("Map update");
            m545_mapping::randomDownSample(params_.downSamplingRatio_, wideCroppedCloud.get());
            wideCroppedCloud->Transform(result.transformation_);
            estimateNormalsIfNeeded(wideCroppedCloud.get());
            map_ += *wideCroppedCloud;

            if (params_.mapVoxelSize_ > 0.0) {
//			Timer timer("voxelize_map",true);
                bbox = boundingBoxAroundPosition(params_.mapBuilderCropBoxLowBound_, params_.mapBuilderCropBoxHighBound_,
                                                 mapToRangeSensor_.translation());
                auto voxelizedMap = voxelizeAroundPosition(params_.mapVoxelSize_, bbox, map_);
                map_ = *voxelizedMap;
            }
            {
//			Timer timer("merge_dense_map");
                denseMap_ += cloud.Transform(result.transformation_);
            }

//		std::cout << "\n";
            isManipulatingMap_ = false;
            odomToRangeSensorPrev_ = odomToRangeSensor;
        }

        isMatchingInProgress_ = false;
    }
    const Mapper::PointCloud& Mapper::getMap() const {
        return map_;
    }

    const Mapper::PointCloud& Mapper::getDenseMap() const {
        return denseMap_;
    }

    bool Mapper::lookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time,
                                 Eigen::Isometry3d *transform) const {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame, time);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("caught exception while looking up the tf: %s", ex.what());
            *transform = Eigen::Isometry3d::Identity();
            return false;
        }

        *transform = tf2::transformToEigen(transformStamped);
        return true;
    }

    bool Mapper::isManipulatingMap() const {
        return isManipulatingMap_;
    }

    Mapper::PointCloud* Mapper::getMapPtr() {
        return &map_;
    }
    Mapper::PointCloud* Mapper::getDenseMapPtr() {
        return &denseMap_;
    }

} /* namespace m545_mapping */
