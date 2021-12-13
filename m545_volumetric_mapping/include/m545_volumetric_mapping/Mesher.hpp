/*
 * Mesher.hpp
 *
 *  Created on: Oct 8, 2021
 *      Author: jelavice
 */

#pragma once

#include <mutex>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "m545_volumetric_mapping/Mapper.hpp"


namespace m545_mapping {


    class Mesher {

    public:

        using PointCloud = open3d::geometry::PointCloud;
        using TriangleMesh = open3d::geometry::TriangleMesh;

        Mesher();
        ~Mesher() = default;
        void buildMeshFromCloud(const PointCloud &cloud);
        bool isMeshingInProgress() const;
        void setParameters(const MesherParameters &p, const MesherNewParams &p2);
        const TriangleMesh &getMesh() const;
        void setCurrentPose(const Eigen::Isometry3d &pose);
        const PointCloud &getMeshMap() const;


    private:
        bool isMeshingInProgress_ = false;
        std::mutex meshingMutex_;
        mutable std::mutex meshingAccessMutex_;
        std::shared_ptr<TriangleMesh> mesh_;
        MesherParameters params_;
        MesherNewParams mesherNewParams_;
        Eigen::Isometry3d currentPose_ = Eigen::Isometry3d::Identity();
        open3d::geometry::PointCloud prevMeshMap_;
        open3d::geometry::PointCloud prevMap_1;
        open3d::geometry::PointCloud prevMap_2;
        std::shared_ptr<open3d::geometry::PointCloud> differenceMapPtr_;
        void computeIndicesOfOverlappingPoints(const open3d::geometry::PointCloud &source,
                                               const open3d::geometry::PointCloud &target, const Eigen::Isometry3d &sourceToTarget, double voxelSize,
                                               size_t minNumPointsPerVoxel,
                                               std::vector<size_t> *idxsSource, std::vector<size_t> *idxsTarget);
        PointCloud cloud_;


    };



} // namespace m545_mapping
