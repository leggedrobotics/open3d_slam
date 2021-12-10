/*
 * Mesher.cpp
 *
 *  Created on: Oct 8, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include <open3d/io/TriangleMeshIO.h>
#include <ros/package.h>
#include "open3d_conversions/open3d_conversions.h"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/Voxel.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"

namespace m545_mapping {

    Mesher::Mesher(){
        mesh_ = std::make_shared<TriangleMesh>();
    }

    void Mesher::setCurrentPose(const Eigen::Isometry3d &pose){
        currentPose_ = pose;
    }

    void Mesher::buildMeshFromCloud(const PointCloud &cloudIn) {

//        std::shared_ptr<m545_mapping::Mapper> mapper;
//        mapper = std::make_shared<m545_mapping::Mapper>();

        std::lock_guard<std::mutex> lck(meshingMutex_);
        isMeshingInProgress_ = true;

        std::vector<size_t> idxsSource;
        std::vector<size_t> idxsTarget;

//        try remove outliers here
        std::shared_ptr<open3d::geometry::PointCloud> cl,cl2;
        std::vector<size_t> idx;
////        std::tie(cl, idx) = cloudIn.RemoveRadiusOutliers(10, 0.1);
//        std::tie(cl, idx) = cloudIn.RemoveStatisticalOutliers(10, 2);
//        auto select_cloud = cloudIn.SelectByIndex(idx);
//        auto newCloudIn = *select_cloud;
        open3d::geometry::PointCloud cloud;
        auto prev_mean = std::get<0>(prevMeshMap_.ComputeMeanAndCovariance());
        auto new_mean = std::get<0>(cloudIn.ComputeMeanAndCovariance());
        if ((prev_mean - new_mean).norm() > paramsInMesher_.computeOverlappingThreshold) {
            computeIndicesOfOverlappingPoints(prevMeshMap_, cloudIn, Eigen::Isometry3d::Identity(), paramsInMesher_.overlapVoxelSize, paramsInMesher_.overlapMinPoints, &idxsSource, &idxsTarget);
            differenceMapPtr_ = cloudIn.SelectByIndex(idxsTarget, true);
            cloud = *differenceMapPtr_;
            prevMeshMap_ = cloudIn;
        }
        else {
            if (differenceMapPtr_->HasPoints())
                cloud = *differenceMapPtr_;
            else
                cloud = cloudIn;
        }
//        Eigen::Vector3d red = {1, 0, 0};
//        Eigen::Vector3d green = {0, 1, 0};
//        Eigen::Vector3d blue = {0, 0, 1};
//        auto cloudInTrans = cloudIn;
//        auto cloudForVisualization = cloud;
//        std::tie(cl, idx) = cloud.RemoveRadiusOutliers(paramsInMesher_.radiusOutlierNbPoints, paramsInMesher_.radiusOutlierRadius);
//        std::tie(cl2, idx) = cl->RemoveRadiusOutliers(paramsInMesher_.statisticalOutlierNbPoints, paramsInMesher_.statisticalOutlierRatio);

//        auto cloudForVisualization = prevMeshMap_.PaintUniformColor(red) + cloudInTrans.PaintUniformColor(green) + cloud.PaintUniformColor(blue);
//        std::cout << "size cloudIn" << cloudIn.points_.size() << "   size cloud   " << cloud.points_.size() << "size idxs   " << idxsTarget.size() << std::endl;

//        auto cloud = cloudIn;
        Timer timer("mesh_construction");
        if (!cloud.HasNormals()) {
            Timer timer("mesher_normal_est");
            cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(params_.knnNormalEstimation_));
            cloud.OrientNormalsConsistentTangentPlane(params_.knnNormalEstimation_);
        }
//	cloud.OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());
        cloud.OrientNormalsTowardsCameraLocation(currentPose_.translation());
//        cloud_ = cloud;

        auto mesh = std::make_shared<TriangleMesh>();

        switch (params_.strategy_) {
            case MesherStrategy::AlphaShape: {
                auto mesh = TriangleMesh::CreateFromPointCloudAlphaShape(cloud, params_.alphaShapeAlpha_);
                break;
            }
            case MesherStrategy::BallPivot: { // slow AF
                mesh = TriangleMesh::CreateFromPointCloudBallPivoting(cloud, params_.ballPivotRadii_);
                break;
            }
            case MesherStrategy::Poisson: {
                int dummyWidth = 0;
                auto meshAndDensities = TriangleMesh::CreateFromPointCloudPoisson(cloud, params_.poissonDepth_, dummyWidth,
                                                                                  params_.poissonScale_);
                mesh = std::get<0>(meshAndDensities);
                auto densities = std::get<1>(meshAndDensities);
                std::vector<size_t> idsToRemove;
                idsToRemove.reserve(mesh->vertices_.size());
                const double removalThreshold = calcMean(densities) + params_.poissonMinDensity_*calcStandardDeviation(densities);
                //color the mesh here, based on that poisson mesh and the pointcloud share the same points
                double aver_density = 0.0;
//                for (int i = 0; i < (int) mesh->vertices_.size(); ++i) {
//                    const double d = densities.at(i);
////                    std::cout << "density" << i << ":  " << d << std::endl;
//                    aver_density += d;
//
//                }
//                aver_density /= mesh->vertices_.size();
//                std::cout << "average density:" << aver_density << std::endl;
                for (int i = 0; i < (int) mesh->vertices_.size(); ++i) {
                    const double d = densities.at(i);
                    if (d < paramsInMesher_.densityThreshold) {
                        idsToRemove.push_back(i);
                    }
                }
                mesh->RemoveVerticesByIndex(idsToRemove);
                break;
            }
            default:
                throw std::runtime_error("Unknown reconstruction strategy");
        }

        {
            std::lock_guard<std::mutex> lck(meshingAccessMutex_);
            open3d::geometry::PointCloud meshCloud;
            for (int i = 0; i < mesh->vertices_.size(); i++) {
                meshCloud.points_.push_back(mesh->vertices_.at(i));
                meshCloud.colors_.push_back(mesh->vertex_colors_.at(i));
            }
            *mesh_ += *mesh;
//        return cloud_;
//        Eigen::Vector3d blue = {0, 0, 1};
//        meshCloud.PaintUniformColor(blue);
            cloud_ = meshCloud;
        }

//	mesh->ComputeTriangleNormals();
//	const std::string filename = ros::package::getPath("m545_volumetric_mapping") + "/data/map_mesh.stl";
//	open3d::io::WriteTriangleMeshToSTL(filename, *mesh, false, false, false, false, false, false);


//        prevMeshMap_ = cloudIn;
        isMeshingInProgress_ = false;
    }

    bool Mesher::isMeshingInProgress() const {
        return isMeshingInProgress_;
    }

    void Mesher::setParameters(const MesherParameters &p, const MesherParamsInMesher &p2) {
        params_ = p;
        paramsInMesher_ = p2;
    }

    const Mesher::TriangleMesh &Mesher::getMesh() const{
        std::lock_guard<std::mutex> lck(meshingAccessMutex_);
        return *mesh_;
    }

    void Mesher::computeIndicesOfOverlappingPoints(const open3d::geometry::PointCloud &source,
                                                   const open3d::geometry::PointCloud &target, const Eigen::Isometry3d &sourceToTarget, double voxelSize,
                                                   size_t minNumPointsPerVoxel,
                                                   std::vector<size_t> *idxsSource, std::vector<size_t> *idxsTarget) {
        assert_ge<size_t>(minNumPointsPerVoxel,1);
        VoxelMap targetMap(Eigen::Vector3d::Constant(voxelSize));
        targetMap.buildFromCloud(target);
        VoxelMap sourceMap(Eigen::Vector3d::Constant(voxelSize));
        sourceMap.buildFromCloud(source);

        idxsSource->clear();
        idxsSource->reserve(source.points_.size());
        idxsTarget->clear();
        idxsTarget->reserve(target.points_.size());
        std::set<size_t> setTargetIdxs;
        for (size_t i = 0; i < source.points_.size(); ++i) {
            const auto p = sourceToTarget * source.points_.at(i);
            const auto targetIdxsInVoxel = targetMap.getIndicesInVoxel(p);
            if (targetIdxsInVoxel.size() >= minNumPointsPerVoxel) {
                setTargetIdxs.insert(targetIdxsInVoxel.begin(), targetIdxsInVoxel.end());
                idxsSource->push_back(i);
            }
        }
//        for (size_t i = 0; i < target.points_.size(); ++i) {
//            const auto p_t = target.points_.at(i);
//            const auto sourceIdxsInVoxel = sourceMap.getIndicesInVoxel(p_t);
//            if (sourceIdxsInVoxel.size() >= minNumPointsPerVoxel) {
//                if (std::find(setTargetIdxs.begin(), setTargetIdxs.end(), i) == setTargetIdxs.end()) {
//                    setTargetIdxs.insert(i);
//                }
//            }
//        }
        idxsTarget->insert(idxsTarget->end(), setTargetIdxs.begin(), setTargetIdxs.end());
    }

    const Mesher::PointCloud& Mesher::getMeshMap() const{
        std::lock_guard<std::mutex> lck(meshingAccessMutex_);

        return cloud_;
    }

} // namespace m545_mapping
