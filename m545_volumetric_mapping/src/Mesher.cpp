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

namespace m545_mapping {

Mesher::Mesher(){
	mesh_ = std::make_shared<TriangleMesh>();
}

void Mesher::setCurrentPose(const Eigen::Isometry3d &pose){
	currentPose_ = pose;
}

void Mesher::buildMeshFromCloud(const PointCloud &cloudIn) {
	std::lock_guard<std::mutex> lck(meshingMutex_);
	isMeshingInProgress_ = true;
	auto cloud = cloudIn;
	Timer timer("mesh_construction");
	if (!cloud.HasNormals()) {
		Timer timer("mesher_normal_est");
		cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(params_.knnNormalEstimation_));
		cloud.OrientNormalsConsistentTangentPlane(params_.knnNormalEstimation_);
	}
//	cloud.OrientNormalsToAlignWithDirection(Eigen::Vector3d::UnitZ());
	cloud.OrientNormalsTowardsCameraLocation(currentPose_.translation());

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
		for (int i = 0; i < (int) mesh->vertices_.size(); ++i) {
			const double d = densities.at(i);
            double threshold = 0.0001;
            for (int j = 0; j < cloud.points_.size(); j++)
                if ((mesh->vertices_[i] - cloud.points_[j]).norm() < threshold)
                    mesh->vertex_colors_[i] = cloud.colors_[j];
			if (d < removalThreshold) {
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
		mesh_ = mesh;
    }

//	mesh->ComputeTriangleNormals();
//	const std::string filename = ros::package::getPath("m545_volumetric_mapping") + "/data/map_mesh.stl";
//	open3d::io::WriteTriangleMeshToSTL(filename, *mesh, false, false, false, false, false, false);



	isMeshingInProgress_ = false;

}

bool Mesher::isMeshingInProgress() const {
	return isMeshingInProgress_;
}

void Mesher::setParameters(const MesherParameters &p) {
	params_ = p;
}

const Mesher::TriangleMesh &Mesher::getMesh() const{
	std::lock_guard<std::mutex> lck(meshingAccessMutex_);
//    Eigen::Vector3d grey{0.5, 0.5, 0.5};
//    for(int i = 0; i < mesh_->vertices_.size(); i++)
//        mesh_->vertex_colors_.at(i) = grey;
    return *mesh_;
}

} // namespace m545_mapping
