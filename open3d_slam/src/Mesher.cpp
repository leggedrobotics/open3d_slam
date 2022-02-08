/*
 * Mesher.cpp
 *
 *  Created on: Oct 8, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Mesher.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/math.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <ros/package.h>

namespace o3d_slam {

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
		for (int i = 0; i < (int) mesh->vertices_.size(); ++i) {
			const double d = densities.at(i);
			if (d < removalThreshold) {
				idsToRemove.push_back(i);
			}
		}
		mesh->RemoveVerticesByIndex(idsToRemove);
//		std::cout<<"Density mean: " <<calcMean(densities) << "\n";
//		std::cout<<"Density std: " <<calcStandardDeviation(densities) << "\n\n";
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
//	const std::string filename = ros::package::getPath("open3d_slam") + "/data/map_mesh.stl";
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
	return *mesh_;
}

} // namespace o3d_slam
