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

namespace m545_mapping {

Mesher::Mesher(){
	mesh_ = std::make_shared<TriangleMesh>();
}

void Mesher::buildMeshFromCloud(const PointCloud &cloudIn) {
	std::lock_guard<std::mutex> lck(meshingMutex_);
	isMeshingInProgress_ = true;
	auto cloud = cloudIn;
	Timer timer("mesh_construction");
	if (!cloud.HasNormals()) {
		cloud.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(params_.knnNormalEstimation_));
	}
	auto mesh = std::make_shared<TriangleMesh>();

	switch (params_.strategy_) {
	case MesherStrategy::AlphaShape: {
		mesh = TriangleMesh::CreateFromPointCloudAlphaShape(cloud, params_.alphaShapeAlpha_);
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
		for (int i = 0; i < (int) mesh->vertices_.size(); ++i) {
			const double d = densities.at(i);
			if (d < params_.poissonMinDensity_) {
				idsToRemove.push_back(i);
			}
		}
		mesh->RemoveVerticesByIndex(idsToRemove);
		break;
	}
	default:
		throw std::runtime_error("Unknown reconstruction strategy");
	}

	mesh->ComputeTriangleNormals();
	{
		std::lock_guard<std::mutex> lck(meshingAccessMutex_);
		mesh_ = mesh;
	}
	std::cout << "normals: " << mesh->HasTriangleNormals() << std::endl;
	std::cout << "mesh size: " << mesh->vertices_.size() << std::endl;
	const std::string filename = ros::package::getPath("m545_volumetric_mapping") + "/data/map_mesh.stl";
	open3d::io::WriteTriangleMeshToSTL(filename, *mesh, false, false, false, false, false, false);
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

} // namespace m545_mapping
