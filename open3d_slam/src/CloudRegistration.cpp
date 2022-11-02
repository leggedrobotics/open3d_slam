/*
 * CloudRegistration.cpp
 *
 *  Created on: Nov 1, 2022
 *      Author: jelavice
 */
#include "open3d_slam/CloudRegistration.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"

namespace o3d_slam {
using namespace open3d::pipelines::registration;

RegistrationIcpPointToPlane::RegistrationResult RegistrationIcpPointToPlane::registerClouds(const PointCloud &source,
		const PointCloud &target, const Transform &init) const {

	return RegistrationICP(
		source, target, maxCorrespondenceDistance_,
		Eigen::Matrix4d::Identity(),pointToPlane_ , icpConvergenceCriteria_);

}
void RegistrationIcpPointToPlane::prepareCloud(PointCloud *cloud) const {
	assert_gt(maxRadiusNormalEstimation_,0.0,"maxRadiusNormalEstimation_");
	assert_gt(knnNormalEstimation_,0,"knnNormalEstimation_");
	open3d::geometry::KDTreeSearchParamHybrid param(maxRadiusNormalEstimation_, knnNormalEstimation_);
	cloud->EstimateNormals(param);
	cloud->NormalizeNormals();
}

std::unique_ptr<RegistrationIcpPointToPlane> createPointToPlaneIcp(const CloudRegistrationParameters &p) {
	auto ret  = std::make_unique<RegistrationIcpPointToPlane>();
	ret->maxCorrespondenceDistance_ = p.icp_.maxCorrespondenceDistance_;
	ret->knnNormalEstimation_ = p.icp_.knn_;
	ret->maxRadiusNormalEstimation_ = p.icp_.maxDistanceKnn_;
	ret->icpConvergenceCriteria_.max_iteration_ = p.icp_.maxNumIter_;
	return std::move(ret);
}
std::unique_ptr<CloudRegistration> cloudRegistrationFactory(const CloudRegistrationParameters &p) {
	switch (p.regType_) {

	case 	CloudRegistrationType::PointToPlaneIcp:{
		return createPointToPlaneIcp(p);
	}

	default:
		throw std::runtime_error("scanToMapRegistrationFactory: unknown type of registration scan to map");
	}

}

} // namespace o3d_slam

