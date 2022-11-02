/*
 * CloudRegistration.cpp
 *
 *  Created on: Nov 1, 2022
 *      Author: jelavice
 */
#include "open3d_slam/CloudRegistration.hpp"
#include "open3d_slam/helpers.hpp"

namespace o3d_slam {
using namespace open3d::pipelines::registration;

RegistrationIcpPointToPlaneOpen3D::RegistrationResult RegistrationIcpPointToPlaneOpen3D::registerClouds(const PointCloud &source,
		const PointCloud &target, const Transform &init) const {

	return RegistrationICP(
		source, target, maxCorrespondenceDistance_,
		Eigen::Matrix4d::Identity(),pointToPlane_ , icpConvergenceCriteria_);

}
void RegistrationIcpPointToPlaneOpen3D::prepareCloud(PointCloud *cloud) const {
	open3d::geometry::KDTreeSearchParamHybrid param(maxRadiusNormalEstimation_, knnNormalEstimation_);
	cloud->EstimateNormals(param);
	cloud->NormalizeNormals();
}

std::unique_ptr<RegistrationIcpPointToPlaneOpen3D> createPointToPlaneIcpOpen3D(const CloudRegistrationParameters &p) {
	auto ret  = std::make_unique<RegistrationIcpPointToPlaneOpen3D>();
	ret->maxCorrespondenceDistance_ = p.icp_.maxCorrespondenceDistance_;
	ret->knnNormalEstimation_ = p.icp_.knn_;
	ret->maxRadiusNormalEstimation_ = p.icp_.maxDistanceKnn_;
	ret->icpConvergenceCriteria_.max_iteration_ = p.icp_.maxNumIter_;
	return std::move(ret);
}
std::unique_ptr<CloudRegistration> cloudRegistrationFactory(const Parameters &p) {
	const auto &scanRegParameters = *(p.as<CloudRegistrationParameters>());
	switch (scanRegParameters.regType_) {

	case 	CloudRegistrationType::PointToPlaneIcp:{
		return createPointToPlaneIcpOpen3D(scanRegParameters);
	}

	default:
		throw std::runtime_error("scanToMapRegistrationFactory: unknown type of registration scan to map");
	}

}

} // namespace o3d_slam

