/*
 * PointcloudRegistration.hpp
 *
 *  Created on: Nov 1, 2022
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include "open3d/pipelines/registration/Registration.h"
#include "open3d/pipelines/registration/GeneralizedICP.h"

#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/Transform.hpp"
#include "open3d_slam/Parameters.hpp"

namespace o3d_slam {

class CloudRegistration {
public:
	using RegistrationResult = open3d::pipelines::registration::RegistrationResult;
	CloudRegistration() = default;
	virtual ~CloudRegistration() = default;

	virtual RegistrationResult registerClouds(const PointCloud &source, const PointCloud &target,
			const Transform &init) const = 0;
	virtual void estimateNormalsOrCovariancesIfNeeded(PointCloud *cloud) const {}


};

class RegistrationIcpPointToPlane: public CloudRegistration {
public:
	using RegistrationResult = open3d::pipelines::registration::RegistrationResult;
	RegistrationIcpPointToPlane() = default;
	~RegistrationIcpPointToPlane() override = default;
	RegistrationResult registerClouds(const PointCloud &source, const PointCloud &target,
			const Transform &init) const final;
	void estimateNormalsOrCovariancesIfNeeded(PointCloud *cloud) const final;

	double maxCorrespondenceDistance_ = 1.0;
	int knnNormalEstimation_ = 10;
	double maxRadiusNormalEstimation_ = 2.0;
	open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria_;
	open3d::pipelines::registration::TransformationEstimationPointToPlane pointToPlane_;
};

class RegistrationIcpPointToPoint: public CloudRegistration {
public:
	using RegistrationResult = open3d::pipelines::registration::RegistrationResult;
	RegistrationIcpPointToPoint() = default;
	~RegistrationIcpPointToPoint() override = default;
	RegistrationResult registerClouds(const PointCloud &source, const PointCloud &target,
			const Transform &init) const final;

	double maxCorrespondenceDistance_ = 1.0;
	open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria_;
};

class RegistrationIcpGeneralized: public CloudRegistration {
public:
	using RegistrationResult = open3d::pipelines::registration::RegistrationResult;
	RegistrationIcpGeneralized() = default;
	~RegistrationIcpGeneralized() override = default;
	RegistrationResult registerClouds(const PointCloud &source, const PointCloud &target,
			const Transform &init) const final;
	void estimateNormalsOrCovariancesIfNeeded(PointCloud *cloud) const final;

	double maxCorrespondenceDistance_ = 1.0;
	int knnNormalEstimation_ = 10;
	double maxRadiusNormalEstimation_ = 2.0;
	open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria_;
	open3d::pipelines::registration::TransformationEstimationForGeneralizedICP tranformationEstimationGICP_;
};

std::unique_ptr<RegistrationIcpGeneralized> createGeneralizedIcp(const CloudRegistrationParameters &p);
std::unique_ptr<RegistrationIcpPointToPoint> createPointToPointIcp(const CloudRegistrationParameters &p);
std::unique_ptr<RegistrationIcpPointToPlane> createPointToPlaneIcp(const CloudRegistrationParameters &p);
std::unique_ptr<CloudRegistration> cloudRegistrationFactory(const CloudRegistrationParameters &p);

} // namespace o3d_slam

