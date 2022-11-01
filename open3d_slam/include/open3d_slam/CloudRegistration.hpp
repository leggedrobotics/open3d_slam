/*
 * PointcloudRegistration.hpp
 *
 *  Created on: Nov 1, 2022
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include "open3d/pipelines/registration/Registration.h"
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
	virtual void prepareCloud(PointCloud *cloud) const {
	}
	;

};

class RegistrationIcpPointToPlaneOpen3D: public CloudRegistration {
public:
	using RegistrationResult = open3d::pipelines::registration::RegistrationResult;
	RegistrationIcpPointToPlaneOpen3D() = default;
	~RegistrationIcpPointToPlaneOpen3D() override = default;
	virtual RegistrationResult registerClouds(const PointCloud &source, const PointCloud &target,
			const Transform &init) const final;
	virtual void prepareCloud(PointCloud *cloud) const final;

	double maxCorrespondenceDistance_ = 1.0;
	int knnNormalEstimation_ = 10;
	double maxRadiusNormalEstimation_ = 2.0;
	open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria_;
	open3d::pipelines::registration::TransformationEstimationPointToPlane pointToPlane_;
};

std::unique_ptr<RegistrationIcpPointToPlaneOpen3D> createPointToPlaneIcpOpen3D(const IcpParameters &p);
std::unique_ptr<CloudRegistration> cloudRegistrationFactory(const Parameters &p);

} // namespace o3d_slam

