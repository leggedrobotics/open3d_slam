/*
 * ScanToMapRegistration.cpp
 *
 *  Created on: Oct 31, 2022
 *      Author: jelavice
 */
#include "open3d_slam/ScanToMapRegistration.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/CloudRegistration.hpp"

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<CloudRegistration> cloudRegistration;

} // namespace

ScanToMapIcp::ScanToMapIcp() {
	update(params_);
}

void ScanToMapIcp::setParameters(const MapperParameters &p) {
	params_ = p;
	update(params_);
}
void ScanToMapIcp::update(const MapperParameters &p) {
	mapBuilderCropper_ = croppingVolumeFactory(params_.mapBuilder_.cropper_);
	scanMatcherCropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
	cloudRegistration = cloudRegistrationFactory(toCloudRegistrationType(p.scanMatcher_));
}

PointCloudPtr ScanToMapIcp::preprocess(const PointCloud &in) const{
	auto croppedCloud = mapBuilderCropper_->crop(in);
	o3d_slam::voxelize(params_.scanProcessing_.voxelSize_, croppedCloud.get());
	cloudRegistration->estimateNormalsOrCovariancesIfNeeded(croppedCloud.get());
	return croppedCloud->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);
}

ProcessedScans ScanToMapIcp::processForScanMatchingAndMerging(const PointCloud &in,
		const Transform &mapToRangeSensor) const {
	ProcessedScans retVal;
	PointCloudPtr narrowCropped, wideCropped;
	Timer timer;
	wideCropped = preprocess(in);
	scanMatcherCropper_->setPose(Transform::Identity());
	narrowCropped = scanMatcherCropper_->crop(*wideCropped);
	retVal.match_ = narrowCropped;
	retVal.merge_ = wideCropped;
	assert_gt<int>(narrowCropped->points_.size(), 0, "ScanToMapIcp::narrow cropped size is zero");
	assert_gt<int>(wideCropped->points_.size(), 0, "ScanToMapIcp::wideCropped cropped size is zero");
	return retVal;
}
RegistrationResult ScanToMapIcp::scanToMapRegistration(const PointCloud &scan, const Submap &activeSubmap,
		const Transform &mapToRangeSensor, const Transform &initialGuess) const {
	const PointCloud &activeSubmapPointCloud = activeSubmap.getMapPointCloud();
	scanMatcherCropper_->setPose(mapToRangeSensor);
	const PointCloudPtr mapPatch = scanMatcherCropper_->crop(activeSubmapPointCloud);
	assert_gt<int>(mapPatch->points_.size(), 0, "map patch size is zero");
	return cloudRegistration->registerClouds(scan, *mapPatch, initialGuess);
}

bool ScanToMapIcp::isMergeScanValid(const PointCloud &in) const {
	switch (params_.scanMatcher_.scanToMapRegType_) {
	case ScanToMapRegistrationType::PointToPlaneIcp: {
		return in.HasNormals();
	}
	case ScanToMapRegistrationType::PointToPointIcp: {
		return true;
	}
	case ScanToMapRegistrationType::GeneralizedIcp: {
		return in.HasCovariances() || in.HasNormals();
	}
	default:
		throw std::runtime_error("cannot check whether merge scan is valid for this registration type");
	}
	return true;
}

void ScanToMapIcp::prepareInitialMap(PointCloud *map) const {
//	estimateNormalsIfNeeded(map);
	cloudRegistration->estimateNormalsOrCovariancesIfNeeded(map);
}

std::unique_ptr<ScanToMapIcp> createScanToMapIcp(const MapperParameters &p) {
	auto ret = std::make_unique<ScanToMapIcp>();
	ret->setParameters(p);
	return std::move(ret);
}
std::unique_ptr<ScanToMapRegistration> scanToMapRegistrationFactory(const MapperParameters &p) {
	switch (p.scanMatcher_.scanToMapRegType_) {
	case ScanToMapRegistrationType::PointToPlaneIcp:
	case ScanToMapRegistrationType::GeneralizedIcp:
	case ScanToMapRegistrationType::PointToPointIcp: {
		return createScanToMapIcp(p);
	}

	default:
		throw std::runtime_error("scanToMapRegistrationFactory: unknown type of registration scan to map");
	}

}

CloudRegistrationParameters toCloudRegistrationType(const ScanToMapRegistrationParameters &p) {
	CloudRegistrationParameters retVal;
	retVal.icp_ = p.icp_;
	switch (p.scanToMapRegType_) {
	case ScanToMapRegistrationType::PointToPlaneIcp: {
		retVal.regType_ = CloudRegistrationType::PointToPlaneIcp;
		break;
	}
	case ScanToMapRegistrationType::PointToPointIcp: {
		retVal.regType_ = CloudRegistrationType::PointToPointIcp;
		break;
	}
	case ScanToMapRegistrationType::GeneralizedIcp: {
		retVal.regType_ = CloudRegistrationType::GeneralizedIcp;
		break;
	}
	default:
		throw std::runtime_error(
				"Conversion not possible from ScanToMapRegistrationParameters to CloudRegistrationParameters, for this particular scan to map reg type");
	}

	return retVal;
}

} // namespace o3d_slam

