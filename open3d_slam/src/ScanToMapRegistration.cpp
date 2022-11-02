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

#include "open3d_slam/cloud_processing.hpp"

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria;
} // namespace

ScanToMapIcpOpen3D::ScanToMapIcpOpen3D() {
	update(params_);
}

void ScanToMapIcpOpen3D::setParameters(const MapperParameters &p) {
	params_ = p;
	update(params_);
}
void ScanToMapIcpOpen3D::update(const MapperParameters &p) {
	icpCriteria.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective = icpObjectiveFactory(p.scanMatcher_.icpObjective_);
	mapBuilderCropper_ = croppingVolumeFactory(params_.mapBuilder_.cropper_);
	scanMatcherCropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
}

void ScanToMapIcpOpen3D::estimateNormalsIfNeeded(PointCloud *pcl) const {
	if (!pcl->HasNormals() && params_.scanMatcher_.icpObjective_ == o3d_slam::IcpObjective::PointToPlane) {
		estimateNormals(params_.scanMatcher_.knn_, pcl);
		pcl->NormalizeNormals();
	}
}

ProcessedScans ScanToMapIcpOpen3D::processForScanMatchingAndMerging(const PointCloud &in,
		const Transform &mapToRangeSensor) const {
	ProcessedScans retVal;
	PointCloudPtr narrowCropped, wideCropped;
	Timer timer;
	wideCropped = cropVoxelizeDownsample(in, *mapBuilderCropper_, params_.scanProcessing_.voxelSize_,
			params_.scanProcessing_.downSamplingRatio_);
	estimateNormalsIfNeeded(wideCropped.get());
	scanMatcherCropper_->setPose(Transform::Identity());
	narrowCropped = scanMatcherCropper_->crop(*wideCropped);
	retVal.match_ = narrowCropped;
	retVal.merge_ = wideCropped;
	assert_gt<int>(narrowCropped->points_.size(), 0, "ScanToMapIcpOpen3D::narrow cropped size is zero");
	assert_gt<int>(wideCropped->points_.size(), 0, "ScanToMapIcpOpen3D::wideCropped cropped size is zero");
	return retVal;
}
RegistrationResult ScanToMapIcpOpen3D::scanToMapRegistration(const PointCloud &scan, const Submap &activeSubmap,
		const Transform &mapToRangeSensor, const Transform &initialGuess) const {
	const PointCloud &activeSubmapPointCloud = activeSubmap.getMapPointCloud();
	scanMatcherCropper_->setPose(mapToRangeSensor);
	const PointCloudPtr mapPatch = scanMatcherCropper_->crop(activeSubmapPointCloud);
	assert_gt<int>(mapPatch->points_.size(), 0, "map patch size is zero");
	const RegistrationResult retVal = open3d::pipelines::registration::RegistrationICP(scan, *mapPatch,
			params_.scanMatcher_.maxCorrespondenceDistance_, initialGuess.matrix(), *icpObjective, icpCriteria);
	return std::move(retVal);
}

std::unique_ptr<ScanToMapIcpOpen3D> createScanToMapIcpOpen3D(const MapperParameters &p) {
	auto ret  = std::make_unique<ScanToMapIcpOpen3D>();
	ret->setParameters(p);
	return std::move(ret);
}
std::unique_ptr<ScanToMapRegistration> scanToMapRegistrationFactory(const MapperParameters &p) {


	switch (p.scanToMapRegType_) {

	case 	ScanToMapRegistrationType::PointToPlaneIcp:
	case 	ScanToMapRegistrationType::PointToPointIcp:{
		return createScanToMapIcpOpen3D(p);
	}

	default:
		throw std::runtime_error("scanToMapRegistrationFactory: unknown type of registration scan to map");
	}

}

} // namespace o3d_slam

