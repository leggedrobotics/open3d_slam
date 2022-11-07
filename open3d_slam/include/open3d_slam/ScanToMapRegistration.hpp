/*
 * ScanToMapRegistration.hpp
 *
 *  Created on: Oct 31, 2022
 *      Author: jelavice
 */

#pragma once
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/Transform.hpp"
#include "open3d_slam/Parameters.hpp"

#include "open3d/pipelines/registration/Registration.h"

#include <memory>

namespace o3d_slam {

class Submap;
class CroppingVolume;

using RegistrationResult = open3d::pipelines::registration::RegistrationResult;

struct ProcessedScans {
	PointCloudPtr merge_;
	PointCloudPtr match_;
};

class ScanToMapRegistration {

public:
	ScanToMapRegistration() = default;
	virtual ~ScanToMapRegistration() = default;
	virtual ProcessedScans processForScanMatchingAndMerging(const PointCloud &in,
			const Transform &mapToRangeSensor) const =0;
	virtual RegistrationResult scanToMapRegistration(const PointCloud &scan, const Submap &activeSubmap,
			const Transform &mapToRangeSensor, const Transform &initialGuess) const = 0;
	virtual bool isMergeScanValid(const PointCloud &in) const =0;
	virtual void prepareInitialMap(PointCloud *map) const =0;
};

class ScanToMapIcp : public ScanToMapRegistration {

public:
	ScanToMapIcp();
	virtual ~ScanToMapIcp() = default;
	void setParameters(const MapperParameters &p);
	ProcessedScans processForScanMatchingAndMerging(const PointCloud &in, const Transform &mapToRangeSensor) const final;
	RegistrationResult scanToMapRegistration(const PointCloud &scan, const Submap &activeSubmap, const Transform &mapToRangeSensor,const Transform &initialGuess) const final;
	bool isMergeScanValid(const PointCloud &in) const final;
	void prepareInitialMap(PointCloud *map) const final;
private:
	PointCloudPtr preprocess(const PointCloud &in) const;
	void update(const MapperParameters &p);

	MapperParameters params_;
	std::shared_ptr<CroppingVolume> scanMatcherCropper_;
	std::shared_ptr<CroppingVolume> mapBuilderCropper_;
};

std::unique_ptr<ScanToMapIcp> createScanToMapIcp(const MapperParameters &p);
std::unique_ptr<ScanToMapRegistration> scanToMapRegistrationFactory(const MapperParameters &p);
CloudRegistrationParameters toCloudRegistrationType(const ScanToMapRegistrationParameters &p);

} // namespace o3d_slam
