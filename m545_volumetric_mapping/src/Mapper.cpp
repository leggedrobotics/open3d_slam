/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mapper.hpp"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>

namespace m545_mapping {

void Mapper::addRangeMeasurement(const Mapper::PointCloud &cloud) {
}
const Mapper::PointCloud& Mapper::getMap() const {
	return map_;
}

} /* namespace m545_mapping */
