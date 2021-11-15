/*
 * Constraint.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */

#pragma once

#include "m545_volumetric_mapping/Transform.hpp"

namespace m545_mapping {

struct Constraint{
	Transform sourceToTarget_ = Transform::Identity();
	size_t sourceSubmapIdx_=0, targetSubmapIdx_=0;
};

using Constraints = std::vector<Constraint>;


} //namespace m545_mapping
