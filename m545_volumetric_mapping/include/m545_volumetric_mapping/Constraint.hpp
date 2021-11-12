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
	Transform transformSubmapToSubmap_ = Transform::Identity();
	size_t fromSubmapIdx_=0, toSubmapIdx_=0;
};



} //namespace m545_mapping
