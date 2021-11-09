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
	Transform relativeTransformation_ = Transform::Identity();
	Time timeBegin_, timeFinish_;
	size_t submapIdxBegin_=0, submapIdxFinish_=0;
};


} //namespace m545_mapping
