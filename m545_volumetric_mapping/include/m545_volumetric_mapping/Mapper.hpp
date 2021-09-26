/*
 * Mapper.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once


#include <open3d/geometry/PointCloud.h>

namespace m545_mapping{


class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper() = default;
	~Mapper()=default;

	void addRangeMeasurement(const PointCloud &cloud);
	const PointCloud &getMap() const;

private:

  PointCloud map_;


};

} /* namespace m545_mapping */
