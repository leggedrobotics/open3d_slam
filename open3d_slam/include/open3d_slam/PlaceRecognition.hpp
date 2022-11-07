/*
 * PlaceRecognition.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#pragma once

#include "open3d_slam/Constraint.hpp"
#include "open3d_slam/Parameters.hpp"
#include <open3d/geometry/PointCloud.h>

namespace o3d_slam {

class SubmapCollection;
class AdjacencyMatrix;
class Submap;

class PlaceRecognition {

public:
	using PointCloud = open3d::geometry::PointCloud;
	PlaceRecognition();
	void setParameters(const MapperParameters &p);
	Constraints buildLoopClosureConstraints(const Transform &mapToRangeSensor, const SubmapCollection &submapCollection,
			const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx, size_t activeSubmapIdx, const Time &timestamp) const;
	std::vector<size_t> getLoopClosureCandidatesIdxs(const Transform &mapToRangeSensor,
			const SubmapCollection &submapCollection, const AdjacencyMatrix &adjMatrix, size_t lastFinishedSubmapIdx,
			size_t activeSubmapIdx) const;
	void setFolderPath(const std::string &folderPath);
private:
	bool isRegistrationConsistent(const Eigen::Matrix4d &T) const;
	void updateRegistrationAlgorithm(const MapperParameters &p);

	std::string folderPath_ = "";
	mutable size_t recognitionCounter_ = 0;
	MapperParameters params_;

};

} //namespace o3d_slam
