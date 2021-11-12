/*
 * PlaceRecognition.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#pragma once

#include "m545_volumetric_mapping/Constraint.hpp"
#include "m545_volumetric_mapping/Parameters.hpp"

namespace m545_mapping {

class SubmapCollection;
class AdjacencyMatrix;

class PlaceRecognition {

public:
	PlaceRecognition();
	void setParameters(const MapperParameters &p);
	Constraints buildLoopClosureConstraints(const Transform &mapToRangeSensor,
			const SubmapCollection &submapCollection,
			const AdjacencyMatrix &adjMatrix,
			size_t lastFinishedSubmapIdx,
			size_t activeSubmapIdx) const;
	std::vector<size_t> getLoopClosureCandidatesIdxs(const Transform &mapToRangeSensor,
			const SubmapCollection &submapCollection,
			const AdjacencyMatrix &adjMatrix,
			size_t lastFinishedSubmapIdx,
			size_t activeSubmapIdx) const;

private:
	MapperParameters params_;

};

} //namespace m545_mapping
