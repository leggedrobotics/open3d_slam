/*
 * AdjacencyMatrix.hpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */
#pragma once

#include <vector>
#include <unordered_map>
#include <set>
#include <Eigen/Core>
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {
class AdjacencyMatrix {

public:
	AdjacencyMatrix() = default;
	using SubmapId = int64;

	void addEdge(SubmapId id1, SubmapId id2);
	bool isAdjacent(SubmapId id1, SubmapId id2) const;
	void print() const;
	std::vector<SubmapId> findLoopInvolvingEdge(SubmapId id1, SubmapId id2) const;
	void clear();
private:



	std::unordered_map<SubmapId, std::set<SubmapId>> adjacency_;


};
} //namespace o3d_slam
