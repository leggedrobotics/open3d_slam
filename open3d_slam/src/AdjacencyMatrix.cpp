/*
 * AdjacencyMatrix.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "open3d_slam/AdjacencyMatrix.hpp"
#include <stack>
#include <queue>
#include <map>
#include <iostream>

namespace o3d_slam {

void AdjacencyMatrix::addEdge(SubmapId id1, SubmapId id2) {
	adjacency_[id1].insert(id2);
	adjacency_[id2].insert(id1);
	isLoopClosureSubmap_[id1] = false;
	isLoopClosureSubmap_[id2] = false;
}

int AdjacencyMatrix::getDistanceToNearestLoopClosureSubmap(SubmapId id) const {
	if (isLoopClosureSubmap_.empty()){
		return std::numeric_limits<int>::max();
	}

	std::queue<SubmapId> toProcess;
	std::set<SubmapId> visited;
	std::map<SubmapId,SubmapId> parents;
	auto isVisited = [&visited](SubmapId id) {
		return visited.find(id) != visited.end();
	};
	SubmapId v = id;
	visited.insert(id);
	toProcess.push(id);
	while (!toProcess.empty()) {
		v = toProcess.front();
		toProcess.pop();
		if (isLoopClosureSubmap_.at(v)) {
			break;
		}
		for (const auto adj : adjacency_.at(v)) {
			if (!isVisited(adj)) {
				visited.insert(adj);
				toProcess.push(adj);
				parents.insert({adj,v});
			}
		}
	} // end while
	int distance = 0;
	while (v!=id){
		v = parents.at(v);
		distance++;
	}
	return std::max(0,distance-1);
}

void AdjacencyMatrix::markAsLoopClosureSubmap(SubmapId id) {
	isLoopClosureSubmap_.at(id) = true;
}

bool AdjacencyMatrix::isAdjacent(SubmapId id1, SubmapId id2) const {

	if (id1 == id2) {
		return true;
	}

	const auto search = adjacency_.find(id1);
	if (search == adjacency_.end()) {
		return false;
	}
	return search->second.find(id2) != search->second.end();
}

void AdjacencyMatrix::print() const {
	for (const auto node : adjacency_) {
		std::cout << "node " << node.first << " has neighbours: ";
		for (const auto neighbour : node.second) {
			std::cout << neighbour << ", ";
		}
		std::cout << "\n";
	}
}

void AdjacencyMatrix::clear() {
	adjacency_.clear();
}

} //namespace o3d_slam
