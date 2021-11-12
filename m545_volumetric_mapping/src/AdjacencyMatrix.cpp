/*
 * AdjacencyMatrix.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/AdjacencyMatrix.hpp"
#include <stack>
#include <map>
#include <iostream>

namespace m545_mapping {

void AdjacencyMatrix::addEdge(SubmapId id1, SubmapId id2) {
	adjacency_[id1].insert(id2);
	adjacency_[id2].insert(id1);
}

bool AdjacencyMatrix::isAdjacent(SubmapId id1, SubmapId id2) const {

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

void AdjacencyMatrix::clear(){
	adjacency_.clear();
}

//todo the function is not really correct
std::vector<AdjacencyMatrix::SubmapId> AdjacencyMatrix::findLoopInvolvingEdge(SubmapId source, SubmapId dest) const {

	std::stack<SubmapId> stack;
	std::set<SubmapId> visited;
	std::vector<SubmapId> cycle;
	cycle.reserve(100);
	std::map<SubmapId, SubmapId> parents;

	auto isVisited = [&visited](SubmapId id) {
		return visited.find(id) != visited.end();
	};
	bool isFound = false;
	stack.push(source);
	SubmapId vPrev = source;
	while (!stack.empty() && !isFound) {
		const SubmapId v = stack.top();
		stack.pop();
		if (!isVisited(v)) {
			visited.insert(v);
			cycle.push_back(v);
//			std::cout << "v: " << v << " parent: " << parents.at(v) << std::endl;
			std::cout << "v: " << v << std::endl;
			for (const auto adj : adjacency_.at(v)) {
				std::cout << "   ,adj: " << adj << "\n";
				stack.push(adj);
				const bool isFollowingEdgeBackwards = adj == vPrev;
				if(!isFollowingEdgeBackwards && adj == source && isAdjacent(v,vPrev)){
					isFound = true;
					break;
				}

			}
			vPrev = v;
		}
	}

	if (isFound){
		cycle.push_back(source);
	} else {
		cycle.clear();
	}

	return cycle;

}

} //namespace m545_mapping
