/*
 * AdjacencyMatrix.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/AdjacencyMatrix.hpp"


namespace m545_mapping {


void AdjacencyMatrix::addNode(SubmapId id1, SubmapId id2)
{
	adjacency_[id1].insert(id2);
	adjacency_[id2].insert(id1);
}

bool AdjacencyMatrix::isAdjacent(SubmapId id1, SubmapId id2) const{

	const auto search = adjacency_.find(id1);
	if(search == adjacency_.end()){
		return false;
	}
	return search->second.find(id2) != search->second.end();
}

} //namespace m545_mapping
