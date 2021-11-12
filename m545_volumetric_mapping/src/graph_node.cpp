/*
 * graph_node.cpp
 *
 *  Created on: Nov 11, 2021
 *      Author: jelavice
 */
#include <iostream>
#include "m545_volumetric_mapping/AdjacencyMatrix.hpp"
using namespace m545_mapping;
using Id = AdjacencyMatrix::SubmapId;

void buildGraph(const std::vector<std::pair<Id, Id>> &edges, AdjacencyMatrix *g) {
	g->clear();
	for (const auto edge : edges) {
		g->addEdge(edge.first, edge.second);
	}
}

void checkRes(std::vector<Id> &loop, const std::string &msg) {
	std::cout << msg;
	if (loop.empty()) {
		std::cout << " no loop found \n";
	} else {
		std::cout << " loop found \n";
		for (auto n : loop) {
			std::cout << n << "->";
		}
		std::cout << "\n";
	}
}

int main(int argc, char **argv) {
	AdjacencyMatrix g;
	std::vector<std::pair<Id, Id>> singleLoop { { 0, 1 }, { 0, 2 }, { 2, 3 }, { 3, 4 }, { 4, 0 } };

	std::vector<std::pair<Id, Id>> noLoop { { 0, 1 }, { 0, 2 }, { 2, 3 }, { 4, 0 } };

	std::vector<std::pair<Id, Id>> singleLoopHarder { { 0, 1 }, { 0, 2 }, { 2, 3 }, { 3, 4 }, { 4, 0 }, { 3, 5 },
			{ 6, 5 }, { 7, 5 } };

	std::vector<std::pair<Id, Id>> twoLoops { { 0, 1 }, { 0, 2 }, { 2, 3 }, { 3, 4 }, { 4, 0 }, { 1, 5 }, { 6, 5 }, { 7,
			5 }, { 7, 0 } };

	buildGraph(singleLoop, &g);
	g.print();
	auto loop = g.findLoopInvolvingEdge(0, 4);
	checkRes(loop, "\n single loop \n");
	std::cout << "\n";

	buildGraph(singleLoopHarder, &g);
	g.print();
	loop = g.findLoopInvolvingEdge(0, 4);
	checkRes(loop, "\n singleLoopHarder \n");
	std::cout << "\n";

	buildGraph(noLoop, &g);
	g.print();
	loop = g.findLoopInvolvingEdge(0, 4);
	checkRes(loop, "\n no loop \n");
	std::cout << "\n";

//	buildGraph(twoLoops, &g);
//	g.print();
//	loop = g.findLoopInvolvingEdge(0, 4);
//	checkRes(loop, "\n two loops \n");
//	std::cout << "\n";

	return 0;
}

