/*
 * adjacency_graph_example_node.cpp
 *
 *  Created on: Nov 7, 2022
 *      Author: jelavice
 */

#include <ros/ros.h>
#include "open3d_slam/AdjacencyMatrix.hpp"

using namespace o3d_slam;

AdjacencyMatrix createToyExample() {
	AdjacencyMatrix graph;
	graph.addEdge(0, 1);
	graph.addEdge(0, 4);
	graph.addEdge(4, 5);
	graph.addEdge(3, 4);
	graph.addEdge(1, 3);
	graph.addEdge(2, 1);
	graph.addEdge(2, 6);
	graph.addEdge(6, 7);
	graph.addEdge(3, 7);
	graph.addEdge(4, 9);
	graph.addEdge(3, 8);
	graph.addEdge(8, 11);
	graph.addEdge(12, 11);
	graph.addEdge(12, 10);
	graph.addEdge(7, 10);
	return graph;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "adjacency_graph_example");

	AdjacencyMatrix graph = createToyExample();
	graph.print();

	graph.markAsLoopClosureSubmap(10);
	graph.markAsLoopClosureSubmap(2);
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(0) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(1) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(7) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(5) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(3) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(8) << std::endl;
	std::cout << graph.getDistanceToNearestLoopClosureSubmap(11) << std::endl;

	return 0;
}




