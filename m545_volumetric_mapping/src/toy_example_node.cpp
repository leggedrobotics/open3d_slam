/*
 * toy_example_node.cpp
 *
 *  Created on: Nov 26, 2021
 *      Author: jelavice
 */

#include <open3d/Open3D.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"
#include "open3d_conversions/open3d_conversions.h"

#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include "m545_volumetric_mapping/typedefs.hpp"

#include "m545_volumetric_mapping/Constraint.hpp"

#include <ros/ros.h>

using namespace m545_mapping;

ros::NodeHandlePtr nh;

void buildLoopClosureConstraints(const SubmapCollection &_submaps, Constraints *cs) {
	cs->clear();
	const int nSubmaps = _submaps.submaps_.size();
	const auto &submaps = _submaps.submaps_;
	Constraint c;
	c.sourceSubmapIdx_ = 3;
	c.targetSubmapIdx_ = 0;
	const Transform dT(makeTransform(Eigen::Vector3d(-2.0,0.0,0), Eigen::Quaterniond::Identity()));
	const Transform dToracle(makeTransform(Eigen::Vector3d(0.0,-2.0,0), Eigen::Quaterniond::Identity()));
	const Transform &mapToSource = submaps.at(c.sourceSubmapIdx_).getMapToSubmapOrigin()*dT;
	const Transform &mapToTarget = submaps.at(c.targetSubmapIdx_).getMapToSubmapOrigin();



	c.sourceToTarget_ = mapToSource.inverse() * mapToTarget;

	std::cout << "Loop closure con: " << asString(c.sourceToTarget_) << "\n";

	//	c.sourceToTarget_ = makeTransform(Eigen::Vector3d(-2.0,-2.0,0), Eigen::Quaterniond::Identity());
//	{
//		const Transform mapToSourcePre = submaps.at(c.sourceSubmapIdx_).getMapToSubmapOrigin().inverse();
//		const Transform mapToTargetPre = submaps.at(c.targetSubmapIdx_).getMapToSubmapOrigin().inverse();
//		c.sourceToTarget_ = mapToTargetPre*mapToSourcePre.inverse();
//	}


//	c.sourceToTarget_ = c.sourceToTarget_.inverse();
//	c.sourceToTarget_ = Transform::Identity();
	c.isOdometryConstraint_ = false;
	c.isInformationMatrixValid_ = true;
	cs->push_back(c);

}

void buildOdometryConstraints(const SubmapCollection &_submaps, Constraints *cs) {
	cs->clear();
	const int nSubmaps = _submaps.submaps_.size();
	const auto &submaps = _submaps.submaps_;
	for (int i = 1; i < nSubmaps; ++i) {
		Constraint c;
		c.sourceSubmapIdx_ = i - 1;
		c.targetSubmapIdx_ = i;
		const Transform &mapToSource = submaps.at(c.sourceSubmapIdx_).getMapToSubmapOrigin();
		const Transform &mapToTarget = submaps.at(c.targetSubmapIdx_).getMapToSubmapOrigin();
		c.sourceToTarget_ = mapToSource.inverse() * mapToTarget;
//		{
//			const Transform mapToSourcePre = submaps.at(c.sourceSubmapIdx_).getMapToSubmapOrigin().inverse();
//			const Transform mapToTargetPre = submaps.at(c.targetSubmapIdx_).getMapToSubmapOrigin().inverse();
//			c.sourceToTarget_ = mapToTargetPre*mapToSourcePre.inverse();
//		}
//		c.sourceToTarget_ = c.sourceToTarget_.inverse();
		c.isOdometryConstraint_ = true;
		c.isInformationMatrixValid_ = true;
		cs->push_back(c);
	}
}

void buildSubmaps(SubmapCollection *submaps) {
	const auto qIdentity = Eigen::Quaterniond::Identity();
	submaps->submaps_.clear();
	for (int i = 0; i < 4; ++i) {
		Submap s(i, std::max<int>(0, i - 1));
		submaps->submaps_.push_back(s);
	}
	submaps->submaps_.at(0).setMapToSubmapOrigin(makeTransform(Eigen::Vector3d(1, 1, 0), qIdentity));
	submaps->submaps_.at(1).setMapToSubmapOrigin(makeTransform(Eigen::Vector3d(1, 10, 0), qIdentity));
	submaps->submaps_.at(2).setMapToSubmapOrigin(makeTransform(Eigen::Vector3d(2, 10, 0), qIdentity));
	submaps->submaps_.at(3).setMapToSubmapOrigin(makeTransform(Eigen::Vector3d(3, 3, 0), qIdentity));

}

void printOptimizedSubmapPoses(const SubmapCollection &submaps, const OptimizedTransforms &transforms){
	std::cout << "print submap poses: \n";
		for (int i = 0; i < submaps.getSubmaps().size(); ++i) {
			const auto submapPose = submaps.submaps_.at(i).getMapToSubmapOrigin();
			std::cout << "Node " << i << ": " << asString(submapPose*transforms.at(i).dT_) << "\n";

		}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));

	SubmapCollection submaps;
	buildSubmaps(&submaps);
	Constraints odometryConstraints, loopClosureConstraints;
	OptimizationProblem optimization;
	buildOdometryConstraints(submaps, &odometryConstraints);
	buildLoopClosureConstraints(submaps, &loopClosureConstraints);

		auto &logger = open3d::utility::Logger::GetInstance();
			logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
	optimization.insertLoopClosureConstraints(loopClosureConstraints);
	optimization.insertOdometryConstraints(odometryConstraints);
	optimization.buildOptimizationProblem(submaps);
	optimization.print();
	optimization.solve();
	optimization.print();

	const auto optimizedTransformIncrements = optimization.getOptimizedTransformIncrements();
	printOptimizedSubmapPoses(submaps, optimizedTransformIncrements);

	std::cout << "Toy example over!!!" << std::endl;
	ros::spin();
	return 0;
}
