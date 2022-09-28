/*
 * math.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "open3d_slam/math.hpp"
#include <numeric>

namespace o3d_slam {

Transform fromXYZandRPY(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy){
	Eigen::Quaterniond q = fromRPY( rpy.x(), rpy.y(), rpy.z());
	return fromXYZandQuaternion(xyz,q);
}
Transform fromXYZandRPY(const Eigen::Vector3d &xyz, double roll, double pitch, double yaw){
	Eigen::Quaterniond q = fromRPY( roll,pitch,yaw);
	return fromXYZandQuaternion(xyz,q);
}
Transform fromXYZandQuaternion(const Eigen::Vector3d &xyz, const Eigen::Quaterniond &q){
	return makeTransform(xyz, q);
}

Transform fromXYZandRPY(double x, double y, double z, double roll, double pitch, double yaw){
	return fromXYZandRPY(Eigen::Vector3d(x,y,z),roll,pitch,yaw);
}
Transform fromXYZandQuaternion(double x, double y, double z, const Eigen::Quaterniond &q){
	return makeTransform(Eigen::Vector3d(x,y,z), q);
}

Eigen::Quaterniond fromRPY(const double roll, const double pitch, const double yaw) {

	const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
	const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
	const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
	return yaw_angle * pitch_angle * roll_angle;
}

Eigen::Vector3d toRPY(const Eigen::Quaterniond &_q) {
	Eigen::Quaterniond q(_q);
	q.normalize();
	const double r = getRollFromQuat(q.w(), q.x(), q.y(), q.z());
	const double p = getPitchFromQuat(q.w(), q.x(), q.y(), q.z());
	const double y = getYawFromQuat(q.w(), q.x(), q.y(), q.z());
	return Eigen::Vector3d(r, p, y);
}

Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy) {
	return fromRPY(rpy.x(), rpy.y(), rpy.z());
}

double calcMean(const std::vector<double> &data) {
	if (data.empty()) {
		return 0.0;
	}
	return static_cast<double>(std::accumulate(data.begin(), data.end(), 0.0)) / data.size();
}

double calcStandardDeviation(const std::vector<double> &data) {
	const int n = data.size();
	if (n < 2) {
		return 0.0;
	}
	const double mean = calcMean(data);
	double stdDev = 0.0;
	for (const auto d : data) {
		const double e = d - mean;
		stdDev += e * e;
	}

	return std::sqrt(stdDev / (n - 1));
}

} /* namespace o3d_slam */
