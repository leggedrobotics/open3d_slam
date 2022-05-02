/*
 * MotionCompensation.hpp
 *
 *  Created on: Apr 24, 2022
 *      Author: jelavice
 */

#pragma once

#include <memory>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"

namespace o3d_slam {

class MotionCompensation {

public:
	MotionCompensation() = default;
	virtual ~MotionCompensation() = default;

	virtual std::shared_ptr<PointCloud> undistortInputPointCloud(const PointCloud &input,
			const Time &timestamp);

};

class ConstantVelocityMotionCompensation : public MotionCompensation {

public:
	ConstantVelocityMotionCompensation(const TransformInterpolationBuffer &buffer);
	~ConstantVelocityMotionCompensation() override = default;

	void setParameters(const ConstantVelocityMotionCompensationParameters &p);
	std::shared_ptr<PointCloud> undistortInputPointCloud(const PointCloud &input, const Time &timestamp) final;


private:
	double computePhase(double x, double y);
	void estimateLinearAndAngularVelocity(const Time &timestamp, Eigen::Vector3d *linearVelocity, Eigen::Vector3d *angularVelocity) const;

	const TransformInterpolationBuffer &buffer_;
	ConstantVelocityMotionCompensationParameters params_;

};

} // namespace o3d_slam
