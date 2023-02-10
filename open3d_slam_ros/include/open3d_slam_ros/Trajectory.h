#include "open3d_slam_ros/Pose.h"

namespace o3d_slam {

/** \brief Class for holding a trajectory.
 *
 * This class provides buffered access to poses to the represented trajectory.
 */
class Trajectory {
 public:
  Trajectory() : _poses(std::vector<Pose>()) {}

  Trajectory(const Trajectory& other) : _poses(other._poses) {}

  void addPose(Eigen::Vector3d position, double time) {
    Pose newPose(position, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), time);
    _poses.push_back(newPose);
  }

  void addPose(Pose pose) { _poses.push_back(pose); }

  double distance() {
    double distance = 0.0;
    if (_poses.size() < 2) return distance;
    Eigen::Vector3d lastPose = _poses.front().position();
    for (auto poseIt = std::next(_poses.begin()); poseIt != _poses.end(); ++poseIt) {
      distance += ((poseIt->position() - lastPose).norm());
      lastPose = poseIt->position();
    }

    return distance;
  }

  bool standing(const int& rate, const int& seconds, const double& noMovementThreshold) {
    if (_poses.size() < rate * seconds) return false;

    double distance = 0.0;
    Eigen::Vector3d lastPose = (_poses.end() - rate * seconds)->position();
    for (auto poseIt = _poses.end() - rate * seconds + 1; poseIt != _poses.end(); ++poseIt) {
      distance += ((poseIt->position() - lastPose).norm());
      lastPose = poseIt->position();
    }

    return distance < noMovementThreshold;
  }

  std::vector<Pose> poses() const { return _poses; }

 private:
  std::vector<Pose> _poses;
};

}  // namespace graph_msf
