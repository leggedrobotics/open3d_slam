namespace o3d_slam {

/** \brief Class for holding a trajectory.
 *
 * This class provides buffered access to poses to the represented trajectory.
 */
  class AlignPose {
  public:
    AlignPose() : _position(Eigen::Vector3d(0.0, 0.0, 0.0)), _orientation(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)), _time(0.0) {}

    AlignPose(Eigen::Vector3d position, double time) : _position(position), _orientation(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)), _time(time) {}

    AlignPose(Eigen::Vector3d position, Eigen::Vector4d orientation, double time) : _position(position), _orientation(orientation), _time(time) {}

    AlignPose(const AlignPose& other) : _position(other._position), _orientation(other._orientation), _time(other._time) {}

    Eigen::Vector3d position() const { return _position; }

    Eigen::Vector4d orientation() const { return _orientation; }

    double time() const { return _time; }

    private:
    Eigen::Vector3d _position;
    Eigen::Vector4d _orientation;  // xyzw
    double _time;
  };

  class Trajectory {
  public:
    Trajectory() : _poses(std::vector<AlignPose>()) {}

    Trajectory(const Trajectory& other) : _poses(other._poses) {}

    void addPose(Eigen::Vector3d position, double time) {
      AlignPose newPose(position, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), time);
      _poses.push_back(newPose);
    }

    void addPose(AlignPose pose) { _poses.push_back(pose); }

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

    std::vector<AlignPose> poses() const { return _poses; }

  private:
    std::vector<AlignPose> _poses;
  };

}  // namespace graph_msf
