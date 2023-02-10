#include <Eigen/Eigen>

namespace o3d_slam {

/** \brief Class for holding a pose.
 *
 * This class provides buffered access to position, orientation, and time values to the represented pose value.
 */
class Pose {
 public:
  Pose() : _position(Eigen::Vector3d(0.0, 0.0, 0.0)), _orientation(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)), _time(0.0) {}

  Pose(Eigen::Vector3d position, double time) : _position(position), _orientation(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)), _time(time) {}

  Pose(Eigen::Vector3d position, Eigen::Vector4d orientation, double time) : _position(position), _orientation(orientation), _time(time) {}

  Pose(const Pose& other) : _position(other._position), _orientation(other._orientation), _time(other._time) {}

  Eigen::Vector3d position() const { return _position; }

  Eigen::Vector4d orientation() const { return _orientation; }

  double time() const { return _time; }

  private:
  Eigen::Vector3d _position;
  Eigen::Vector4d _orientation;  // xyzw
  double _time;
};

}  // namespace graph_msf