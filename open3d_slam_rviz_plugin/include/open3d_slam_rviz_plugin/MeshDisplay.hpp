#pragma once
#include <memory>
#include <Eigen/Dense>
#include <rviz/message_filter_display.h>
#include <open3d_slam_msgs/PolygonMesh.h>


namespace open3d_slam_rviz_plugin {

class MeshVisual;

class MeshDisplay
    : public rviz::MessageFilterDisplay<open3d_slam_msgs::PolygonMesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshDisplay();
  virtual ~MeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const open3d_slam_msgs::PolygonMesh::ConstPtr& msg);

  std::unique_ptr<MeshVisual> visual_;
};

}  // namespace open3d_slam_rviz_plugin

