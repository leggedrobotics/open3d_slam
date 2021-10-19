#pragma once
#include <memory>
#include <Eigen/Dense>
#include <rviz/message_filter_display.h>
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>


namespace m545_volumetric_mapping_rviz_plugin {

class MeshVisual;

class MeshDisplay
    : public rviz::MessageFilterDisplay<m545_volumetric_mapping_msgs::PolygonMesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MeshDisplay();
  virtual ~MeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const m545_volumetric_mapping_msgs::PolygonMesh::ConstPtr& msg);

  std::unique_ptr<MeshVisual> visual_;
};

}  // namespace m545_volumetric_mapping_rviz_plugin

