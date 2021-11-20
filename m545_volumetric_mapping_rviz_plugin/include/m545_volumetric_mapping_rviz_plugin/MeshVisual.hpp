#pragma once

#include <OGRE/OgreManualObject.h>

#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_volumetric_mapping_rviz_plugin {

/// Visualizes a single voxblox_msgs::Mesh message.
class MeshVisual {
 public:
	MeshVisual(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node);
  virtual ~MeshVisual();

  void setMessage(const m545_volumetric_mapping_msgs::PolygonMesh::ConstPtr& msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);
//  void visualizeMesh(const m545_volumetric_mapping_msgs::PolygonMesh::ConstPtr& msg);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;
  Ogre::ManualObject* object_;
  bool firstTime_=true;
//  voxblox::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace m545_volumetric_mapping_rviz_plugin

