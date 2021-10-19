#include "m545_volumetric_mapping_rviz_plugin/MeshVisual.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <limits>
#include "open3d_conversions/open3d_conversions.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

namespace m545_volumetric_mapping_rviz_plugin {

unsigned int MeshVisual::instance_counter_ = 0;

MeshVisual::MeshVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node) {
	scene_manager_ = scene_manager;
	frame_node_ = parent_node->createChildSceneNode();
	instance_number_ = instance_counter_++;
	object_ = scene_manager_->createManualObject("mesh");

}

MeshVisual::~MeshVisual() {
	// Destroy all the objects
	scene_manager_->destroyManualObject(object_);
}

void MeshVisual::setMessage(const m545_volumetric_mapping_msgs::PolygonMesh::ConstPtr &msg) {
//	auto 	startTime_ = std::chrono::steady_clock::now();
	open3d::geometry::TriangleMesh mesh;
	open3d_conversions::rosToOpen3d(msg, mesh);
	// calculate normals
	mesh.ComputeVertexNormals();

	object_->clear();

	if (firstTime_) {
		frame_node_->attachObject(object_);
		firstTime_ = false;
	}

	if (object_ == nullptr) {
		throw std::runtime_error("Ogre Could not create object.");
	}

	object_->estimateVertexCount(mesh.vertices_.size());
	object_->estimateIndexCount(mesh.triangles_.size());
	object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

	for (size_t i = 0; i < mesh.vertices_.size(); ++i) {
		object_->position(mesh.vertices_[i].x(), mesh.vertices_[i].y(), mesh.vertices_[i].z());
		object_->normal(mesh.vertex_normals_[i].x(), mesh.vertex_normals_[i].y(), mesh.vertex_normals_[i].z());


		Eigen::Vector3d color(0.0,0.0,0.0);
		const bool isHasColors = false;
	      if (isHasColors) {
	        color = mesh.vertex_colors_.at(i);
	      } else {
	        // reconstruct normals coloring
	        color.x() = mesh.vertex_normals_[i].x() * 0.5f + 0.5f;
	        color.y() = mesh.vertex_normals_[i].y() * 0.5f + 0.5f;
	        color.z() = mesh.vertex_normals_[i].z() * 0.5f + 0.5f;
	      }

		object_->colour(static_cast<float>(color.x()), static_cast<float>(color.y()),
				static_cast<float>(color.z()), static_cast<float>(255));
	}

	// needed for anything other than flat rendering
	int ogreTriangleIdx = 0;
	for (int i = 0; i < mesh.triangles_.size(); ++i) {
		const auto &index = mesh.triangles_[i];
        object_->triangle(index.x(), index.y(), index.z());
//        object_->index(index.x());
//        object_->index(index.y());
//        object_->index(index.z());

	}
	object_->end();
//	const auto endTime = std::chrono::steady_clock::now();
//	double msec = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count() / 1e3;
//	std::cerr << "rendering took: " << msec <<" msec \n";
}

void MeshVisual::setFramePosition(const Ogre::Vector3 &position) {
	frame_node_->setPosition(position);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
	frame_node_->setOrientation(orientation);
}

}  // namespace m545_volumetric_mapping_rviz_plugin
