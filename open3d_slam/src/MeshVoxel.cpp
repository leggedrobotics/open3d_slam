//
// Created by peyschen on 15/02/23.
//

#include "open3d_slam/MeshVoxel.hpp"
#include <open3d/visualization/utility/DrawGeometry.h>
#include <iostream>
#include <thread>
#include "delaunator.hpp"
#include "open3d_slam/helpers.hpp"

namespace o3d_slam {
void L2Voxel::initPlane() {
  std::vector<Eigen::Vector3d> pointSet = getPointSetFromIdx(pts_, parentMap_->allPts_);
  planePtr_->initialize(pointSet);
}
void L2Voxel::addPoint(int vert) {
  pts_.push_back(vert);
  Eigen::Vector3d center(0);
  for (auto pt : pts_) {
    center += (center_ - parentMap_->allPts_.points_.at(pt));
  }
  // std::cout << "avg dist to ctr " << center/pts_.size() << std::endl;

  isModified_ = true;
}

void MeshMap::addNewPointCloud(const PointCloud& pc) {
  PointCloudPtr downsampled = pc.VoxelDownSample(downsampleVoxelSize_);
  {
    std::lock_guard<std::mutex> lock{meshLock_};
    if (allPts_.points_.empty()) {
      allPts_ += *downsampled;
    } else {
      kdTree_.SetGeometry(allPts_);
      for (const auto& pt : downsampled->points_) {
        std::vector<int> indices;
        std::vector<double> distances;
        open3d::geometry::KDTreeSearchParamKNN params(30);
        kdTree_.Search(pt, params, indices, distances);
        if (*std::min_element(distances.begin(), distances.end()) > (newVertexThreshold_ * newVertexThreshold_)) {
          allPts_.points_.push_back(pt);
          Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(l2VoxelSize_));
          Eigen::Vector3d ctr = getVoxelCenter(idx, Eigen::Vector3d::Constant(l2VoxelSize_));
          if (l2Voxels_.find(idx) == l2Voxels_.end()) {
            l2Voxels_.insert(std::make_pair(idx, L2Voxel(l2VoxelSize_, this, ctr)));
          }
          l2Voxels_.at(idx).addPoint(allPts_.points_.size() - 1);
          kdTree_.SetGeometry(allPts_);
        }
      }
    }
  }
  mesh();
}

void MeshMap::mesh() {
  for (auto& it : l2Voxels_) {
    if (it.second.isUpdated() && it.second.getPoints().size() > 3) {
      std::vector<int> vertices = getVoxelVertexSet(it.second);
      std::vector<Triangle> newTris = triangulateVertexSetForVoxel(it.second, vertices);

      std::vector<Triangle> pulledTris;
      std::vector<int> pulledTriIdx;
      pullTriangles(vertices, pulledTris, pulledTriIdx);

      std::vector<Triangle> trisToAdd;
      std::unordered_set<int> trisToDelete;
      for (const auto& tri : newTris) {
        if (std::find(pulledTris.begin(), pulledTris.end(), tri) == pulledTris.end()) {
          trisToAdd.push_back(tri);
        }
      }
      for (int i = 0; i < pulledTris.size(); i++) {
        if (std::find(newTris.begin(), newTris.end(), pulledTris[i]) == newTris.end()) {
          trisToDelete.insert(pulledTriIdx[i]);
        }
      }
      for (const auto& tri : trisToAdd) {
        addTriangle(tri);
      }
      /*      for (const auto& tri : trisToDelete) {
              eraseTriangle(tri);
            }*/
      it.second.deactivate();
    }
  }
  std::cout << "MESH HAS " << triangles_.size() << " TRIANGLES" << std::endl;
}
void MeshMap::addTriangle(const Triangle& tri) {
  std::lock_guard<std::mutex> lock{meshLock_};
  int idx = triangles_.size();
  triangles_.insert(std::make_pair(idx, tri));
  vertexToTriangles_[tri.i].insert(idx);
  vertexToTriangles_[tri.j].insert(idx);
  vertexToTriangles_[tri.k].insert(idx);
}

void Print(const std::unordered_set<int>& vec) {
  for (const auto& i : vec) {
    std::cout << i << ' ';
  }
  std::cout << '\n';
}

void MeshMap::eraseTriangle(const int& triIdx) {
  std::lock_guard<std::mutex> lock{meshLock_};
  if (triangles_.find(triIdx) == triangles_.end()) {
    std::cout << "TRIANGLE " << triIdx << " NOT FOUND FOR DELETION" << std::endl;
    return;
  }
  Triangle t = triangles_.at(triIdx);
  vertexToTriangles_[t.i].erase(triIdx);
  vertexToTriangles_[t.j].erase(triIdx);
  vertexToTriangles_[t.k].erase(triIdx);
  /*std::cout << "IDX: " << triIdx << std::endl;
  Print(vertexToTriangles_[t.i]);
  Print(vertexToTriangles_[t.j]);
  Print(vertexToTriangles_[t.k]);*/

  triangles_.erase(triIdx);
  std::cout << "ERASE " << triIdx << std::endl;
}
void MeshMap::pullTriangles(std::vector<int>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<int>& pulledIdx) {
  std::lock_guard<std::mutex> lock{meshLock_};
  auto vertexSet = std::unordered_set<int>(vertices.begin(), vertices.end());
  for (const auto& vertex : vertices) {
    std::unordered_set<int> tris = getTriangleIndexesForVertex(vertex);
    if (tris.empty()) {
      continue;
    }
    for (const auto& tri : tris) {
      Triangle t = triangles_.at(tri);

      if ((vertexSet.count(t.i) + vertexSet.count(t.j) + vertexSet.count(t.k)) == 3) {
        pulledTriangles.push_back(t);
        pulledIdx.push_back(tri);
      }
    }
  }
}

std::vector<int> MeshMap::getVoxelVertexSet(const L2Voxel& voxel) {
  std::lock_guard<std::mutex> lock{meshLock_};
  std::unordered_set<int> vertices;
  appendToSet(vertices, voxel.getPoints());
  auto pts = getPointSetFromIdx(vertices, allPts_);
  for (const auto& pt : pts) {
    std::vector<int> indices;
    std::vector<double> distances;
    kdTree_.SearchRadius(pt, l2VoxelSize_ * 0.75, indices, distances);
    vertices.insert(indices.begin(), indices.end());
  }
  return std::vector<int>(vertices.begin(), vertices.end());
}

std::vector<Triangle> MeshMap::triangulateVertexSetForVoxel(L2Voxel& voxel, const std::vector<int>& vertices) const {
  auto ptr = voxel.getPlanePtr();
  Eigen::Vector3d q = ptr->getPlaneCenter();
  Eigen::Matrix<double, 3, 2> tangBase = ptr->getTangentialBase();
  auto meshVertices = getPointSetFromIdx(vertices, allPts_);
  std::vector<Eigen::Vector2d> projectedVertices;
  projectedVertices.reserve(meshVertices.size());

  auto projectToPlane = [&](const Eigen::Vector3d& pt) -> Eigen::Vector2d { return ((pt - q).transpose() * tangBase).transpose(); };
  for (const auto& vert : meshVertices) {
    projectedVertices.push_back(projectToPlane(vert));
  }
  std::vector<double> delaunayInput;
  delaunayInput.reserve(projectedVertices.size() * 2);
  for (const auto& pt : projectedVertices) {
    delaunayInput.push_back(pt.x());
    delaunayInput.push_back(pt.y());
  }
  delaunator::Delaunator delaunay(delaunayInput);
  std::vector<Triangle> triangles;
  for (size_t i = 0; i < delaunay.triangles.size(); i += 3) {
    // std::cout << delaunay.triangles[i] << "\t" << delaunay.triangles[i + 1] << "\t" << delaunay.triangles[i + 2] << std::endl;
    triangles.emplace_back(vertices[delaunay.triangles[i]], vertices[delaunay.triangles[i + 1]], vertices[delaunay.triangles[i + 2]]);
  }
  return triangles;
}

std::unordered_set<int> MeshMap::getTriangleIndexesForVertex(const int& vertex) {
  if (vertexToTriangles_.find(vertex) != vertexToTriangles_.end()) {
    return vertexToTriangles_.at(vertex);
  }
  return {};
}
open3d::geometry::TriangleMesh MeshMap::toO3dMesh() const {
  std::lock_guard<std::mutex> lock{meshLock_};

  open3d::geometry::TriangleMesh mesh;
  std::vector<Eigen::Vector3i> triList;
  mesh.vertices_ = allPts_.points_;
  triList.reserve(triangles_.size());
  for (const auto& tri : triangles_) {
    triList.push_back(tri.second.toEigen());
  }
  mesh.triangles_ = triList;
  mesh = mesh.RemoveUnreferencedVertices();
  auto colors = std::vector<Eigen::Vector3d>(mesh.vertices_.size());
  mesh.vertex_colors_ = colors;
  mesh.vertex_normals_ = colors;
  return mesh;
}
PointCloud MeshMap::getVoxelCloud() const {
  PointCloud voxelCenters;
  for (const auto& it : l2Voxels_) {
    voxelCenters.points_.push_back(it.second.getCenter());
  }
  return voxelCenters;
}
Eigen::Vector3i MeshMap::voxelIdx(const Eigen::Vector3d& p) const {
  Eigen::Vector3d coordinate = (p.array() * 100) / l2VoxelSize_;
  return {int(std::floor(coordinate.x())), int(std::floor(coordinate.y())), int(std::floor(coordinate.z()))};
}

Eigen::Vector3d MeshMap::voxelCenter(const Eigen::Vector3i& i) const {
  return ((i.cast<double>() * l2VoxelSize_) / 100) + 0.5 * Eigen::Vector3d::Constant(l2VoxelSize_);
}

}  // namespace o3d_slam
