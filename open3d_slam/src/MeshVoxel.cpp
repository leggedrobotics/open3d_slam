//
// Created by peyschen on 15/02/23.
//

#include "open3d_slam/MeshVoxel.hpp"
#include <open3d/visualization/utility/DrawGeometry.h>
#include <iostream>
#include "delaunator.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/helpers.hpp"

namespace o3d_slam {
void MeshVoxel::initPlane() {
  std::vector<Eigen::Vector3d> pointSet = getPointSetFromIdx(pts_, parentMap_->allPts_);
  planePtr_->initialize(pointSet);
}
void MeshVoxel::addPoint(int vert) {
  pts_.push_back(vert);
  isModified_ = true;
}

void MeshMap::addNewPointCloud(const PointCloud& pc) {
  PointCloudPtr downsampled = pc.VoxelDownSample(downsampleVoxelSize_);
  {
    std::lock_guard<std::mutex> lock{meshLock_};
    if (allPts_.points_.empty()) {
      allPts_ += *downsampled;
      ikdTree_->Build(allPts_.points_);
    } else {
      for (const auto& pt : downsampled->points_) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> distances;
        distances.reserve(50);
        pts.reserve(50);
        ikdTree_->Nearest_Search(pt, 50, pts, distances);
        if (*std::min_element(distances.begin(), distances.end()) >= (newVertexThreshold_ * newVertexThreshold_)) {
          allPts_.points_.push_back(pt);
          Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(l2VoxelSize_));
          if (l2Voxels_.find(idx) == l2Voxels_.end()) {
            l2Voxels_.insert(std::make_pair(idx, MeshVoxel(l2VoxelSize_, this)));
          }
          l2Voxels_.at(idx).addPoint(allPts_.points_.size() - 1);
          std::vector<Eigen::Vector3d> ptAdd = {pt};
          ikdTree_->Add_Points(ptAdd, false);
        }
      }
    }
  }
  mesh();
}

void MeshMap::mesh() {
  std::mutex globalListMutex;
  std::vector<Triangle> globalTrisToAdd;
  std::unordered_set<int> globalTrisToDelete;
  std::vector<Eigen::Vector3i> updateIndices;
  for (auto& it : l2Voxels_) {
    if (it.second.isUpdated() && it.second.getPoints().size() > 3) {
      updateIndices.push_back(it.first);
    }
  }

#pragma omp parallel for default(none) shared(updateIndices, globalTrisToAdd, globalTrisToDelete,globalListMutex)
  for (const auto& idx : updateIndices) {
    std::vector<int> vertices = getVoxelVertexSet(l2Voxels_.at(idx));
    std::vector<Triangle> newTris = triangulateVertexSetForVoxel(l2Voxels_.at(idx), vertices);

    std::vector<Triangle> pulledTris;
    std::vector<int> pulledTriIdx;
    pullTriangles(vertices, pulledTris, pulledTriIdx);

    for (const auto& tri : newTris) {
      if (std::find(pulledTris.begin(), pulledTris.end(), tri) == pulledTris.end()) {
        std::lock_guard<std::mutex> lck{globalListMutex};
        globalTrisToAdd.push_back(tri);
      }
    }
    for (int j = 0; j < pulledTris.size(); j++) {
      std::lock_guard<std::mutex> lck{globalListMutex};
      if (std::find(newTris.begin(), newTris.end(), pulledTris.at(j)) == newTris.end()) {
        globalTrisToDelete.insert(pulledTriIdx[j]);
      }
    }
    l2Voxels_.at(idx).deactivate();
  }

  /*for (const auto& tri : globalTrisToDelete) {
    eraseTriangle(tri);
  }*/
  for (const auto& tri : globalTrisToAdd) {
    addTriangle(tri);
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

void MeshMap::eraseTriangle(const int& triIdx) {
  std::lock_guard<std::mutex> lock{meshLock_};
  if (triangles_.find(triIdx) == triangles_.end()) {
    //std::cout << "TRIANGLE " << triIdx << " NOT FOUND FOR DELETION" << std::endl;
    return;
  }
  Triangle t = triangles_.at(triIdx);
  vertexToTriangles_[t.i].erase(triIdx);
  vertexToTriangles_[t.j].erase(triIdx);
  vertexToTriangles_[t.k].erase(triIdx);
  triangles_.erase(triIdx);
  std::cout << "ERASE " << triIdx << std::endl;
}
void MeshMap::pullTriangles(std::vector<int>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<int>& pulledIdx) {
  std::lock_guard<std::mutex> lock{meshLock_};
  auto vertexSet = std::unordered_set<int>(vertices.begin(), vertices.end());
  for (const auto& vertex : vertices) {
    std::unordered_set<int> tris = getTriangleIndexesForVertex(vertex);
    if (tris.empty()) {
      //std::cout << "GOT NO TRIANGLES" << std::endl;
      continue;
    }
    for (const auto& tri : tris) {
      //std::cout << "GOT TRIANGLES" << std::endl;
      Triangle t = triangles_.at(tri);

      if ((vertexSet.count(t.i) + vertexSet.count(t.j) + vertexSet.count(t.k)) == 3) {
        //std::cout << "FOUND TRI [" << t.i << ", " << t.j << ", " << t.k << "]" << std::endl;
        pulledTriangles.push_back(t);
        pulledIdx.push_back(tri);
      }
    }
  }
}

std::vector<int> MeshMap::getVoxelVertexSet(const MeshVoxel& voxel) {
  std::lock_guard<std::mutex> lock{meshLock_};
  std::unordered_set<int> vertices;
  appendToSet(vertices, voxel.getPoints());
  auto pts = getPointSetFromIdx(vertices, allPts_);
  for (const auto& pt : pts) {
    std::vector<Eigen::Vector3d> ptSearch;
    ikdTree_->Radius_Search(pt, l2VoxelSize_ * 1, ptSearch);
    for (const auto& p : ptSearch) {
      auto it = std::find(allPts_.points_.begin(), allPts_.points_.end(), p);
      long idx = it - allPts_.points_.begin();
      vertices.insert((int)idx);
    }
  }
  return {vertices.begin(), vertices.end()};
}

std::vector<Triangle> MeshMap::triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<int>& vertices) const {
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
  //std::cout << "FOR VERTEX " << vertex << ": ";
  if (vertexToTriangles_.find(vertex) != vertexToTriangles_.end()) {
   // std::cout << "FOUND CORRESPONDANCE" << std::endl;
    return vertexToTriangles_.at(vertex);
  }
  //std::cout << "NO CORRESPONDANCE" << std::endl;
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
  mesh = mesh.RemoveDuplicatedTriangles();
  mesh = mesh.RemoveDuplicatedVertices();
  mesh = mesh.RemoveDegenerateTriangles();
  auto colors = std::vector<Eigen::Vector3d>(mesh.vertices_.size());
  mesh.vertex_colors_ = colors;
  mesh.vertex_normals_ = colors;
  return mesh;
}
void MeshMap::removePoints(PointCloud& cloud) {
  std::vector<Eigen::Vector3d> removePts;
  removePts.reserve(cloud.points_.size());
  for (const auto& pt : cloud.points_) {
    std::vector<Eigen::Vector3d> pts;
    std::vector<double> distances;
    ikdTree_->Nearest_Search(pt, 50, pts, distances);
    removePts.push_back(pts[0]);
  }
  allPts_.points_.erase(std::remove_if( allPts_.points_.begin(),  allPts_.points_.end(), [&](const auto&x) {
                  return std::find(removePts.begin(), removePts.end(), x) != removePts.end();
                }),  allPts_.points_.end());
  ikdTree_->Delete_Points(removePts);
}
}  // namespace o3d_slam
