//
// Created by peyschen on 15/02/23.
//

#include "open3d_slam/MeshVoxel.hpp"
#include <iostream>
#include "delaunator.hpp"
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
  addingTimer_.startStopwatch();
  auto cleaned = pc.RemoveStatisticalOutliers(50, 1);
  PointCloudPtr downsampled = std::get<0>(cleaned)->VoxelDownSample(downsampleVoxelSize_);

  {
    std::unique_lock<std::mutex> vertLock{vertexLock_, std::defer_lock};
    std::unique_lock<std::mutex> voxLock{voxelLock_, std::defer_lock};
    std::lock(voxLock, vertLock);

    if (allPts_.points_.empty()) {
      allPts_ += *downsampled;
      ikdTree_->Build(allPts_.points_);
    } else {
      for (const auto& pt : downsampled->points_) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> distances;
        distances.reserve(20);
        pts.reserve(20);
        ikdTree_->Nearest_Search(pt, 20, pts, distances);
        if (*std::min_element(distances.begin(), distances.end()) >= (newVertexThreshold_ * newVertexThreshold_)) {
          allPts_.points_.push_back(pt);
          Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(voxelSize_));
          if (voxels_.find(idx) == voxels_.end()) {
            voxels_.insert(std::make_pair(idx, MeshVoxel(voxelSize_, this)));
          }
          voxels_.at(idx).addPoint(allPts_.points_.size() - 1);
          std::vector<Eigen::Vector3d> ptAdd = {pt};
          ikdTree_->Add_Points(ptAdd, false);
        }
      }
    }
  }
  const double elapsed = addingTimer_.elapsedMsecSinceStopwatchStart();
  addingTimer_.addMeasurementMsec(elapsed);
  // mesh();
}

void MeshMap::cleanup() {
  int cnt = 0;
  for (auto tri : triangles_) {
    if (tri.second.isDegenerate()) {
      eraseTriangle(tri.first);
      cnt++;
    }
  }
  std::cout << "removed " << cnt << " degenerate tris" << std::endl;
}

void MeshMap::mesh() {
  if (meshCount_ == 5) {
    cleanup();
    meshCount_ = 0;
  }
  meshingTimer_.startStopwatch();
  std::mutex globalListMutex;
  std::vector<Triangle> globalTrisToAdd;
  std::unordered_set<size_t> globalTrisToDelete;
  std::vector<Eigen::Vector3i> updateIndices;
  for (auto& it : voxels_) {
    if (it.second.isUpdated()) {
      updateIndices.push_back(it.first);
    }
  }

#pragma omp parallel for default(none) shared(updateIndices, globalTrisToAdd, globalTrisToDelete, globalListMutex)
  for (const auto& idx : updateIndices) {
    std::vector<size_t> vertices = getVoxelVertexSet(voxels_.at(idx));
    if (vertices.size() < 3) {
      continue;
    }
    std::vector<Triangle> newTris = triangulateVertexSetForVoxel(voxels_.at(idx), vertices);
    std::vector<Triangle> pulledTris;
    std::vector<size_t> pulledTriIdx;
    pullTriangles(vertices, pulledTris, pulledTriIdx);
    {
      std::lock_guard<std::mutex> lck{globalListMutex};
      for (int j = 0; j < pulledTris.size(); j++) {
        if (std::find(newTris.begin(), newTris.end(), pulledTris.at(j)) == newTris.end()) {
          globalTrisToDelete.insert(pulledTriIdx[j]);
        }
      }

      for (const auto& tri : newTris) {
        if (std::find(pulledTris.begin(), pulledTris.end(), tri) == pulledTris.end()) {
          globalTrisToAdd.push_back(tri);
        }
      }
    }
    voxels_.at(idx).deactivate();
  }
  {
    std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
    std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
    std::lock(triLock, vttLock);
    for (const auto& tri : globalTrisToAdd) {
      addTriangle(tri);
    }
    for (const auto& triIdx : globalTrisToDelete) {
      eraseTriangle(triIdx);
    }
  }
  std::cout << "MESH HAS " << triangles_.size() << " TRIANGLES" << std::endl;
  const double elapsed = meshingTimer_.elapsedMsecSinceStopwatchStart();
  meshingTimer_.addMeasurementMsec(elapsed);
  meshCount_++;
}
void MeshMap::addTriangle(const Triangle& tri) {
  size_t idx = nextTriIdx_++;
  triangles_.insert(std::make_pair(idx, tri));
  vertexToTriangles_[tri.i].insert(idx);
  vertexToTriangles_[tri.j].insert(idx);
  vertexToTriangles_[tri.k].insert(idx);
}

void MeshMap::eraseTriangle(const size_t& triIdx) {
  if (triangles_.find(triIdx) == triangles_.end()) {
    return;
  }
  Triangle t = triangles_.at(triIdx);
  bool isDeleted = true;
  isDeleted &= vertexToTriangles_[t.i].erase(triIdx);
  isDeleted &= vertexToTriangles_[t.j].erase(triIdx);
  isDeleted &= vertexToTriangles_[t.k].erase(triIdx);
  triangles_.erase(triIdx);
  if (!isDeleted) {
    std::cout << "SOMETHING WENT WRONG DELETING!!!" << std::endl;
    throw std::logic_error("Delete");
  }
}
void MeshMap::pullTriangles(const std::vector<size_t>& vertices, std::vector<Triangle>& pulledTriangles, std::vector<size_t>& pulledIdx) {
  std::lock_guard<std::mutex> lck{triangleLock_};
  std::unordered_set<size_t> vertexSet;
  vertexSet.reserve(vertices.size());
  for (const auto& vert : vertices) {
    vertexSet.insert(vert);
  }

  for (const auto& vertex : vertices) {
    std::unordered_set<size_t> tris = getTriangleIndexesForVertex(vertex);
    if (tris.empty()) {
      continue;
    }
    for (const auto& tri : tris) {
      try {
        Triangle t = triangles_.at(tri);

        if ((vertexSet.find(t.i) != vertexSet.end()) && (vertexSet.find(t.j) != vertexSet.end()) &&
            (vertexSet.find(t.k) != vertexSet.end())) {
          pulledTriangles.push_back(t);
          pulledIdx.push_back(tri);
        }
      } catch (std::out_of_range& e) {
        std::cout << "Tried to access non-existing triangle " << tri << std::endl;
        std::cout << "Vertex " << vertex << " has triangles: [ ";
        for (auto t : vertexToTriangles_[vertex]) std::cout << t << " ";
        std::cout << "]" << std::endl;
        throw e;
      }
    }
  }
}

std::vector<size_t> MeshMap::getVoxelVertexSet(const MeshVoxel& voxel) {
  std::lock_guard<std::mutex> lck{vertexLock_};
  std::unordered_set<size_t> vertices;
  appendToSet(vertices, voxel.getPoints());
  auto pts = getPointSetFromIdx(vertices, allPts_);
  for (const auto& pt : pts) {
    std::vector<Eigen::Vector3d> ptSearch;
    ikdTree_->Radius_Search(pt, voxelSize_ * dilationRatio_, ptSearch);
    for (const auto& p : ptSearch) {
      auto it = std::find(allPts_.points_.begin(), allPts_.points_.end(), p);
      long idx = it - allPts_.points_.begin();
      vertices.insert((int)idx);
    }
  }
  return {vertices.begin(), vertices.end()};
}

std::vector<Triangle> MeshMap::triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<size_t>& vertices) const {
  auto ptr = voxel.getPlanePtr();
  ptr->initialize(getPointSetFromIdx(voxel.getPoints(), allPts_));
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
  std::vector<Triangle> triangles;
  try {
    delaunator::Delaunator delaunay(delaunayInput);

    for (size_t i = 0; i < delaunay.triangles.size(); i += 3) {
      triangles.emplace_back(vertices[delaunay.triangles[i]], vertices[delaunay.triangles[i + 1]], vertices[delaunay.triangles[i + 2]]);
    }
  } catch(std::runtime_error& e){
    //pass
  }
  return triangles;
}

std::unordered_set<size_t> MeshMap::getTriangleIndexesForVertex(const size_t& vertex) {
  std::lock_guard<std::mutex> lck{verToTriLock_};
  if (vertexToTriangles_.find(vertex) != vertexToTriangles_.end()) {
    return vertexToTriangles_.at(vertex);
  }
  return {};
}
open3d::geometry::TriangleMesh MeshMap::toO3dMesh() const {
  std::unique_lock<std::mutex> vertL{vertexLock_, std::defer_lock};
  std::unique_lock<std::mutex> triL{triangleLock_, std::defer_lock};
  std::lock(vertL, triL);

  open3d::geometry::TriangleMesh mesh;
  std::vector<Eigen::Vector3i> triList;
  mesh.vertices_ = allPts_.points_;
  triList.reserve(triangles_.size());
  for (const auto& tri : triangles_) {
    triList.push_back(tri.second.toEigen());
  }
  mesh.triangles_ = triList;
  /*  mesh = mesh.RemoveUnreferencedVertices();
    mesh = mesh.RemoveDuplicatedTriangles();
    mesh = mesh.RemoveDuplicatedVertices();
    mesh = mesh.RemoveDegenerateTriangles();
    mesh = mesh.RemoveNonManifoldEdges();*/

  auto colors = std::vector<Eigen::Vector3d>(mesh.vertices_.size());
  mesh.vertex_colors_ = colors;
  mesh.vertex_normals_ = colors;
  return mesh;
}

void MeshMap::printMeshingStats() {
  std::cout << "Meshing timing stats ADDING: Avg execution time: " << addingTimer_.getAvgMeasurementMsec()
            << " msec , frequency: " << 1e3 / addingTimer_.getAvgMeasurementMsec() << " Hz \n";
  addingTimer_.reset();
  std::cout << "Meshing timing stats MESHING: Avg execution time: " << meshingTimer_.getAvgMeasurementMsec()
            << " msec , frequency: " << 1e3 / meshingTimer_.getAvgMeasurementMsec() << " Hz \n";
  meshingTimer_.reset();
}

void MeshMap::updateParameters(MeshingParameters params) {
  downsampleVoxelSize_ = params.downsamplingVoxelSize_;
  voxelSize_ = params.meshingVoxelSize_;
  newVertexThreshold_ = params.newVertexDistanceThreshold_;
}
}  // namespace o3d_slam
