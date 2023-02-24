//
// Created by peyschen on 15/02/23.
//

#include "open3d_slam/MeshVoxel.hpp"
#include <iostream>
#include "delaunator.hpp"
#include "open3d_slam/helpers.hpp"

namespace o3d_slam {
void MeshVoxel::initPlane() {
  std::vector<Eigen::Vector3d> pointSet = parentMap_->getPoints(pts_);
  planePtr_->initialize(pointSet);
}
void MeshVoxel::addPoint(size_t vert) {
  pts_.push_back(vert);
  isModified_ = true;
}
void MeshVoxel::removePoint(size_t vert) {
  pts_.erase(std::remove(pts_.begin(), pts_.end(), vert), pts_.end());
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

    if (points_.empty()) {
      for (const auto& pt : downsampled->points_) {
        points_.insert(PointPair(nextVertIdx_++, pt));
        Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(voxelSize_));
        if (voxels_.find(idx) == voxels_.end()) {
          voxels_.insert(std::make_pair(idx, MeshVoxel(voxelSize_, this)));
        }
        voxels_.at(idx).addPoint(nextVertIdx_ - 1);
      }
      ikdTree_->Build(downsampled->points_);
    } else {
      for (const auto& pt : downsampled->points_) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> distances;
        distances.reserve(20);
        pts.reserve(20);
        ikdTree_->Nearest_Search(pt, 20, pts, distances);
        if (*std::min_element(distances.begin(), distances.end()) >= (newVertexThreshold_ * newVertexThreshold_)) {
          points_.insert(PointPair(nextVertIdx_++, pt));
          Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(voxelSize_));
          if (voxels_.find(idx) == voxels_.end()) {
            voxels_.insert(std::make_pair(idx, MeshVoxel(voxelSize_, this)));
          }
          voxels_.at(idx).addPoint(nextVertIdx_ - 1);
          std::vector<Eigen::Vector3d> ptAdd = {pt};
          ikdTree_->Add_Points(ptAdd, false);
        }
      }
    }
  }
  const double elapsed = addingTimer_.elapsedMsecSinceStopwatchStart();
  addingTimer_.addMeasurementMsec(elapsed);
}
void MeshMap::removePoints(const PointCloud& pts) {
  std::unordered_set<size_t> trisToDelete;
  {
    std::unique_lock<std::mutex> vertLock{vertexLock_, std::defer_lock};
    std::unique_lock<std::mutex> voxLock{voxelLock_, std::defer_lock};
    std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
    std::lock(voxLock, vertLock, vttLock);
    for (const auto& pt : pts.points_) {
      std::vector<Eigen::Vector3d> nearest;
      std::vector<double> distances;
      ikdTree_->Nearest_Search(pt, 1, nearest, distances);
      if (!nearest.empty()) {
        Eigen::Vector3d toRemove = nearest[0];
        Eigen::Vector3i voxelIdx = getVoxelIdx(toRemove, Eigen::Vector3d::Constant(voxelSize_));

        auto itToRemove = points_.right.find(toRemove);
        if (itToRemove != points_.right.end()) {
          size_t ptIdx = itToRemove->second;
          if (vertexToTriangles_.find(ptIdx) != vertexToTriangles_.end()) {
            trisToDelete.insert(vertexToTriangles_.at(ptIdx).begin(), vertexToTriangles_.at(ptIdx).end());
          }
          if (voxels_.find(voxelIdx) != voxels_.end()) {
            voxels_.at(voxelIdx).removePoint(ptIdx);
          }
          points_.left.erase(ptIdx);
        }
        ikdTree_->Delete_Points(nearest);
      } else {
        std::cout << "no point found" << std::endl;
      }
    }
  }
    for (const auto& tri : trisToDelete) {
      eraseTriangle(tri);
    }
    std::cout << "Removed " << pts.points_.size() << " points." << std::endl;

  mesh();
}

void MeshMap::cleanup() {
  int cnt = 0;
  for (auto tri : triangles_) {
    if (tri.second.isDegenerate()) {
      eraseTriangle(tri.first);
      cnt++;
    }
  }
  // std::cout << "removed " << cnt << " degenerate tris" << std::endl;
}
void MeshMap::mesh() {
  std::lock_guard<std::mutex> lck{meshLock_};
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

//#pragma omp parallel for default(none) shared(updateIndices, globalTrisToAdd, globalTrisToDelete, globalListMutex)
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
      std::lock_guard<std::mutex> gll{globalListMutex};
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
  for (const auto& tri : globalTrisToAdd) {
    addTriangle(tri);
  }
  for (const auto& triIdx : globalTrisToDelete) {
    eraseTriangle(triIdx);
  }
  const double elapsed = meshingTimer_.elapsedMsecSinceStopwatchStart();
  meshingTimer_.addMeasurementMsec(elapsed);
  meshCount_++;
}

void MeshMap::addTriangle(const Triangle& tri) {
  std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
  std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
  std::lock(triLock, vttLock);
  size_t idx = nextTriIdx_++;
  triangles_.insert(std::make_pair(idx, tri));
  vertexToTriangles_[tri.i].insert(idx);
  vertexToTriangles_[tri.j].insert(idx);
  vertexToTriangles_[tri.k].insert(idx);
}
void MeshMap::eraseTriangle(const size_t& triIdx) {
  std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
  std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
  std::lock(triLock, vttLock);
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
    {
      std::lock_guard<std::mutex> lck{triangleLock_};
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
}

std::vector<size_t> MeshMap::getVoxelVertexSet(const MeshVoxel& voxel) {
  std::unordered_set<size_t> vertices;
  appendToSet(vertices, voxel.getPoints());
  auto pts = getPoints(std::vector<size_t>(vertices.begin(), vertices.end()));
  {
    std::lock_guard<std::mutex> lck{vertexLock_};
    for (const auto& pt : pts) {
      std::vector<Eigen::Vector3d> ptSearch;
      ikdTree_->Radius_Search(pt, voxelSize_ * dilationRatio_, ptSearch);
      for (const auto& p : ptSearch) {
        auto ptIdx = points_.right.find(p)->second;
        vertices.insert(ptIdx);
      }
    }
  }
  return {vertices.begin(), vertices.end()};
}
std::vector<Triangle> MeshMap::triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<size_t>& vertices) const {
  auto ptr = voxel.getPlanePtr();
  ptr->initialize(getPoints(voxel.getPoints()));
  Eigen::Vector3d q = ptr->getPlaneCenter();
  Eigen::Matrix<double, 3, 2> tangBase = ptr->getTangentialBase();
  auto meshVertices = getPoints(vertices);
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
      Eigen::Vector3d AB = meshVertices[delaunay.triangles[i + 1]] - meshVertices[delaunay.triangles[i]];
      Eigen::Vector3d AC = meshVertices[delaunay.triangles[i + 2]] - meshVertices[delaunay.triangles[i]];
      Eigen::Vector3d BC = meshVertices[delaunay.triangles[i + 2]] - meshVertices[delaunay.triangles[i + 1]];
      double area = (AB.cross(AC)).norm() / 2;
      double perim = AB.norm() + AC.norm() + BC.norm();
      double sliverParameter = (2 * area) / perim;
      if (sliverParameter > sliverThreshold_) {
        triangles.emplace_back(vertices[delaunay.triangles[i]], vertices[delaunay.triangles[i + 1]], vertices[delaunay.triangles[i + 2]]);
      }
    }
  } catch (std::runtime_error& e) {
    // could not triangulate, but we don't care
  }
  return triangles;
}
std::vector<Eigen::Vector3d> MeshMap::getPoints(const std::vector<size_t>& vertices) const {
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(vertices.size());
  {
    std::lock_guard<std::mutex> lck{vertexLock_};
    for (const auto& vert : vertices) {
      pts.push_back(points_.left.at(vert));
    }
  }
  return pts;
}
std::unordered_set<size_t> MeshMap::getTriangleIndexesForVertex(const size_t& vertex) {
  std::lock_guard<std::mutex> lck{verToTriLock_};
  if (vertexToTriangles_.find(vertex) != vertexToTriangles_.end()) {
    return vertexToTriangles_.at(vertex);
  }
  return {};
}

open3d::geometry::TriangleMesh MeshMap::toO3dMesh() const {
  open3d::geometry::TriangleMesh mesh;
  std::unordered_map<size_t, size_t> vertIdxToMeshIdx;
  std::vector<Eigen::Vector3i> triList;
  {
    std::lock_guard<std::mutex> lck{vertexLock_};
    for (const auto& vert : points_.left) {
      vertIdxToMeshIdx.insert(std::make_pair(vert.first, mesh.vertices_.size()));
      mesh.vertices_.push_back(vert.second);
    }
  }
  triList.reserve(triangles_.size());
  {
    std::lock_guard<std::mutex> lck{triangleLock_};
    for (const auto& tri : triangles_) {
      Triangle t = tri.second;
      triList.emplace_back(vertIdxToMeshIdx.at(t.i), vertIdxToMeshIdx.at(t.j), vertIdxToMeshIdx.at(t.k));
    }
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
  std::cout << "Mesh has " << triangles_.size() << " triangles" << std::endl;
}
void MeshMap::updateParameters(MeshingParameters params) {
  downsampleVoxelSize_ = params.downsamplingVoxelSize_;
  voxelSize_ = params.meshingVoxelSize_;
  newVertexThreshold_ = params.newVertexDistanceThreshold_;
}
}  // namespace o3d_slam
