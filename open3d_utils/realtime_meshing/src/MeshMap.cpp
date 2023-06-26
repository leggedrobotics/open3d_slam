#include "realtime_meshing/MeshMap.h"
#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/TriangleMesh.h>
#include "realtime_meshing/helpers.h"
#include "realtime_meshing/MeshVoxel.hpp"

MeshMap::MeshMap(double downsampleSize, double newVertexThreshold, double voxelSize, bool shouldFilter, double filterEps,
                 double filterRadius, int voxelMaxUpdates, double sliverDeletionThreshold)
    : downsampleVoxelSize_(downsampleSize),
      newVertexThreshold_(newVertexThreshold),
      voxelSize_(voxelSize),
      shouldFilter_(shouldFilter),
      filterEps_(filterEps),
      filterRadius_(filterRadius),
      voxelMaxUpdateCount_(voxelMaxUpdates),
      sliverThreshold_(sliverDeletionThreshold) {
  std::cout << "#########################\n"
            << "## MeshMap initialized ##\n"
            << "#########################\n";
  ikdTree_ = std::make_unique<ikdTree::KdTree<double>>(0.3, 0.6, 0.01);
}

void MeshMap::addNewPointCloud(const o3d_slam::PointCloud& pc) {
  o3d_slam::PointCloudPtr downsampled = pc.VoxelDownSample(downsampleVoxelSize_);
  if (shouldFilter_) {
    downsampled = guidedFiltering(downsampled, filterEps_, filterRadius_);
  }
  downsampled = std::get<0>(downsampled->RemoveStatisticalOutliers(25, 1));
  {
    std::lock_guard<std::mutex> lck{meshCloudLock_};
    mesherInput_ = downsampled;
  }

  {
    std::unique_lock<std::mutex> vertLock{vertexLock_, std::defer_lock};
    std::unique_lock<std::mutex> voxLock{voxelLock_, std::defer_lock};
    std::lock(voxLock, vertLock);

    if (points_.empty()) {
      for (const auto& pt : downsampled->points_) {
        insertPoint(pt);
      }
      ikdTree_->build(downsampled->points_);
    } else {
      for (const auto& pt : downsampled->points_) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> distances;
        distances.reserve(20);
        pts.reserve(20);
        ikdTree_->searchNearest(pt, 20, pts, distances);
        if (*std::min_element(distances.begin(), distances.end()) >= (newVertexThreshold_ * newVertexThreshold_)) {
          insertPoint(pt);
          ikdTree_->addPoint(pt, false);
        }
      }
    }
  }
}
void MeshMap::insertPoint(const Eigen::Vector3d& pt) {
  points_.insert(PointPair(nextVertIdx_++, pt));
  Eigen::Vector3i idx = getVoxelIdx(pt, Eigen::Vector3d::Constant(voxelSize_));
  if (voxels_.find(idx) == voxels_.end()) {
    voxels_.insert(std::make_pair(idx, MeshVoxel(voxelSize_, voxelMaxUpdateCount_, this)));
  }
  voxels_.at(idx).addPoint(nextVertIdx_ - 1);
}
void MeshMap::removePoints(const o3d_slam::PointCloud& pts) {
  auto mapBbox = ikdTree_->treeRange();
  open3d::geometry::AxisAlignedBoundingBox bbox(Eigen::Vector3d(mapBbox.minVertex), Eigen::Vector3d(mapBbox.maxVertex));
  auto cropped = pts.Crop(bbox);
  int removeCtr = 0;
  std::unordered_set<size_t> trisToDelete;
  std::set<size_t> removedPts;
  {
    std::unique_lock<std::mutex> vertLock{vertexLock_, std::defer_lock};
    std::unique_lock<std::mutex> voxLock{voxelLock_, std::defer_lock};
    std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
    std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
    std::lock(voxLock, vertLock, vttLock, triLock);
    for (const auto& pt : cropped->points_) {
      std::vector<Eigen::Vector3d> nearest;
      std::vector<double> distances;
      ikdTree_->searchNearest(pt, 1, nearest, distances);
      if (!nearest.empty() && distances[0] <= 2 * (voxelSize_ * voxelSize_)) {
        Eigen::Vector3d toRemove = nearest[0];
        Eigen::Vector3i voxelIdx = getVoxelIdx(toRemove, Eigen::Vector3d::Constant(voxelSize_));

        auto itToRemove = points_.right.find(toRemove);
        if (itToRemove != points_.right.end()) {
          std::unordered_set<size_t> potentialTrisToDelete;
          size_t ptIdx = itToRemove->second;
          if (vertexToTriangles_.find(ptIdx) != vertexToTriangles_.end()) {
            potentialTrisToDelete.insert(vertexToTriangles_.at(ptIdx).begin(), vertexToTriangles_.at(ptIdx).end());
          }
          bool gotRemoved = false;
          if (voxels_.find(voxelIdx) != voxels_.end()) {
            gotRemoved = voxels_.at(voxelIdx).removePoint(ptIdx);
          }
          if (gotRemoved) {
            removeCtr++;
            points_.left.erase(ptIdx);
            ikdTree_->deletePoints(nearest);
            removedPts.insert(ptIdx);
            trisToDelete.insert(potentialTrisToDelete.begin(), potentialTrisToDelete.end());
          }
        }
      }
    }
  }

  std::set<size_t> triPoints;
  {
    std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
    std::unique_lock<std::mutex> vttLock{verToTriLock_, std::defer_lock};
    std::lock(triLock, vttLock);
    for (const auto& tri : trisToDelete) {
      auto tv = triangles_.at(tri).toVector();
      triPoints.insert(tv.begin(), tv.end());
      eraseTriangle(tri);
    }
  }
  std::vector<size_t> remainingPts;
  std::set_difference(triPoints.begin(), triPoints.end(), removedPts.begin(), removedPts.end(), std::back_inserter(remainingPts));
  auto rpts = getPoints(remainingPts);
  for (const auto& pt : rpts) {
    voxels_.at(getVoxelIdx(pt, Eigen::Vector3d::Constant(voxelSize_))).activate();
  }
  std::cout << "Removed " << cropped->points_.size() << " points. (" << removeCtr << " points actually deleted)" << std::endl;

  mesh();
}

void MeshMap::mesh() {
  std::lock_guard<std::mutex> lck{meshLock_};
  std::mutex globalListMutex;
  std::vector<Triangle> globalTrisToAdd;
  std::unordered_set<size_t> globalTrisToDelete;
  std::vector<Eigen::Vector3i> updateIndices;
  for (auto& it : voxels_) {
    if (it.second.isUpdated()) {
      updateIndices.push_back(it.first);
    }
  }

  // #pragma omp parallel for default(none) shared(updateIndices, globalTrisToAdd, globalTrisToDelete, globalListMutex) num_threads(4)
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
  meshCount_++;
}

void MeshMap::addTriangle(const Triangle& tri) {
  size_t idx = nextTriIdx_++;
  triangles_.insert(std::make_pair(idx, tri));
  vertexToTriangles_[tri.i_].insert(idx);
  vertexToTriangles_[tri.j_].insert(idx);
  vertexToTriangles_[tri.k_].insert(idx);
}
void MeshMap::eraseTriangle(const size_t& triIdx) {
  if (triangles_.find(triIdx) == triangles_.end()) {
    return;
  }
  Triangle t = triangles_.at(triIdx);
  bool isDeleted = true;
  isDeleted &= vertexToTriangles_[t.i_].erase(triIdx);
  isDeleted &= vertexToTriangles_[t.j_].erase(triIdx);
  isDeleted &= vertexToTriangles_[t.k_].erase(triIdx);
  if (vertexToTriangles_.at(t.i_).empty()) {
    vertexToTriangles_.erase(t.i_);
  }
  if (vertexToTriangles_.at(t.j_).empty()) {
    vertexToTriangles_.erase(t.j_);
  }
  if (vertexToTriangles_.at(t.k_).empty()) {
    vertexToTriangles_.erase(t.k_);
  }
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

          if ((vertexSet.find(t.i_) != vertexSet.end()) && (vertexSet.find(t.j_) != vertexSet.end()) &&
              (vertexSet.find(t.k_) != vertexSet.end())) {
            pulledTriangles.push_back(t);
            pulledIdx.push_back(tri);
          }
        } catch (std::out_of_range& e) {
          std::cout << "Tried to access non-existing triangle " << tri << std::endl;
          std::cout << "Vertex " << vertex << " has triangles: [ ";
          for (auto t : vertexToTriangles_[vertex]) {
            std::cout << t << " ";
          }
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
  return dilateVertexSet(vertices);
}
std::vector<size_t> MeshMap::dilateVertexSet(const std::unordered_set<size_t>& vertices) {
  std::unordered_set<size_t> vertexSet = vertices;
  auto pts = getPoints(std::vector<size_t>(vertices.begin(), vertices.end()));
  Eigen::Vector3d vertexSetCentroid = std::accumulate(pts.begin(), pts.end(), Eigen::Vector3d::Zero().eval());
  vertexSetCentroid /= static_cast<double>(pts.size());
  auto distToLidar = (mapToRange_.translation() - vertexSetCentroid).norm();
  auto it = std::lower_bound(dilationDistances_.begin(), dilationDistances_.end(), distToLidar);
  size_t idx; // NOLINT(cppcoreguidelines-init-variables)
  if (it == dilationDistances_.end()) {
    idx = dilationDistances_.size() - 1;
  } else {
    idx = std::distance(dilationDistances_.begin(), it);
  }

  double dilationRadius = voxelSize_ * dilationRatios_[idx];
  {
    std::lock_guard<std::mutex> lck{vertexLock_};
    for (const auto& pt : pts) {
      std::vector<Eigen::Vector3d> ptSearch;
      ikdTree_->searchRadius(pt, dilationRadius, ptSearch);
      for (const auto& p : ptSearch) {
        auto ptIt = points_.right.find(p);
        if (ptIt != points_.right.end()) {
          auto ptIdx = ptIt->second;
          vertexSet.insert(ptIdx);
        }
      }
    }
  }
  return {vertexSet.begin(), vertexSet.end()};
}
std::vector<Triangle> MeshMap::triangulateVertexSetForVoxel(MeshVoxel& voxel, const std::vector<size_t>& vertices) const {
  if (vertices.size() < 3) {
    return {};  // We need at least 3 vertices for a triangle
  }
  voxel.initPlane();
  auto *ptr = voxel.getPlanePtr();
  Eigen::Vector3d q = ptr->getPlaneCenter();
  Eigen::Matrix<double, 3, 2> tangBase = ptr->getTangentialBase();
  auto meshVertices = getPoints(vertices);
  std::vector<Eigen::Vector2d> projectedVertices;
  projectedVertices.reserve(meshVertices.size());

  auto projectToPlane = [&](const Eigen::Vector3d& pt) -> Eigen::Vector2d { return ((pt - q).transpose() * tangBase).transpose(); };
  for (const auto& vert : meshVertices) {
    projectedVertices.push_back(projectToPlane(vert));
  }
  std::vector<Triangle> triangles;
  CDT::Triangulation<double> triangulator;
  auto dups = CDT::FindDuplicates<double>(
      projectedVertices.begin(), projectedVertices.end(), [](Eigen::Vector2d p) { return p.x() * 100; },
      [](Eigen::Vector2d p) { return p.y() * 100; });
  if (!dups.duplicates.empty()) {
    std::cout << "Found duplicate vertices in input, skipping triangulation step..." << std::endl;
    return {};
  }
  triangulator.insertVertices(
      projectedVertices.begin(), projectedVertices.end(), [](Eigen::Vector2d p) { return p.x() * 100; },
      [](Eigen::Vector2d p) { return p.y() * 100; });
  triangulator.eraseSuperTriangle();
  for (const auto tri : triangulator.triangles) {
    double sliverParameter = calculateSliverParameter(meshVertices, tri);
    if (sliverParameter > sliverThreshold_) {
      triangles.emplace_back(vertices[tri.vertices[0]], vertices[tri.vertices[1]], vertices[tri.vertices[2]]);
    }
  }
  return triangles;
}
double MeshMap::calculateSliverParameter(const std::vector<Eigen::Vector3d>& meshVertices, const CDT::Triangle& tri) {
  Eigen::Vector3d AB = meshVertices[tri.vertices[1]] - meshVertices[tri.vertices[0]];
  Eigen::Vector3d AC = meshVertices[tri.vertices[2]] - meshVertices[tri.vertices[0]];
  Eigen::Vector3d BC = meshVertices[tri.vertices[2]] - meshVertices[tri.vertices[1]];
  double area = (AB.cross(AC)).norm() / 2;
  double perim = AB.norm() + AC.norm() + BC.norm();
  double sliverParameter = (2 * area) / perim;
  return sliverParameter;
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
  std::unique_lock<std::mutex> triLock{triangleLock_, std::defer_lock};
  std::unique_lock<std::mutex> vertLock{vertexLock_, std::defer_lock};
  std::lock(triLock, vertLock);

  open3d::geometry::TriangleMesh mesh;
  std::vector<Eigen::Vector3i> triList;

  triList.reserve(triangles_.size());
  std::unordered_map<size_t, size_t> vertIdxToMeshIdx;
  mesh.vertices_.reserve(triangles_.size()*3);
#pragma omp for default(none) shared(triangles_, mesh, vertIdxToMeshIdx, triList) num_threads(4)
  for (const auto& tri : triangles_) {
    Triangle t = tri.second;
    auto it_i = vertIdxToMeshIdx.find(t.i_);
    if (it_i == vertIdxToMeshIdx.end()) {
      mesh.vertices_.push_back(points_.left.at(t.i_));
      it_i = vertIdxToMeshIdx.emplace(t.i_, mesh.vertices_.size() - 1).first;
    }
    auto it_j = vertIdxToMeshIdx.find(t.j_);
    if (it_j == vertIdxToMeshIdx.end()) {
      mesh.vertices_.push_back(points_.left.at(t.j_));
      it_j = vertIdxToMeshIdx.emplace(t.j_, mesh.vertices_.size() - 1).first;
    }

    auto it_k = vertIdxToMeshIdx.find(t.k_);
    if (it_k == vertIdxToMeshIdx.end()) {
      mesh.vertices_.push_back(points_.left.at(t.k_));
      it_k = vertIdxToMeshIdx.emplace(t.k_, mesh.vertices_.size() - 1).first;
    }
    triList.emplace_back(it_i->second, it_j->second, it_k->second);
  }

  mesh.triangles_ = triList;

  auto colors = std::vector<Eigen::Vector3d>(mesh.vertices_.size());
  mesh.vertex_colors_ = colors;
  mesh.vertex_normals_ = colors;
  return mesh;
}

void MeshMap::updateParameters(MeshingParameters params) {
  downsampleVoxelSize_ = params.downsamplingVoxelSize_;
  voxelSize_ = params.meshingVoxelSize_;
  newVertexThreshold_ = params.newVertexDistanceThreshold_;
  shouldFilter_ = params.shouldFilter_;
  filterEps_ = params.filterEps_;
  filterRadius_ = params.filterRadius_;
  voxelMaxUpdateCount_ = params.voxelMaxUpdates_;
  std::cout << "Downsample Size:\t\t" << downsampleVoxelSize_ << "m\n"
            << "New Vertex Threshold:\t\t" << newVertexThreshold_ << "m\n"
            << "Voxel Size:\t\t\t" << voxelSize_ << "m\n"
            << "Should Filter:\t\t\t" << shouldFilter_ << "\n"
            << "Filter Epsilon:\t\t\t" << filterEps_ << "\n"
            << "Filter Radius:\t\t\t" << filterRadius_ << "m\n"
            << "Maximum Voxel Updates:\t\t" << voxelMaxUpdateCount_ << "\n"
            << "Sliver Deletion Threshold:\t" << sliverThreshold_ << std::endl;
}

o3d_slam::PointCloud MeshMap::getVertices() {
  o3d_slam::PointCloud vertices;
  vertices.points_.reserve(points_.left.size());
  for (const auto& it : points_.left) {
    vertices.points_.push_back(it.second);
  }
  return vertices;
}
o3d_slam::PointCloud MeshMap::getMeshingInput() {
  std::lock_guard<std::mutex> lck{meshCloudLock_};
  if (mesherInput_ != nullptr) {
    return *mesherInput_;
  }
  return {};
}
