#include <gtest/gtest.h>

#include <open3d/Open3D.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "open3d_slam/output.hpp"

namespace {

class ScopedDirectory {
 public:
  ScopedDirectory() {
    char directory_template[] = "/tmp/open3d_slam_output_XXXXXX";
    const char* created = mkdtemp(directory_template);
    if (created == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = created;
  }

  ~ScopedDirectory() { std::filesystem::remove_all(path_); }

  [[nodiscard]] const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

o3d_slam::PointCloud makeUncoloredCloud() {
  o3d_slam::PointCloud cloud;
  cloud.points_.emplace_back(0.0, 0.0, 0.0);
  cloud.points_.emplace_back(1.0, -0.5, 0.25);
  cloud.points_.emplace_back(2.0, 0.75, -0.1);
  return cloud;
}

o3d_slam::PointCloud makeColoredCloud() {
  auto cloud = makeUncoloredCloud();
  cloud.colors_.emplace_back(1.0, 0.0, 0.0);
  cloud.colors_.emplace_back(0.0, 1.0, 0.0);
  cloud.colors_.emplace_back(0.0, 0.0, 1.0);
  return cloud;
}

}  // namespace

TEST(Output, UncoloredExportDefaultsToPcd) {
  ScopedDirectory temp_dir;
  const auto base_path = temp_dir.path() / "map";
  std::ofstream(base_path.string() + ".ply") << "stale";

  ASSERT_TRUE(o3d_slam::saveToFile(base_path.string(), makeUncoloredCloud()));
  EXPECT_TRUE(std::filesystem::exists(base_path.string() + ".pcd"));
  EXPECT_FALSE(std::filesystem::exists(base_path.string() + ".ply"));
}

TEST(Output, ExplicitExtensionIsHonoredForUncoloredClouds) {
  ScopedDirectory temp_dir;
  const auto file_path = temp_dir.path() / "map.ply";

  ASSERT_TRUE(o3d_slam::saveToFile(file_path.string(), makeUncoloredCloud()));
  EXPECT_TRUE(std::filesystem::exists(file_path));
  EXPECT_FALSE(std::filesystem::exists(file_path.string() + ".pcd"));
}

TEST(Output, ColoredExportDefaultsToPlyAndPreservesVariation) {
  ScopedDirectory temp_dir;
  const auto base_path = temp_dir.path() / "map";
  std::ofstream(base_path.string() + ".pcd") << "stale";

  ASSERT_TRUE(o3d_slam::saveToFile(base_path.string(), makeColoredCloud()));
  const auto ply_path = base_path.string() + ".ply";
  EXPECT_TRUE(std::filesystem::exists(ply_path));
  EXPECT_FALSE(std::filesystem::exists(base_path.string() + ".pcd"));

  open3d::geometry::PointCloud saved_cloud;
  ASSERT_TRUE(open3d::io::ReadPointCloud(ply_path, saved_cloud));
  ASSERT_TRUE(saved_cloud.HasColors());
  ASSERT_EQ(saved_cloud.points_.size(), saved_cloud.colors_.size());
  ASSERT_GE(saved_cloud.colors_.size(), 3U);
  EXPECT_NEAR(saved_cloud.colors_.front().x(), 1.0, 1e-6);
  EXPECT_NEAR(saved_cloud.colors_.front().y(), 0.0, 1e-6);
  EXPECT_NEAR(saved_cloud.colors_.front().z(), 0.0, 1e-6);
  EXPECT_NE(saved_cloud.colors_.front(), saved_cloud.colors_.back());
}
