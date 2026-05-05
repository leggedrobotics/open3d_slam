#include <gtest/gtest.h>

#include <memory>

#include "open3d_slam/magic.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {
namespace {

class CapturingProcessor : public DataProcessorRos {
 public:
  explicit CapturingProcessor(const rclcpp::Node::SharedPtr& node) : DataProcessorRos(node) {}

  void initialize() override {}
  void startProcessing() override {}

  void processMeasurement(const PointCloud& cloud, const Time& timestamp, const Transform& odomToRangeSensor) override {
    processedCloud = cloud;
    processedTimestamp = timestamp;
    processedOdomToRangeSensor = odomToRangeSensor;
    ++numProcessed;
  }

  PointCloud processedCloud;
  Time processedTimestamp;
  Transform processedOdomToRangeSensor = Transform::Identity();
  size_t numProcessed = 0;
};

PointCloud makeSinglePointCloud(const double x, const double y, const double z) {
  PointCloud cloud;
  cloud.points_.emplace_back(x, y, z);
  return cloud;
}

Transform makeXTranslation(const double x) {
  Transform transform = Transform::Identity();
  transform.translation().x() = x;
  return transform;
}

std::shared_ptr<rclcpp::Node> makeNode(const bool endpointFrameAccumulation) {
  auto node = std::make_shared<rclcpp::Node>("endpoint_frame_accumulation_test");
  node->declare_parameter("cloud_topic", "/points");
  node->declare_parameter("num_accumulated_range_data", 2);
  node->declare_parameter("transform_accumulated_range_data_to_endpoint_frame", endpointFrameAccumulation);
  return node;
}

void skipStartupClouds(CapturingProcessor& processor) {
  const PointCloud cloud = makeSinglePointCloud(0.0, 0.0, 0.0);
  for (size_t i = 0; i < magic::skipFirstNPointClouds; ++i) {
    processor.accumulateAndProcessRangeData(cloud, fromUniversal(static_cast<int64>(i + 1)), Transform::Identity());
  }
}

}  // namespace

TEST(EndpointFrameAccumulation, TransformsExternalOdomCloudsIntoLastSensorFrame) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  CapturingProcessor processor(makeNode(true));
  processor.initCommonRosStuff();
  skipStartupClouds(processor);

  processor.accumulateAndProcessRangeData(makeSinglePointCloud(0.0, 0.0, 0.0), fromUniversal(10), makeXTranslation(0.0));
  processor.accumulateAndProcessRangeData(makeSinglePointCloud(0.0, 0.0, 0.0), fromUniversal(20), makeXTranslation(1.0));

  ASSERT_EQ(processor.numProcessed, 1u);
  ASSERT_EQ(processor.processedCloud.points_.size(), 2u);
  EXPECT_NEAR(processor.processedCloud.points_.at(0).x(), -1.0, 1e-9);
  EXPECT_NEAR(processor.processedCloud.points_.at(1).x(), 0.0, 1e-9);
  EXPECT_NEAR(processor.processedOdomToRangeSensor.translation().x(), 1.0, 1e-9);
  EXPECT_EQ(toUniversal(processor.processedTimestamp), 20);
}

TEST(EndpointFrameAccumulation, KeepsRawAccumulationWhenDisabled) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  CapturingProcessor processor(makeNode(false));
  processor.initCommonRosStuff();
  skipStartupClouds(processor);

  processor.accumulateAndProcessRangeData(makeSinglePointCloud(0.0, 0.0, 0.0), fromUniversal(10), makeXTranslation(0.0));
  processor.accumulateAndProcessRangeData(makeSinglePointCloud(0.0, 0.0, 0.0), fromUniversal(20), makeXTranslation(1.0));

  ASSERT_EQ(processor.numProcessed, 1u);
  ASSERT_EQ(processor.processedCloud.points_.size(), 2u);
  EXPECT_NEAR(processor.processedCloud.points_.at(0).x(), 0.0, 1e-9);
  EXPECT_NEAR(processor.processedCloud.points_.at(1).x(), 0.0, 1e-9);
  EXPECT_NEAR(processor.processedOdomToRangeSensor.translation().x(), 1.0, 1e-9);
}

}  // namespace o3d_slam
