#include "lidar_dock_detector/config.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace lidar_dock_detector
{
namespace
{

class ConfigValidationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<rclcpp::Node> makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp::Node>("config_validation_test", options);
  }
};

TEST_F(ConfigValidationTest, RejectsInvalidRoi)
{
  auto node = makeNode({rclcpp::Parameter("roi.min_x", 2.0), rclcpp::Parameter("roi.max_x", 1.0)});
  EXPECT_THROW(loadDetectorConfig(*node), std::runtime_error);
}

TEST_F(ConfigValidationTest, RejectsZeroSampleSpacing)
{
  auto node = makeNode({rclcpp::Parameter("landmark.sample_spacing", 0.0)});
  EXPECT_THROW(loadDetectorConfig(*node), std::runtime_error);
}

TEST_F(ConfigValidationTest, RejectsInvalidConfidenceAndTimeout)
{
  auto confidence = makeNode({rclcpp::Parameter("matching.min_confidence", 1.1)});
  EXPECT_THROW(loadDetectorConfig(*confidence), std::runtime_error);
  auto timeout = makeNode({rclcpp::Parameter("temporal.lost_timeout_sec", 0.0)});
  EXPECT_THROW(loadDetectorConfig(*timeout), std::runtime_error);
}

}  // namespace
}  // namespace lidar_dock_detector
