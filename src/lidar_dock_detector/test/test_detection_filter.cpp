#include "lidar_dock_detector/detection_filter.hpp"

#include <gtest/gtest.h>

namespace lidar_dock_detector
{
namespace
{

DockMeasurement measurement(const int sec, const double x = 1.0, const double yaw = 0.0)
{
  DockMeasurement result;
  result.marker_pose = {x, 0.0, yaw, true};
  result.dock_pose = {x + 0.1, 0.0, yaw, true};
  result.stamp.sec = sec;
  result.frame_id = "odom";
  result.confidence = 0.9;
  result.valid = true;
  return result;
}

TemporalConfig config()
{
  TemporalConfig result;
  result.required_consecutive_detections = 2;
  result.acquisition_timeout_sec = 0.5;
  result.stale_timeout_sec = 0.1;
  result.lost_timeout_sec = 0.3;
  result.scan_timeout_sec = 0.3;
  return result;
}

TEST(DetectionFilter, PublishesOnlyFreshAcceptedMeasurements)
{
  DetectionFilter filter(config(), std::nullopt);
  filter.setEnabled(true);
  const auto t0 = DetectionFilter::MonotonicTime {};
  EXPECT_FALSE(filter.update(measurement(1), t0).fresh_measurement);
  const auto acquired = filter.update(measurement(2), t0 + std::chrono::milliseconds(20));
  EXPECT_TRUE(acquired.fresh_measurement);
  EXPECT_TRUE(acquired.usable);
  EXPECT_EQ(acquired.last_measurement_stamp.sec, 2);
  const auto held = filter.update(std::nullopt, t0 + std::chrono::milliseconds(40));
  EXPECT_FALSE(held.fresh_measurement);
  EXPECT_TRUE(held.has_estimate);
  EXPECT_EQ(held.last_measurement_stamp.sec, 2);
}

TEST(DetectionFilter, RejectsOutOfOrderMeasurement)
{
  DetectionFilter filter(config(), std::nullopt);
  filter.setEnabled(true);
  const auto t0 = DetectionFilter::MonotonicTime {};
  filter.update(measurement(10), t0);
  ASSERT_TRUE(filter.update(measurement(11), t0 + std::chrono::milliseconds(10)).fresh_measurement);
  const auto result = filter.update(measurement(10), t0 + std::chrono::milliseconds(20));
  EXPECT_FALSE(result.fresh_measurement);
  EXPECT_EQ(result.last_measurement_stamp.sec, 11);
}

TEST(DetectionFilter, WatchdogLosesTrackWithoutScanCallbacks)
{
  DetectionFilter filter(config(), std::nullopt);
  filter.setEnabled(true);
  const auto t0 = DetectionFilter::MonotonicTime {};
  filter.update(measurement(1), t0);
  filter.update(measurement(2), t0 + std::chrono::milliseconds(10));
  const auto result = filter.watchdog(t0 + std::chrono::milliseconds(400));
  EXPECT_EQ(result.state, DetectionState::kLost);
  EXPECT_FALSE(result.has_estimate);
}

TEST(DetectionFilter, WatchdogDoesNotLoseBeforeInitialScanTimeout)
{
  DetectionFilter filter(config(), std::nullopt);
  filter.setEnabled(true);
  const auto result = filter.watchdog(DetectionFilter::MonotonicTime::clock::now());
  EXPECT_EQ(result.state, DetectionState::kSearching);
}

TEST(DetectionFilter, DropoutResetsUnconfirmedAcquisition)
{
  DetectionFilter filter(config(), std::nullopt);
  filter.setEnabled(true);
  const auto t0 = DetectionFilter::MonotonicTime {};
  filter.update(measurement(1), t0);
  filter.update(std::nullopt, t0 + std::chrono::milliseconds(10));
  EXPECT_FALSE(filter.update(measurement(2), t0 + std::chrono::milliseconds(20)).fresh_measurement);
}

}  // namespace
}  // namespace lidar_dock_detector
