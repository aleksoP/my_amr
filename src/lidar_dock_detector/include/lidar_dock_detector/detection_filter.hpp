#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/types.hpp"

#include <optional>
#include <chrono>

namespace lidar_dock_detector
{

class DetectionFilter
{
public:
  using MonotonicTime = std::chrono::steady_clock::time_point;

  DetectionFilter(TemporalConfig config, std::optional<Pose2D> initial_search_pose);

  void setEnabled(bool enabled);
  void reset();
  FilterOutput update(const std::optional<DockMeasurement> & measurement, MonotonicTime now);
  FilterOutput watchdog(MonotonicTime now);
  std::optional<Pose2D> predictedDockPose() const;
  bool hasTrackedPose() const;
  DetectionState state() const;

private:
  bool isConsistentWithPending(const DockMeasurement & measurement) const;
  FilterOutput output(bool fresh_measurement, MonotonicTime now) const;
  void clearTracking();

  TemporalConfig config_;
  std::optional<Pose2D> initial_search_pose_;
  bool enabled_ {false};
  DetectionState state_ {DetectionState::kIdle};
  int consecutive_hits_ {0};
  std::optional<DockMeasurement> pending_measurement_;
  std::optional<Pose2D> marker_pose_;
  std::optional<Pose2D> dock_pose_;
  double confidence_ {0.0};
  std::optional<builtin_interfaces::msg::Time> last_measurement_stamp_;
  std::optional<MonotonicTime> last_measurement_time_;
  std::optional<MonotonicTime> last_scan_time_;
  std::optional<MonotonicTime> enabled_time_;
  std::optional<MonotonicTime> acquisition_start_time_;
};

}  // namespace lidar_dock_detector
