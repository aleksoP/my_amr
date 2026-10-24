#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/types.hpp"

#include <optional>

namespace lidar_dock_detector
{

class DetectionFilter
{
public:
  DetectionFilter(TemporalConfig config, std::optional<Pose2D> initial_search_pose);

  void setEnabled(bool enabled);
  void reset();
  FilterOutput update(const std::optional<CandidateDebug> & candidate);
  std::optional<Pose2D> predictedDockPose() const;
  bool hasTrackedPose() const;
  DetectionState state() const;

private:
  bool isConsistentWithPending(const CandidateDebug & candidate) const;
  void clearTracking();

  TemporalConfig config_;
  std::optional<Pose2D> initial_search_pose_;
  bool enabled_ {false};
  DetectionState state_ {DetectionState::kIdle};
  int consecutive_hits_ {0};
  int missed_frames_ {0};
  std::optional<CandidateDebug> pending_candidate_;
  std::optional<Pose2D> marker_pose_;
  std::optional<Pose2D> dock_pose_;
  double confidence_ {0.0};
};

}  // namespace lidar_dock_detector
