#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/model.hpp"
#include "lidar_dock_detector/types.hpp"

#include <Eigen/Core>

#include <optional>
#include <utility>
#include <vector>

namespace lidar_dock_detector
{

class PoseEstimator
{
public:
  explicit PoseEstimator(const DetectorConfig & config);

  CandidateDebug estimate(
    const MatchCandidate & candidate,
    const std::optional<Pose2D> & prior_dock_pose,
    bool tracking_active) const;

private:
  struct IcpResult
  {
    Pose2D pose;
    double alignment_error {0.0};
    double correspondence_ratio {0.0};
    double model_coverage_ratio {0.0};
  };

  IcpResult runIcp(
    const Pose2D & initial_marker_pose,
    const std::vector<Point2D> & observed_points) const;
  std::vector<Eigen::Vector2d> transformModel(const Pose2D & marker_pose) const;
  std::pair<double, double> alignmentStats(
    const std::vector<Eigen::Vector2d> & transformed_model,
    const std::vector<Point2D> & observed_points) const;
  double modelCoverageRatio(
    const std::vector<Eigen::Vector2d> & transformed_model,
    const std::vector<Point2D> & observed_points) const;

  DetectorConfig config_;
  LandmarkModel model_;
};

}  // namespace lidar_dock_detector
