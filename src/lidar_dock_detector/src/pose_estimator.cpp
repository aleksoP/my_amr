#include "lidar_dock_detector/pose_estimator.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

RigidTransform2D composeTransforms(const RigidTransform2D & delta, const RigidTransform2D & base)
{
  RigidTransform2D composed;
  composed.rotation = delta.rotation * base.rotation;
  composed.translation = delta.rotation * base.translation + delta.translation;
  return composed;
}

}  // namespace

PoseEstimator::PoseEstimator(const DetectorConfig & config)
: config_(config),
  model_(buildLandmarkModel(config.landmark))
{
}

CandidateDebug PoseEstimator::estimate(
  const MatchCandidate & candidate,
  const std::optional<Pose2D> & prior_dock_pose,
  const bool tracking_active) const
{
  CandidateDebug debug;
  debug.lines = candidate.lines;
  debug.points = candidate.points;
  debug.score = candidate.geometric_score;

  const IcpResult icp = runIcp(candidate.initial_marker_pose, candidate.points);
  debug.marker_pose = icp.pose;
  debug.dock_pose = composePose(debug.marker_pose, config_.landmark.marker_to_dock_offset);
  debug.alignment_error = icp.alignment_error;
  debug.correspondence_ratio = icp.correspondence_ratio;
  debug.model_coverage_ratio = icp.model_coverage_ratio;

  if (!debug.marker_pose.valid || !debug.dock_pose.valid || !std::isfinite(icp.alignment_error)) {
    debug.reason = "icp_failed";
    return debug;
  }

  if (icp.alignment_error > config_.matching.max_alignment_error) {
    debug.reason = "alignment_error";
    return debug;
  }

  if (icp.correspondence_ratio < config_.matching.min_icp_correspondence_ratio) {
    debug.reason = "low_correspondence";
    debug.confidence = icp.correspondence_ratio;
    return debug;
  }

  if (icp.model_coverage_ratio < config_.matching.min_model_coverage_ratio) {
    debug.reason = "low_model_coverage";
    debug.confidence = icp.model_coverage_ratio;
    return debug;
  }

  if (prior_dock_pose.has_value()) {
    const double allowed_distance = tracking_active ?
      config_.matching.tracking_search_radius : config_.matching.initial_search_radius;
    const double allowed_yaw = tracking_active ?
      config_.matching.tracking_yaw_tolerance_rad : config_.matching.initial_yaw_tolerance_rad;

    if (distance(debug.dock_pose, *prior_dock_pose) > allowed_distance) {
      debug.reason = "search_gate_distance";
      return debug;
    }
    if (std::abs(shortestAngularDistance(prior_dock_pose->yaw, debug.dock_pose.yaw)) > allowed_yaw) {
      debug.reason = "search_gate_yaw";
      return debug;
    }
  }

  const double alignment_confidence = clamp01(1.0 - icp.alignment_error / config_.matching.max_alignment_error);
  debug.score = 0.5 * candidate.geometric_score + 0.5 * alignment_confidence;
  debug.confidence = clamp01(
    0.40 * candidate.geometric_confidence +
    0.45 * alignment_confidence +
    0.10 * icp.correspondence_ratio +
    0.05 * icp.model_coverage_ratio);

  if (debug.confidence < config_.matching.min_confidence) {
    debug.reason = "low_confidence";
    return debug;
  }

  debug.accepted = true;
  debug.reason = "accepted";
  return debug;
}

PoseEstimator::IcpResult PoseEstimator::runIcp(
  const Pose2D & initial_marker_pose,
  const std::vector<Point2D> & observed_points) const
{
  if (observed_points.size() < 2U) {
    return {Pose2D {}, std::numeric_limits<double>::infinity(), 0.0, 0.0};
  }

  Pose2D current_pose = initial_marker_pose;
  double current_error = std::numeric_limits<double>::infinity();
  double current_correspondence_ratio = 0.0;

  for (int iteration = 0; iteration < config_.matching.icp_max_iterations; ++iteration) {
    const std::vector<Eigen::Vector2d> transformed_model = transformModel(current_pose);

    std::vector<Eigen::Vector2d> source_points;
    std::vector<Eigen::Vector2d> target_points;
    source_points.reserve(observed_points.size());
    target_points.reserve(observed_points.size());

    for (const auto & observed_point : observed_points) {
      const Eigen::Vector2d observed = toEigen(observed_point);
      double best_distance = config_.matching.icp_max_correspondence_distance;
      const Eigen::Vector2d * best_point = nullptr;
      for (const auto & model_point : transformed_model) {
        const double distance_to_model = (observed - model_point).norm();
        if (distance_to_model < best_distance) {
          best_distance = distance_to_model;
          best_point = &model_point;
        }
      }

      if (best_point != nullptr) {
        source_points.push_back(*best_point);
        target_points.push_back(observed);
      }
    }

    if (source_points.size() < 2U) {
      return {Pose2D {}, std::numeric_limits<double>::infinity(), 0.0, 0.0};
    }

    const auto delta = estimateRigidTransform(source_points, target_points);
    if (!delta.has_value()) {
      return {Pose2D {}, std::numeric_limits<double>::infinity(), 0.0, 0.0};
    }

    const RigidTransform2D updated_transform = composeTransforms(*delta, poseToTransform(current_pose));
    current_pose = transformToPose(updated_transform);
    const auto [alignment_error, correspondence_ratio] =
      alignmentStats(transformModel(current_pose), observed_points);
    current_error = alignment_error;
    current_correspondence_ratio = correspondence_ratio;

    const double translation_step = delta->translation.norm();
    const double rotation_step = std::abs(std::atan2(delta->rotation(1, 0), delta->rotation(0, 0)));
    if (
      translation_step < config_.matching.icp_translation_tolerance &&
      rotation_step < config_.matching.icp_rotation_tolerance_rad)
    {
      break;
    }
  }

  const double current_model_coverage_ratio =
    modelCoverageRatio(transformModel(current_pose), observed_points);
  return {current_pose, current_error, current_correspondence_ratio, current_model_coverage_ratio};
}

std::vector<Eigen::Vector2d> PoseEstimator::transformModel(const Pose2D & marker_pose) const
{
  const RigidTransform2D transform = poseToTransform(marker_pose);
  std::vector<Eigen::Vector2d> transformed;
  transformed.reserve(model_.samples.size());
  for (const auto & point : model_.samples) {
    transformed.push_back(applyTransform(transform, point));
  }
  return transformed;
}

std::pair<double, double> PoseEstimator::alignmentStats(
  const std::vector<Eigen::Vector2d> & transformed_model,
  const std::vector<Point2D> & observed_points) const
{
  double squared_error = 0.0;
  int matches = 0;

  for (const auto & observed_point : observed_points) {
    const Eigen::Vector2d observed = toEigen(observed_point);
    double best_distance = config_.matching.icp_max_correspondence_distance;
    bool found_match = false;
    for (const auto & model_point : transformed_model) {
      const double distance_to_model = (observed - model_point).norm();
      if (distance_to_model < best_distance) {
        best_distance = distance_to_model;
        found_match = true;
      }
    }

    if (found_match) {
      squared_error += best_distance * best_distance;
      ++matches;
    }
  }

  if (matches < 2) {
    return {std::numeric_limits<double>::infinity(), 0.0};
  }
  return {
    std::sqrt(squared_error / static_cast<double>(matches)),
    static_cast<double>(matches) / static_cast<double>(observed_points.size())};
}

double PoseEstimator::modelCoverageRatio(
  const std::vector<Eigen::Vector2d> & transformed_model,
  const std::vector<Point2D> & observed_points) const
{
  if (transformed_model.empty() || observed_points.empty()) {
    return 0.0;
  }

  int covered_samples = 0;
  for (const auto & model_point : transformed_model) {
    bool covered = false;
    for (const auto & observed_point : observed_points) {
      if ((toEigen(observed_point) - model_point).norm() <= config_.matching.icp_max_correspondence_distance) {
        covered = true;
        break;
      }
    }
    if (covered) {
      ++covered_samples;
    }
  }

  return static_cast<double>(covered_samples) / static_cast<double>(transformed_model.size());
}

}  // namespace lidar_dock_detector
