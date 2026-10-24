#pragma once

#include <Eigen/Core>

#include <builtin_interfaces/msg/time.hpp>

#include <optional>
#include <string>
#include <vector>

namespace lidar_dock_detector
{

struct Point2D
{
  double x {0.0};
  double y {0.0};
  int scan_index {0};
};

struct Pose2D
{
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
  bool valid {false};
};

struct Region
{
  std::vector<Point2D> points;
  double span {0.0};
};

struct LineSegment
{
  Eigen::Vector2d start {Eigen::Vector2d::Zero()};
  Eigen::Vector2d end {Eigen::Vector2d::Zero()};
  Eigen::Vector2d centroid {Eigen::Vector2d::Zero()};
  Eigen::Vector2d direction {Eigen::Vector2d::UnitX()};
  double length {0.0};
  double rms_error {0.0};
  int start_index {0};
  int end_index {0};
};

struct RejectedLineSegmentDebug
{
  LineSegment line;
  std::string reason;
  double max_error {0.0};
  int point_count {0};
};

struct MatchCandidate
{
  std::vector<LineSegment> lines;
  std::vector<Point2D> points;
  Pose2D initial_marker_pose;
  double geometric_score {0.0};
  double geometric_confidence {0.0};
};

struct LandmarkMatchDebug
{
  int windows {0};
  int transform_rejects {0};
  int length_rejects {0};
  int angle_rejects {0};
  int vertex_rejects {0};
  int geometric_rejects {0};
  double best_length_score {0.0};
  double best_angle_score {0.0};
  double best_vertex_score {0.0};
  double best_geometric_score {0.0};
};

struct CandidateDebug
{
  Pose2D marker_pose;
  Pose2D dock_pose;
  double score {0.0};
  double confidence {0.0};
  double alignment_error {0.0};
  double correspondence_ratio {0.0};
  double model_coverage_ratio {0.0};
  bool accepted {false};
  std::string reason;
  std::vector<Point2D> points;
  std::vector<LineSegment> lines;
};

// Immutable result of one successful geometric scan evaluation.  It deliberately
// represents an observation, not a filtered or held track state.
struct DockMeasurement
{
  Pose2D marker_pose;
  Pose2D dock_pose;
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
  double score {0.0};
  double confidence {0.0};
  double alignment_error {0.0};
  double correspondence_ratio {0.0};
  double model_coverage_ratio {0.0};
  bool valid {false};
};

enum class DetectionState
{
  kIdle,
  kSearching,
  kCandidate,
  kTracking,
  kLost
};

struct FilterOutput
{
  DetectionState state {DetectionState::kIdle};
  double confidence {0.0};
  bool has_estimate {false};
  bool fresh_measurement {false};
  bool usable {false};
  bool stale {false};
  Pose2D marker_pose;
  Pose2D dock_pose;
  builtin_interfaces::msg::Time last_measurement_stamp;
};

struct DetectionDebugFrame
{
  std::vector<Point2D> roi_points;
  std::vector<Region> regions;
  std::vector<LineSegment> fitted_lines;
  std::vector<Point2D> split_points;
  std::vector<Point2D> residual_peaks;
  std::vector<RejectedLineSegmentDebug> rejected_line_segments;
  std::vector<CandidateDebug> rejected_candidates;
  std::optional<CandidateDebug> accepted_candidate;
};

}  // namespace lidar_dock_detector
