#pragma once

#include "lidar_dock_detector/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace lidar_dock_detector
{

struct TopicConfig
{
  std::string scan_topic {"scan"};
  std::string pose_topic {"/detected_dock_pose"};
  std::string pose_frame {""};
  std::string confidence_topic {"/dock_detector/confidence"};
  std::string state_topic {"/dock_detector/state"};
  std::string marker_topic {"/dock_detector/debug_markers"};
  std::string output_frame {"base_link"};
  std::string marker_frame {"dock_marker"};
};

struct ServiceConfig
{
  std::string start_service {"/dock_detector/start"};
  std::string stop_service {"/dock_detector/stop"};
  std::string toggle_service {"/dock_detector/toggle_detection"};
};

struct RoiConfig
{
  double min_x {-0.50};
  double max_x {2.00};
  double min_y {-1.00};
  double max_y {1.00};
  double min_range {0.05};
  double max_range {2.50};
};

struct SplitConfig
{
  double point_gap {0.06};
  int min_region_points {8};
  double min_region_span {0.10};
  double max_region_span {1.20};
};

struct LineExtractionConfig
{
  double split_threshold {0.025};
  int min_line_points {5};
  double min_line_length {0.05};
  double max_fit_error {0.02};
  double merge_angle_threshold_rad {0.12};
  double merge_gap_threshold {0.03};
  int smoothing_window {0};
  double corner_angle_threshold_rad {0.0};
};

struct LandmarkConfig
{
  double total_length {0.0};
  std::vector<double> segment_lengths;
  std::vector<double> turn_angles_rad;
  std::vector<double> vertices_x;
  std::vector<double> vertices_y;
  double length_tolerance {0.04};
  double angle_tolerance_rad {0.26};
  int min_visible_segments {3};
  double sample_spacing {0.01};
  std::string origin_mode {"start"};
  int origin_vertex {0};
  double forward_axis_rad {0.0};
  Pose2D marker_to_dock_offset {0.0, 0.0, 0.0, true};
};

struct MatchingConfig
{
  double initial_search_radius {0.80};
  double initial_yaw_tolerance_rad {1.05};
  double tracking_search_radius {0.30};
  double tracking_yaw_tolerance_rad {0.52};
  double icp_max_correspondence_distance {0.08};
  int icp_max_iterations {20};
  int max_candidates_per_region {8};
  double icp_translation_tolerance {0.002};
  double icp_rotation_tolerance_rad {0.01};
  double max_alignment_error {0.03};
  double min_icp_correspondence_ratio {0.70};
  double min_model_coverage_ratio {0.55};
  double min_geometric_score {0.55};
  double min_confidence {0.60};
};

struct TemporalConfig
{
  int required_consecutive_detections {3};
  int max_missed_frames {3};
  double max_position_jump {0.10};
  double max_yaw_jump_rad {0.35};
  double smoothing_alpha_position {0.35};
  double smoothing_alpha_yaw {0.25};
  double pending_reacquire_distance {0.20};
  double pending_reacquire_yaw_rad {0.35};
};

struct InitialEstimateConfig
{
  bool use_initial_estimate {true};
  Pose2D pose {1.00, 0.0, 0.0, true};
};

struct DetectorConfig
{
  TopicConfig topics;
  ServiceConfig services;
  RoiConfig roi;
  SplitConfig splitting;
  LineExtractionConfig line_extraction;
  LandmarkConfig landmark;
  MatchingConfig matching;
  TemporalConfig temporal;
  InitialEstimateConfig initial_estimate;
  bool enabled_on_startup {false};
  bool publish_debug_markers {true};
  bool log_pipeline_debug {false};
};

DetectorConfig loadDetectorConfig(rclcpp::Node & node);

}  // namespace lidar_dock_detector
