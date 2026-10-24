#include "lidar_dock_detector/config.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace lidar_dock_detector
{

namespace
{

template<typename T>
T declareParam(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  return node.declare_parameter<T>(name, default_value);
}

}  // namespace

DetectorConfig loadDetectorConfig(rclcpp::Node & node)
{
  DetectorConfig config;

  config.topics.scan_topic = declareParam(node, "scan_topic", config.topics.scan_topic);
  config.topics.pose_topic = declareParam(node, "pose_topic", config.topics.pose_topic);
  config.topics.pose_frame = declareParam(node, "pose_frame", config.topics.pose_frame);
  config.topics.confidence_topic = declareParam(node, "confidence_topic", config.topics.confidence_topic);
  config.topics.state_topic = declareParam(node, "state_topic", config.topics.state_topic);
  config.topics.marker_topic = declareParam(node, "marker_topic", config.topics.marker_topic);
  config.topics.output_frame = declareParam(node, "output_frame", config.topics.output_frame);
  config.topics.marker_frame = declareParam(node, "marker_frame", config.topics.marker_frame);

  config.services.start_service = declareParam(node, "services.start", config.services.start_service);
  config.services.stop_service = declareParam(node, "services.stop", config.services.stop_service);
  config.services.toggle_service = declareParam(node, "services.toggle", config.services.toggle_service);

  config.enabled_on_startup = declareParam(node, "enabled_on_startup", config.enabled_on_startup);
  config.publish_debug_markers = declareParam(node, "publish_debug_markers", config.publish_debug_markers);
  config.log_pipeline_debug = declareParam(node, "log_pipeline_debug", config.log_pipeline_debug);

  config.roi.min_x = declareParam(node, "roi.min_x", config.roi.min_x);
  config.roi.max_x = declareParam(node, "roi.max_x", config.roi.max_x);
  config.roi.min_y = declareParam(node, "roi.min_y", config.roi.min_y);
  config.roi.max_y = declareParam(node, "roi.max_y", config.roi.max_y);
  config.roi.min_range = declareParam(node, "roi.min_range", config.roi.min_range);
  config.roi.max_range = declareParam(node, "roi.max_range", config.roi.max_range);

  config.splitting.point_gap = declareParam(node, "splitting.point_gap", config.splitting.point_gap);
  config.splitting.min_region_points = declareParam(
    node, "splitting.min_region_points", config.splitting.min_region_points);
  config.splitting.min_region_span = declareParam(
    node, "splitting.min_region_span", config.splitting.min_region_span);
  config.splitting.max_region_span = declareParam(
    node, "splitting.max_region_span", config.splitting.max_region_span);

  config.line_extraction.split_threshold = declareParam(
    node, "line_extraction.split_threshold", config.line_extraction.split_threshold);
  config.line_extraction.min_line_points = declareParam(
    node, "line_extraction.min_line_points", config.line_extraction.min_line_points);
  config.line_extraction.min_line_length = declareParam(
    node, "line_extraction.min_line_length", config.line_extraction.min_line_length);
  config.line_extraction.max_fit_error = declareParam(
    node, "line_extraction.max_fit_error", config.line_extraction.max_fit_error);
  config.line_extraction.merge_angle_threshold_rad = degreesToRadians(declareParam(
      node, "line_extraction.merge_angle_threshold_deg",
      radiansToDegrees(config.line_extraction.merge_angle_threshold_rad)));
  config.line_extraction.merge_gap_threshold = declareParam(
    node, "line_extraction.merge_gap_threshold", config.line_extraction.merge_gap_threshold);
  config.line_extraction.smoothing_window = declareParam(
    node, "line_extraction.smoothing_window", config.line_extraction.smoothing_window);
  config.line_extraction.corner_angle_threshold_rad = degreesToRadians(declareParam(
      node, "line_extraction.corner_angle_threshold_deg",
      radiansToDegrees(config.line_extraction.corner_angle_threshold_rad)));

  config.landmark.segment_lengths = declareParam<std::vector<double>>(
    node, "landmark.segment_lengths", {0.14, 0.10, 0.14, 0.10});
  config.landmark.turn_angles_rad.clear();
  const auto turn_angles_deg = declareParam<std::vector<double>>(
    node, "landmark.turn_angles_deg", {45.0, -90.0, 45.0});
  config.landmark.turn_angles_rad.reserve(turn_angles_deg.size());
  for (const double angle_deg : turn_angles_deg) {
    config.landmark.turn_angles_rad.push_back(degreesToRadians(angle_deg));
  }
  config.landmark.total_length = declareParam(
    node,
    "landmark.total_length",
    std::accumulate(
      config.landmark.segment_lengths.begin(),
      config.landmark.segment_lengths.end(),
      0.0));
  config.landmark.vertices_x = declareParam<std::vector<double>>(node, "landmark.vertices_x", {});
  config.landmark.vertices_y = declareParam<std::vector<double>>(node, "landmark.vertices_y", {});
  config.landmark.length_tolerance = declareParam(
    node, "landmark.length_tolerance", config.landmark.length_tolerance);
  config.landmark.angle_tolerance_rad = degreesToRadians(declareParam(
      node, "landmark.angle_tolerance_deg",
      radiansToDegrees(config.landmark.angle_tolerance_rad)));
  config.landmark.min_visible_segments = declareParam(
    node, "landmark.min_visible_segments", config.landmark.min_visible_segments);
  config.landmark.sample_spacing = declareParam(
    node, "landmark.sample_spacing", config.landmark.sample_spacing);
  config.landmark.origin_mode = declareParam(node, "landmark.origin_mode", config.landmark.origin_mode);
  config.landmark.origin_vertex = declareParam(node, "landmark.origin_vertex", config.landmark.origin_vertex);
  config.landmark.forward_axis_rad = degreesToRadians(declareParam(
      node, "landmark.forward_axis_deg", radiansToDegrees(config.landmark.forward_axis_rad)));
  config.landmark.marker_to_dock_offset.x = declareParam(
    node, "landmark.marker_to_dock_offset.x", -0.18);
  config.landmark.marker_to_dock_offset.y = declareParam(
    node, "landmark.marker_to_dock_offset.y", 0.0);
  config.landmark.marker_to_dock_offset.yaw = degreesToRadians(declareParam(
      node, "landmark.marker_to_dock_offset.yaw_deg", 0.0));
  config.landmark.marker_to_dock_offset.valid = true;

  config.matching.initial_search_radius = declareParam(
    node, "matching.initial_search_radius", config.matching.initial_search_radius);
  config.matching.initial_yaw_tolerance_rad = degreesToRadians(declareParam(
      node, "matching.initial_yaw_tolerance_deg",
      radiansToDegrees(config.matching.initial_yaw_tolerance_rad)));
  config.matching.tracking_search_radius = declareParam(
    node, "matching.tracking_search_radius", config.matching.tracking_search_radius);
  config.matching.tracking_yaw_tolerance_rad = degreesToRadians(declareParam(
      node, "matching.tracking_yaw_tolerance_deg",
      radiansToDegrees(config.matching.tracking_yaw_tolerance_rad)));
  config.matching.icp_max_correspondence_distance = declareParam(
    node, "matching.icp_max_correspondence_distance", config.matching.icp_max_correspondence_distance);
  config.matching.icp_max_iterations = declareParam(
    node, "matching.icp_max_iterations", config.matching.icp_max_iterations);
  config.matching.max_candidates_per_region = declareParam(
    node, "matching.max_candidates_per_region", config.matching.max_candidates_per_region);
  config.matching.icp_translation_tolerance = declareParam(
    node, "matching.icp_translation_tolerance", config.matching.icp_translation_tolerance);
  config.matching.icp_rotation_tolerance_rad = degreesToRadians(declareParam(
      node, "matching.icp_rotation_tolerance_deg",
      radiansToDegrees(config.matching.icp_rotation_tolerance_rad)));
  config.matching.max_alignment_error = declareParam(
    node, "matching.max_alignment_error", config.matching.max_alignment_error);
  config.matching.min_icp_correspondence_ratio = declareParam(
    node, "matching.min_icp_correspondence_ratio", config.matching.min_icp_correspondence_ratio);
  config.matching.min_model_coverage_ratio = declareParam(
    node, "matching.min_model_coverage_ratio", config.matching.min_model_coverage_ratio);
  config.matching.min_geometric_score = declareParam(
    node, "matching.min_geometric_score", config.matching.min_geometric_score);
  config.matching.min_confidence = declareParam(
    node, "matching.min_confidence", config.matching.min_confidence);

  config.temporal.required_consecutive_detections = declareParam(
    node, "temporal.required_consecutive_detections", config.temporal.required_consecutive_detections);
  config.temporal.max_missed_frames = declareParam(
    node, "temporal.max_missed_frames", config.temporal.max_missed_frames);
  config.temporal.max_position_jump = declareParam(
    node, "temporal.max_position_jump", config.temporal.max_position_jump);
  config.temporal.max_yaw_jump_rad = degreesToRadians(declareParam(
      node, "temporal.max_yaw_jump_deg", radiansToDegrees(config.temporal.max_yaw_jump_rad)));
  config.temporal.smoothing_alpha_position = declareParam(
    node, "temporal.smoothing_alpha_position", config.temporal.smoothing_alpha_position);
  config.temporal.smoothing_alpha_yaw = declareParam(
    node, "temporal.smoothing_alpha_yaw", config.temporal.smoothing_alpha_yaw);
  config.temporal.pending_reacquire_distance = declareParam(
    node, "temporal.pending_reacquire_distance", config.temporal.pending_reacquire_distance);
  config.temporal.pending_reacquire_yaw_rad = degreesToRadians(declareParam(
      node, "temporal.pending_reacquire_yaw_deg",
      radiansToDegrees(config.temporal.pending_reacquire_yaw_rad)));

  config.initial_estimate.use_initial_estimate = declareParam(
    node, "initial_estimate.use", config.initial_estimate.use_initial_estimate);
  config.initial_estimate.pose.x = declareParam(node, "initial_estimate.x", config.initial_estimate.pose.x);
  config.initial_estimate.pose.y = declareParam(node, "initial_estimate.y", config.initial_estimate.pose.y);
  config.initial_estimate.pose.yaw = degreesToRadians(declareParam(node, "initial_estimate.yaw_deg", 0.0));
  config.initial_estimate.pose.valid = config.initial_estimate.use_initial_estimate;

  std::size_t segment_count = 0;
  if (!config.landmark.vertices_x.empty() || !config.landmark.vertices_y.empty()) {
    if (config.landmark.vertices_x.size() != config.landmark.vertices_y.size()) {
      throw std::runtime_error("landmark.vertices_x and landmark.vertices_y must have the same length");
    }
    if (config.landmark.vertices_x.size() < 2U) {
      throw std::runtime_error("landmark.vertices_x/y must contain at least two vertices");
    }
    segment_count = config.landmark.vertices_x.size() - 1U;
    if (config.landmark.total_length <= 0.0) {
      config.landmark.total_length = 0.0;
      for (std::size_t i = 0; i + 1 < config.landmark.vertices_x.size(); ++i) {
        config.landmark.total_length += std::hypot(
          config.landmark.vertices_x[i + 1] - config.landmark.vertices_x[i],
          config.landmark.vertices_y[i + 1] - config.landmark.vertices_y[i]);
      }
    }
  } else {
    if (config.landmark.segment_lengths.empty()) {
      throw std::runtime_error("landmark.segment_lengths must not be empty");
    }
    if (config.landmark.turn_angles_rad.size() + 1U != config.landmark.segment_lengths.size()) {
      throw std::runtime_error("landmark.turn_angles_deg must have segment_lengths.size() - 1 entries");
    }
    segment_count = config.landmark.segment_lengths.size();
    if (config.landmark.total_length <= 0.0) {
      config.landmark.total_length = std::accumulate(
        config.landmark.segment_lengths.begin(), config.landmark.segment_lengths.end(), 0.0);
    }
  }

  config.landmark.min_visible_segments = std::clamp(
    config.landmark.min_visible_segments,
    2,
    static_cast<int>(segment_count));

  return config;
}

}  // namespace lidar_dock_detector
