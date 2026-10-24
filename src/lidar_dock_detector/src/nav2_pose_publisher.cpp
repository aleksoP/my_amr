#include "lidar_dock_detector/nav2_pose_publisher.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sstream>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace

Nav2PosePublisher::Nav2PosePublisher(rclcpp::Node & node, const DetectorConfig & config)
: config_(config),
  model_(buildLandmarkModel(config.landmark))
{
  pose_publisher_ =
    node.create_publisher<geometry_msgs::msg::PoseStamped>(config_.topics.pose_topic, 10);
  raw_marker_pose_publisher_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
    config_.topics.raw_marker_pose_topic, 10);
  raw_dock_pose_publisher_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
    config_.topics.raw_dock_pose_topic, 10);
  confidence_publisher_ =
    node.create_publisher<std_msgs::msg::Float32>(config_.topics.confidence_topic, 10);
  state_publisher_ = node.create_publisher<std_msgs::msg::String>(config_.topics.state_topic, 10);
  marker_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>(config_.topics.marker_topic, 10);
  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void Nav2PosePublisher::publish(
  const builtin_interfaces::msg::Time & stamp,
  const FilterOutput & filter_output,
  const DetectionDebugFrame & debug_frame,
  const std::optional<DockMeasurement> & raw_measurement)
{
  std_msgs::msg::Float32 confidence_msg;
  confidence_msg.data = static_cast<float>(filter_output.confidence);
  confidence_publisher_->publish(confidence_msg);

  std_msgs::msg::String state_msg;
  state_msg.data = stateToString(filter_output.state);
  state_publisher_->publish(state_msg);

  if (raw_measurement.has_value() && raw_measurement->valid) {
    publishPose(raw_marker_pose_publisher_, raw_measurement->stamp, raw_measurement->marker_pose,
      raw_measurement->frame_id);
    publishPose(raw_dock_pose_publisher_, raw_measurement->stamp, raw_measurement->dock_pose,
      raw_measurement->frame_id);
    publishRawTransforms(*raw_measurement);
  }
  // Nav2 compatibility output is emitted only for a fresh tracked measurement.
  if (filter_output.fresh_measurement && filter_output.usable) {
    publishPose(pose_publisher_, filter_output.last_measurement_stamp, filter_output.dock_pose,
      config_.topics.tracking_frame);
  }

  if (!config_.publish_debug_markers) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = config_.topics.output_frame;
  clear_marker.header.stamp = stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 0;
  marker_array.markers.push_back(makePointsMarker(
      marker_id++, "roi_points", stamp, debug_frame.roi_points, 0.0, 0.5, 1.0, 0.03));
  marker_array.markers.push_back(makeLineListMarker(
      marker_id++, "fitted_lines", stamp, debug_frame.fitted_lines, 0.95, 0.75, 0.1, 0.02));
  marker_array.markers.push_back(makePointsMarker(
      marker_id++, "split_points", stamp, debug_frame.split_points, 0.15, 1.0, 0.15, 0.05));
  marker_array.markers.push_back(makePointsMarker(
      marker_id++, "residual_peaks", stamp, debug_frame.residual_peaks, 1.0, 1.0, 1.0, 0.04));

  std::vector<LineSegment> rejected_lines;
  rejected_lines.reserve(debug_frame.rejected_line_segments.size());
  for (const auto & rejected_segment : debug_frame.rejected_line_segments) {
    rejected_lines.push_back(rejected_segment.line);
  }
  marker_array.markers.push_back(makeLineListMarker(
      marker_id++, "rejected_line_segments", stamp, rejected_lines, 1.0, 0.15, 0.15, 0.012));
  for (const auto & rejected_segment : debug_frame.rejected_line_segments) {
    Pose2D label_pose;
    label_pose.x = rejected_segment.line.centroid.x();
    label_pose.y = rejected_segment.line.centroid.y();
    label_pose.valid = true;
    marker_array.markers.push_back(makeTextMarker(
        marker_id++, "rejected_line_labels", stamp, label_pose, rejected_segment.reason));
  }

  for (const auto & rejected : debug_frame.rejected_candidates) {
    marker_array.markers.push_back(makeArrowMarker(
        marker_id++, "rejected_candidates", stamp, rejected.dock_pose, 0.95, 0.2, 0.2, 0.18));
    if (rejected.reason == "lower_score_than_best") {
      continue;
    }
    std::ostringstream text;
    text << rejected.reason << " " << static_cast<int>(rejected.confidence * 100.0) << "%";
    if (rejected.reason == "low_model_coverage") {
      text << " cover=" << static_cast<int>(rejected.model_coverage_ratio * 100.0) << "%";
    }
    marker_array.markers.push_back(makeTextMarker(
        marker_id++, "rejected_candidate_labels", stamp, rejected.dock_pose, text.str()));
  }

  if (debug_frame.accepted_candidate.has_value()) {
    marker_array.markers.push_back(makeLineListMarker(
        marker_id++, "accepted_model_lines", stamp,
        transformedModelLines(debug_frame.accepted_candidate->marker_pose),
        1.0, 0.1, 0.9, 0.018));
    marker_array.markers.push_back(makeArrowMarker(
        marker_id++, "accepted_marker_pose", stamp, debug_frame.accepted_candidate->marker_pose,
        0.1, 0.9, 0.3, 0.18));
    marker_array.markers.push_back(makeArrowMarker(
        marker_id++, "accepted_dock_pose", stamp, debug_frame.accepted_candidate->dock_pose,
        0.1, 0.9, 0.9, 0.20));
  }

  if (filter_output.has_estimate && config_.topics.tracking_frame == config_.topics.output_frame) {
    marker_array.markers.push_back(makeLineListMarker(
        marker_id++, "filtered_model_lines", stamp,
        transformedModelLines(filter_output.marker_pose),
        0.3, 1.0, 0.7, 0.016));
    marker_array.markers.push_back(makeArrowMarker(
        marker_id++, "filtered_dock_pose", stamp, filter_output.dock_pose, 0.1, 1.0, 0.2, 0.25));
    marker_array.markers.push_back(makeArrowMarker(
        marker_id++, "marker_frame", stamp, filter_output.marker_pose, 0.1, 0.7, 1.0, 0.20));

    std::ostringstream text;
    text << stateToString(filter_output.state) << " " <<
      static_cast<int>(filter_output.confidence * 100.0) << "%";
    marker_array.markers.push_back(makeTextMarker(
        marker_id++, "detector_state", stamp, filter_output.dock_pose, text.str()));
  }

  marker_publisher_->publish(marker_array);
}

void Nav2PosePublisher::clearDebug(const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = config_.topics.output_frame;
  clear_marker.header.stamp = stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  marker_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker Nav2PosePublisher::makeArrowMarker(
  const int id,
  const std::string & ns,
  const builtin_interfaces::msg::Time & stamp,
  const Pose2D & pose,
  const double r,
  const double g,
  const double b,
  const double scale) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.topics.output_frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = pose.x;
  marker.pose.position.y = pose.y;
  marker.pose.position.z = 0.02;
  marker.pose.orientation = yawToQuaternion(pose.yaw);
  marker.scale.x = scale;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color.a = 0.95;
  marker.color.r = static_cast<float>(r);
  marker.color.g = static_cast<float>(g);
  marker.color.b = static_cast<float>(b);
  return marker;
}

visualization_msgs::msg::Marker Nav2PosePublisher::makePointsMarker(
  const int id,
  const std::string & ns,
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<Point2D> & points,
  const double r,
  const double g,
  const double b,
  const double scale) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.topics.output_frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.color.a = 0.8F;
  marker.color.r = static_cast<float>(r);
  marker.color.g = static_cast<float>(g);
  marker.color.b = static_cast<float>(b);
  marker.points.reserve(points.size());
  for (const auto & point : points) {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.01;
    marker.points.push_back(marker_point);
  }
  return marker;
}

visualization_msgs::msg::Marker Nav2PosePublisher::makeLineListMarker(
  const int id,
  const std::string & ns,
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<LineSegment> & lines,
  const double r,
  const double g,
  const double b,
  const double width) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.topics.output_frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = width;
  marker.color.a = 0.9F;
  marker.color.r = static_cast<float>(r);
  marker.color.g = static_cast<float>(g);
  marker.color.b = static_cast<float>(b);
  marker.points.reserve(lines.size() * 2U);
  for (const auto & line : lines) {
    geometry_msgs::msg::Point start;
    start.x = line.start.x();
    start.y = line.start.y();
    start.z = 0.03;
    geometry_msgs::msg::Point end;
    end.x = line.end.x();
    end.y = line.end.y();
    end.z = 0.03;
    marker.points.push_back(start);
    marker.points.push_back(end);
  }
  return marker;
}

visualization_msgs::msg::Marker Nav2PosePublisher::makeTextMarker(
  const int id,
  const std::string & ns,
  const builtin_interfaces::msg::Time & stamp,
  const Pose2D & pose,
  const std::string & text) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.topics.output_frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = pose.x;
  marker.pose.position.y = pose.y;
  marker.pose.position.z = 0.25;
  marker.scale.z = 0.12;
  marker.color.a = 0.95F;
  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.text = text;
  return marker;
}

std::vector<LineSegment> Nav2PosePublisher::transformedModelLines(const Pose2D & pose) const
{
  std::vector<LineSegment> lines;
  if (!pose.valid || model_.vertices.size() < 2U) {
    return lines;
  }

  const RigidTransform2D transform = poseToTransform(pose);
  lines.reserve(model_.vertices.size() - 1U);
  for (std::size_t i = 0; i + 1 < model_.vertices.size(); ++i) {
    LineSegment line;
    line.start = applyTransform(transform, model_.vertices[i]);
    line.end = applyTransform(transform, model_.vertices[i + 1]);
    line.centroid = 0.5 * (line.start + line.end);
    const Eigen::Vector2d delta = line.end - line.start;
    line.length = delta.norm();
    line.direction = line.length > 1e-6 ?
      Eigen::Vector2d(delta / line.length) : Eigen::Vector2d::UnitX();
    lines.push_back(line);
  }
  return lines;
}

void Nav2PosePublisher::publishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher,
  const builtin_interfaces::msg::Time & stamp, const Pose2D & pose,
  const std::string & frame_id) const
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position.x = pose.x;
  pose_msg.pose.position.y = pose.y;
  pose_msg.pose.orientation = yawToQuaternion(pose.yaw);
  publisher->publish(pose_msg);
}

void Nav2PosePublisher::publishRawTransforms(const DockMeasurement & measurement) const
{
  if (!config_.publish_tf || !measurement.valid || config_.topics.marker_frame.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = measurement.frame_id;
  transform.header.stamp = measurement.stamp;
  transform.child_frame_id = config_.topics.marker_frame;
  transform.transform.translation.x = measurement.marker_pose.x;
  transform.transform.translation.y = measurement.marker_pose.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = yawToQuaternion(measurement.marker_pose.yaw);
  transform_broadcaster_->sendTransform(transform);
  if (!config_.topics.dock_frame.empty()) {
    transform.header.frame_id = config_.topics.marker_frame;
    transform.child_frame_id = config_.topics.dock_frame;
    transform.transform.translation.x = config_.landmark.marker_to_dock_offset.x;
    transform.transform.translation.y = config_.landmark.marker_to_dock_offset.y;
    transform.transform.rotation = yawToQuaternion(config_.landmark.marker_to_dock_offset.yaw);
    transform_broadcaster_->sendTransform(transform);
  }
}

std::string Nav2PosePublisher::stateToString(const DetectionState state)
{
  switch (state) {
    case DetectionState::kIdle:
      return "IDLE";
    case DetectionState::kSearching:
      return "SEARCHING";
    case DetectionState::kCandidate:
      return "CANDIDATE";
    case DetectionState::kTracking:
      return "TRACKING";
    case DetectionState::kLost:
      return "LOST";
    default:
      return "UNKNOWN";
  }
}

}  // namespace lidar_dock_detector
