#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/detection_filter.hpp"
#include "lidar_dock_detector/geometry_utils.hpp"
#include "lidar_dock_detector/landmark_matcher.hpp"
#include "lidar_dock_detector/line_extractor.hpp"
#include "lidar_dock_detector/nav2_pose_publisher.hpp"
#include "lidar_dock_detector/pose_estimator.hpp"
#include "lidar_dock_detector/region_splitter.hpp"
#include "lidar_dock_detector/scan_processor.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

struct PointBounds
{
  bool valid {false};
  double min_x {0.0};
  double max_x {0.0};
  double min_y {0.0};
  double max_y {0.0};
};

builtin_interfaces::msg::Time timeToMsg(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

double candidateSelectionScore(
  const CandidateDebug & candidate,
  const std::optional<Pose2D> & prior_pose,
  const DetectorConfig & config,
  const bool tracking_active)
{
  if (!prior_pose.has_value()) {
    return candidate.confidence;
  }

  const double allowed_distance = tracking_active ?
    config.matching.tracking_search_radius : config.matching.initial_search_radius;
  const double normalized_distance = allowed_distance > 1e-6 ?
    distance(candidate.dock_pose, *prior_pose) / allowed_distance : 0.0;
  const double allowed_yaw = tracking_active ?
    config.matching.tracking_yaw_tolerance_rad : config.matching.initial_yaw_tolerance_rad;
  const double normalized_yaw = allowed_yaw > 1e-6 ?
    std::abs(shortestAngularDistance(prior_pose->yaw, candidate.dock_pose.yaw)) / allowed_yaw : 0.0;
  const double distance_penalty = tracking_active ? 0.35 : 0.20;
  const double yaw_penalty = tracking_active ? 0.35 : 0.25;
  return candidate.confidence -
         distance_penalty * normalized_distance -
         yaw_penalty * normalized_yaw;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Pose2D transformPose(
  const Pose2D & pose,
  const geometry_msgs::msg::TransformStamped & transform)
{
  Pose2D transformed;
  const double transform_yaw = yawFromQuaternion(transform.transform.rotation);
  const Eigen::Vector2d transformed_xy =
    rotate(Eigen::Vector2d(pose.x, pose.y), transform_yaw) +
    Eigen::Vector2d(transform.transform.translation.x, transform.transform.translation.y);

  transformed.x = transformed_xy.x();
  transformed.y = transformed_xy.y();
  transformed.yaw = normalizeAngle(transform_yaw + pose.yaw);
  transformed.valid = pose.valid;
  return transformed;
}

PointBounds boundsOfPoints(const std::vector<Point2D> & points)
{
  PointBounds bounds;
  if (points.empty()) {
    return bounds;
  }

  bounds.valid = true;
  bounds.min_x = bounds.max_x = points.front().x;
  bounds.min_y = bounds.max_y = points.front().y;
  for (const auto & point : points) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  return bounds;
}

std::string formatBounds(const PointBounds & bounds)
{
  if (!bounds.valid) {
    return "empty";
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "x=[" << bounds.min_x << "," << bounds.max_x << "]"
       << " y=[" << bounds.min_y << "," << bounds.max_y << "]";
  return text.str();
}

std::string summarizeLines(const std::vector<LineSegment> & lines)
{
  if (lines.empty()) {
    return "none";
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i > 0U) {
      text << ";";
    }
    text << "l" << i
         << "(len=" << lines[i].length
         << ",yaw=" << radiansToDegrees(std::atan2(lines[i].direction.y(), lines[i].direction.x()))
         << ",rms=" << std::setprecision(3) << lines[i].rms_error << std::setprecision(2)
         << ",idx=" << lines[i].start_index << "-" << lines[i].end_index << ")";
  }
  return text.str();
}

void mergeLandmarkDebug(LandmarkMatchDebug & target, const LandmarkMatchDebug & source)
{
  target.windows += source.windows;
  target.transform_rejects += source.transform_rejects;
  target.length_rejects += source.length_rejects;
  target.angle_rejects += source.angle_rejects;
  target.vertex_rejects += source.vertex_rejects;
  target.geometric_rejects += source.geometric_rejects;
  target.best_length_score = std::max(target.best_length_score, source.best_length_score);
  target.best_angle_score = std::max(target.best_angle_score, source.best_angle_score);
  target.best_vertex_score = std::max(target.best_vertex_score, source.best_vertex_score);
  target.best_geometric_score = std::max(target.best_geometric_score, source.best_geometric_score);
}

std::string summarizeLandmarkDebug(const LandmarkMatchDebug & debug)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "win=" << debug.windows
       << " rej={tf:" << debug.transform_rejects
       << ",len:" << debug.length_rejects
       << ",ang:" << debug.angle_rejects
       << ",vtx:" << debug.vertex_rejects
       << ",geo:" << debug.geometric_rejects
       << "} best={len:" << debug.best_length_score
       << ",ang:" << debug.best_angle_score
       << ",vtx:" << debug.best_vertex_score
       << ",geo:" << debug.best_geometric_score
       << "}";
  return text.str();
}

std::string summarizePose(const Pose2D & pose)
{
  if (!pose.valid) {
    return "invalid";
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(3)
       << "x=" << pose.x
       << ",y=" << pose.y
       << ",yaw=" << std::setprecision(1) << radiansToDegrees(pose.yaw);
  return text.str();
}

std::string summarizeCandidate(const std::optional<CandidateDebug> & candidate)
{
  if (!candidate.has_value()) {
    return "none";
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "dock={" << summarizePose(candidate->dock_pose) << "}"
       << " marker={" << summarizePose(candidate->marker_pose) << "}"
       << " score=" << candidate->score
       << " conf=" << candidate->confidence
       << " align=" << std::setprecision(3) << candidate->alignment_error << std::setprecision(2)
       << " corr=" << candidate->correspondence_ratio
       << " cover=" << candidate->model_coverage_ratio
       << " lines=" << candidate->lines.size()
       << " pts=" << candidate->points.size();
  return text.str();
}

std::string summarizeReasons(const std::unordered_map<std::string, int> & reasons)
{
  if (reasons.empty()) {
    return "none";
  }

  std::ostringstream summary;
  bool first = true;
  for (const auto & [reason, count] : reasons) {
    if (!first) {
      summary << ",";
    }
    summary << reason << ":" << count;
    first = false;
  }
  return summary.str();
}

const char * stateName(const DetectionState state)
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

}  // namespace

class DockDetectorNode : public rclcpp::Node
{
public:
  DockDetectorNode()
  : Node("lidar_dock_detector"),
    config_(loadDetectorConfig(*this)),
    scan_processor_(config_.roi),
    region_splitter_(config_.splitting),
    line_extractor_(config_.line_extraction),
    landmark_matcher_(config_),
    pose_estimator_(config_),
    detection_filter_(config_.temporal, std::nullopt),
    publisher_(*this, config_),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    detection_filter_.setEnabled(config_.enabled_on_startup);

    scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      config_.topics.scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&DockDetectorNode::scanCallback, this, std::placeholders::_1));

    start_service_ = create_service<std_srvs::srv::Trigger>(
      config_.services.start_service,
      std::bind(&DockDetectorNode::handleStart, this, std::placeholders::_1,
        std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      config_.services.stop_service,
      std::bind(&DockDetectorNode::handleStop, this, std::placeholders::_1, std::placeholders::_2));
    toggle_service_ = create_service<std_srvs::srv::Trigger>(
      config_.services.toggle_service,
      std::bind(&DockDetectorNode::handleToggle, this, std::placeholders::_1,
        std::placeholders::_2));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        if (detection_filter_.state() == DetectionState::kIdle) {return;}
        const auto output =
        detection_filter_.watchdog(DetectionFilter::MonotonicTime::clock::now());
        if (output.state == DetectionState::kLost) {
          publisher_.publish(timeToMsg(now()), output, DetectionDebugFrame {});
        }
      });

    FilterOutput startup_state;
    startup_state.state =
      config_.enabled_on_startup ? DetectionState::kSearching : DetectionState::kIdle;
    publisher_.publish(timeToMsg(now()), startup_state, DetectionDebugFrame {});
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto monotonic_now = DetectionFilter::MonotonicTime::clock::now();
    DetectionDebugFrame debug_frame;

    if (detection_filter_.state() == DetectionState::kIdle) {
      FilterOutput idle_output;
      idle_output.state = DetectionState::kIdle;
      publisher_.publish(scan->header.stamp, idle_output, debug_frame);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        config_.topics.output_frame,
        scan->header.frame_id,
        rclcpp::Time(scan->header.stamp),
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Skipping scan: transform %s <- %s unavailable: %s",
        config_.topics.output_frame.c_str(),
        scan->header.frame_id.c_str(),
        ex.what());
      FilterOutput output = detection_filter_.update(std::nullopt, monotonic_now);
      publisher_.publish(scan->header.stamp, output, debug_frame);
      return;
    }

    debug_frame.roi_points = scan_processor_.process(*scan, transform);
    debug_frame.regions = region_splitter_.split(debug_frame.roi_points);

    std::optional<Pose2D> geometry_prior;
    if (detection_filter_.hasTrackedPose()) {
      try {
        const auto tracking_to_output = tf_buffer_.lookupTransform(
          config_.topics.output_frame, config_.topics.tracking_frame,
            rclcpp::Time(scan->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
        geometry_prior = transformPose(*detection_filter_.predictedDockPose(), tracking_to_output);
      } catch (const tf2::TransformException &) {
        // Keep perception running without a tracking gate; the tracking update below remains blocked.
      }
    } else if (config_.initial_estimate.use_initial_estimate) {
      geometry_prior = config_.initial_estimate.pose;
    }

    std::optional<CandidateDebug> best_candidate;
    double best_candidate_score = -1.0;
    int generated_matches = 0;
    int evaluated_candidates = 0;
    std::unordered_map<std::string, int> rejection_reasons;
    LandmarkMatchDebug landmark_debug;
    std::ostringstream region_summary;
    region_summary << std::fixed << std::setprecision(2);
    bool first_region = true;
    for (std::size_t region_index = 0; region_index < debug_frame.regions.size(); ++region_index) {
      const auto & region = debug_frame.regions[region_index];
      const auto extraction = line_extractor_.extract(region);
      debug_frame.fitted_lines.insert(
        debug_frame.fitted_lines.end(), extraction.lines.begin(), extraction.lines.end());
      debug_frame.split_points.insert(
        debug_frame.split_points.end(),
        extraction.debug.split_points.begin(),
        extraction.debug.split_points.end());
      debug_frame.residual_peaks.insert(
        debug_frame.residual_peaks.end(),
        extraction.debug.residual_peaks.begin(),
        extraction.debug.residual_peaks.end());
      debug_frame.rejected_line_segments.insert(
        debug_frame.rejected_line_segments.end(),
        extraction.debug.rejected_segments.begin(),
        extraction.debug.rejected_segments.end());

      LandmarkMatchDebug region_landmark_debug;
      const auto matches = landmark_matcher_.match(region, extraction.lines, region_landmark_debug);
      mergeLandmarkDebug(landmark_debug, region_landmark_debug);
      generated_matches += static_cast<int>(matches.size());
      if (!first_region) {
        region_summary << " | ";
      }
      first_region = false;
      region_summary
        << "r" << region_index
        << "{pts=" << region.points.size()
        << ",span=" << region.span
        << "," << formatBounds(boundsOfPoints(region.points))
        << ",lines=" << extraction.lines.size()
        << ",matches=" << matches.size()
        << "," << summarizeLandmarkDebug(region_landmark_debug)
        << ",line_detail=" << summarizeLines(extraction.lines)
        << "}";

      const std::optional<Pose2D> prior_pose = geometry_prior;
      int evaluated_region_candidates = 0;
      for (const auto & match : matches) {
        if (evaluated_region_candidates >= config_.matching.max_candidates_per_region) {
          break;
        }
        ++evaluated_region_candidates;
        ++evaluated_candidates;

        CandidateDebug candidate = pose_estimator_.estimate(
          match,
          prior_pose,
          detection_filter_.hasTrackedPose());

        if (candidate.accepted) {
          const double candidate_score = candidateSelectionScore(
            candidate,
            prior_pose,
            config_,
            detection_filter_.hasTrackedPose());
          if (!best_candidate.has_value() || candidate_score > best_candidate_score) {
            if (best_candidate.has_value()) {
              best_candidate->accepted = false;
              best_candidate->reason = "lower_score_than_best";
              debug_frame.rejected_candidates.push_back(*best_candidate);
            }
            best_candidate = candidate;
            best_candidate_score = candidate_score;
          } else {
            candidate.accepted = false;
            candidate.reason = "lower_score_than_best";
            debug_frame.rejected_candidates.push_back(candidate);
          }
        } else {
          rejection_reasons[candidate.reason]++;
          debug_frame.rejected_candidates.push_back(candidate);
        }
      }
    }

    debug_frame.accepted_candidate = best_candidate;
    std::optional<DockMeasurement> raw_measurement;
    std::optional<DockMeasurement> tracking_measurement;
    if (best_candidate.has_value()) {
      DockMeasurement raw;
      raw.marker_pose = best_candidate->marker_pose;
      raw.dock_pose = best_candidate->dock_pose;
      raw.stamp = scan->header.stamp;
      raw.frame_id = config_.topics.output_frame;
      raw.score = best_candidate->score;
      raw.confidence = best_candidate->confidence;
      raw.alignment_error = best_candidate->alignment_error;
      raw.correspondence_ratio = best_candidate->correspondence_ratio;
      raw.model_coverage_ratio = best_candidate->model_coverage_ratio;
      raw.valid = best_candidate->accepted;
      raw_measurement = raw;
      try {
        const auto transform_to_tracking = tf_buffer_.lookupTransform(
          config_.topics.tracking_frame, config_.topics.output_frame,
            rclcpp::Time(scan->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
        DockMeasurement tracked = raw;
        tracked.marker_pose = transformPose(raw.marker_pose, transform_to_tracking);
        tracked.dock_pose = transformPose(raw.dock_pose, transform_to_tracking);
        tracked.frame_id = config_.topics.tracking_frame;
        tracking_measurement = tracked;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Raw dock measurement retained but tracker update skipped: transform %s <- %s unavailable: %s",
          config_.topics.tracking_frame.c_str(), config_.topics.output_frame.c_str(), ex.what());
      }
    }
    FilterOutput output = detection_filter_.update(tracking_measurement, monotonic_now);
    if (config_.log_pipeline_debug) {
      const char * accepted = best_candidate.has_value() ? "yes" : "no";
      const double confidence = best_candidate.has_value() ? best_candidate->confidence : 0.0;
      const std::optional<Pose2D> prior_pose = detection_filter_.predictedDockPose();
      const std::string roi_bounds = formatBounds(boundsOfPoints(debug_frame.roi_points));
      const std::string prior_summary =
        prior_pose.has_value() ? summarizePose(*prior_pose) : "none";
      const std::string best_summary = summarizeCandidate(best_candidate);
      const std::string matcher_summary = summarizeLandmarkDebug(landmark_debug);
      const std::string reason_summary = summarizeReasons(rejection_reasons);
      const std::string region_summary_text = region_summary.str();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dock pipeline: roi=%zu roi_bounds=%s regions=%zu lines=%zu matches=%d eval=%d accepted=%s conf=%.2f state=%s prior={%s} best={%s} matcher={%s} reasons=%s region_detail=[%s]",
        debug_frame.roi_points.size(), roi_bounds.c_str(), debug_frame.regions.size(),
        debug_frame.fitted_lines.size(), generated_matches, evaluated_candidates, accepted,
          confidence,
        stateName(output.state), prior_summary.c_str(), best_summary.c_str(),
          matcher_summary.c_str(),
        reason_summary.c_str(), region_summary_text.c_str());
    }
    publisher_.publish(scan->header.stamp, output, debug_frame, raw_measurement);
  }

  bool detectionActive() const
  {
    return detection_filter_.state() != DetectionState::kIdle;
  }

  void startDetection()
  {
    detection_filter_.setEnabled(true);
    detection_filter_.reset();

    FilterOutput output;
    output.state = DetectionState::kSearching;
    publisher_.publish(timeToMsg(now()), output, DetectionDebugFrame {});
  }

  void stopDetection()
  {
    detection_filter_.setEnabled(false);

    FilterOutput output;
    output.state = DetectionState::kIdle;
    publisher_.publish(timeToMsg(now()), output, DetectionDebugFrame {});
    publisher_.clearDebug(timeToMsg(now()));
  }

  void handleStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    startDetection();

    response->success = true;
    response->message = "LiDAR dock detection started";
  }

  void handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stopDetection();

    response->success = true;
    response->message = "LiDAR dock detection stopped";
  }

  void handleToggle(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (detectionActive()) {
      stopDetection();
      response->success = true;
      response->message = "LiDAR dock detection toggled off";
      return;
    }

    startDetection();
    response->success = true;
    response->message = "LiDAR dock detection toggled on";
  }

  DetectorConfig config_;
  ScanProcessor scan_processor_;
  RegionSplitter region_splitter_;
  LineExtractor line_extractor_;
  LandmarkMatcher landmark_matcher_;
  PoseEstimator pose_estimator_;
  DetectionFilter detection_filter_;
  Nav2PosePublisher publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace lidar_dock_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_dock_detector::DockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
