#include "lidar_dock_detector/detection_filter.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <cmath>
#include <utility>

namespace lidar_dock_detector
{

DetectionFilter::DetectionFilter(TemporalConfig config, std::optional<Pose2D> initial_search_pose)
: config_(std::move(config)), initial_search_pose_(std::move(initial_search_pose)) {}

void DetectionFilter::setEnabled(const bool enabled)
{
  enabled_ = enabled;
  reset();
  if (!enabled_) {state_ = DetectionState::kIdle;} else {
    enabled_time_ = MonotonicTime::clock::now();
  }
}

void DetectionFilter::reset()
{
  consecutive_hits_ = 0;
  pending_measurement_.reset(); marker_pose_.reset(); dock_pose_.reset();
  confidence_ = 0.0; last_measurement_stamp_.reset(); last_measurement_time_.reset();
  last_scan_time_.reset(); acquisition_start_time_.reset();
  if (enabled_) {state_ = DetectionState::kSearching;}
}

FilterOutput DetectionFilter::update(
  const std::optional<DockMeasurement> & measurement, const MonotonicTime now)
{
  if (!enabled_) {return output(false, now);}
  last_scan_time_ = now;
  if (!measurement.has_value() || !measurement->valid) {
    // Acquisition is intentionally strict: a dropout resets unconfirmed evidence.
    if (!dock_pose_.has_value()) {
      consecutive_hits_ = 0; pending_measurement_.reset(); acquisition_start_time_.reset();
      state_ = DetectionState::kSearching;
    }
    return output(false, now);
  }
  if (last_measurement_stamp_.has_value()) {
    const auto & previous = *last_measurement_stamp_;
    if (measurement->stamp.sec < previous.sec ||
      (measurement->stamp.sec == previous.sec && measurement->stamp.nanosec <= previous.nanosec))
    {return output(false, now);}
  }
  if (dock_pose_.has_value() &&
    (distance(measurement->dock_pose, *dock_pose_) > config_.max_position_jump ||
    std::abs(shortestAngularDistance(dock_pose_->yaw, measurement->dock_pose.yaw)) >
    config_.max_yaw_jump_rad))
  {return output(false, now);}

  if (!dock_pose_.has_value()) {
    const bool same = isConsistentWithPending(*measurement);
    if (!same || !acquisition_start_time_.has_value() ||
      now - *acquisition_start_time_ >
      std::chrono::duration<double>(config_.acquisition_timeout_sec))
    {
      pending_measurement_ = measurement; consecutive_hits_ = 1; acquisition_start_time_ = now;
      state_ = DetectionState::kCandidate;
    } else {
      pending_measurement_ = measurement; ++consecutive_hits_;
    }
    if (consecutive_hits_ < config_.required_consecutive_detections) {return output(false, now);}
    marker_pose_ = measurement->marker_pose; dock_pose_ = measurement->dock_pose;
    confidence_ = measurement->confidence;
  } else {
    *marker_pose_ = lowPassPose(*marker_pose_, measurement->marker_pose,
        config_.smoothing_alpha_position, config_.smoothing_alpha_yaw);
    *dock_pose_ = lowPassPose(*dock_pose_, measurement->dock_pose,
        config_.smoothing_alpha_position, config_.smoothing_alpha_yaw);
    confidence_ = 0.5 * confidence_ + 0.5 * measurement->confidence;
  }
  pending_measurement_ = measurement; last_measurement_stamp_ = measurement->stamp;
  last_measurement_time_ = now; state_ = DetectionState::kTracking;
  return output(true, now);
}

FilterOutput DetectionFilter::watchdog(const MonotonicTime now)
{
  if (!enabled_) {return output(false, now);}
  if ((!last_scan_time_.has_value() && enabled_time_.has_value() &&
    now - *enabled_time_ > std::chrono::duration<double>(config_.scan_timeout_sec)) ||
    (last_scan_time_.has_value() && now - *last_scan_time_ >
    std::chrono::duration<double>(config_.scan_timeout_sec)) ||
    (last_measurement_time_.has_value() && now - *last_measurement_time_ >
    std::chrono::duration<double>(config_.lost_timeout_sec)))
  {clearTracking(); state_ = DetectionState::kLost;}
  return output(false, now);
}

FilterOutput DetectionFilter::output(const bool fresh, const MonotonicTime now) const
{
  FilterOutput result; result.state = state_; result.fresh_measurement = fresh;
  result.has_estimate = dock_pose_.has_value(); result.confidence = confidence_;
  if (result.has_estimate) {result.marker_pose = *marker_pose_; result.dock_pose = *dock_pose_;}
  if (last_measurement_stamp_.has_value()) {
    result.last_measurement_stamp = *last_measurement_stamp_;
  }
  if (last_measurement_time_.has_value()) {
    result.stale = now - *last_measurement_time_ >
      std::chrono::duration<double>(config_.stale_timeout_sec);
    result.usable = result.has_estimate && !result.stale && state_ == DetectionState::kTracking;
  }
  return result;
}

std::optional<Pose2D> DetectionFilter::predictedDockPose() const
{return dock_pose_.has_value() ? dock_pose_ : initial_search_pose_;}
bool DetectionFilter::hasTrackedPose() const {return dock_pose_.has_value();}
DetectionState DetectionFilter::state() const {return state_;}
bool DetectionFilter::isConsistentWithPending(const DockMeasurement & measurement) const
{
  return pending_measurement_.has_value() &&
         distance(measurement.dock_pose,
      pending_measurement_->dock_pose) <= config_.pending_reacquire_distance &&
         std::abs(shortestAngularDistance(pending_measurement_->dock_pose.yaw,
      measurement.dock_pose.yaw)) <=
         config_.pending_reacquire_yaw_rad;
}
void DetectionFilter::clearTracking()
{
  consecutive_hits_ = 0; pending_measurement_.reset(); marker_pose_.reset(); dock_pose_.reset();
  confidence_ = 0.0; last_measurement_stamp_.reset(); last_measurement_time_.reset();
  acquisition_start_time_.reset();
}

}  // namespace lidar_dock_detector
