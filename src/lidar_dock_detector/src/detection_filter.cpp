#include "lidar_dock_detector/detection_filter.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <cmath>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

FilterOutput holdTrackedPose(
  const DetectionState state,
  const Pose2D & marker_pose,
  const Pose2D & dock_pose,
  const double confidence)
{
  FilterOutput output;
  output.state = state;
  output.publish_pose = true;
  output.marker_pose = marker_pose;
  output.dock_pose = dock_pose;
  output.confidence = confidence;
  return output;
}

}  // namespace

DetectionFilter::DetectionFilter(TemporalConfig config, std::optional<Pose2D> initial_search_pose)
: config_(std::move(config)),
  initial_search_pose_(std::move(initial_search_pose))
{
}

void DetectionFilter::setEnabled(const bool enabled)
{
  enabled_ = enabled;
  if (!enabled_) {
    reset();
    state_ = DetectionState::kIdle;
  } else if (state_ == DetectionState::kIdle) {
    state_ = DetectionState::kSearching;
  }
}

void DetectionFilter::reset()
{
  consecutive_hits_ = 0;
  missed_frames_ = 0;
  pending_candidate_.reset();
  marker_pose_.reset();
  dock_pose_.reset();
  confidence_ = 0.0;
  if (enabled_) {
    state_ = DetectionState::kSearching;
  }
}

FilterOutput DetectionFilter::update(const std::optional<CandidateDebug> & candidate)
{
  FilterOutput output;

  if (!enabled_) {
    output.state = DetectionState::kIdle;
    return output;
  }

  if (!candidate.has_value() || !candidate->accepted) {
    missed_frames_++;
    if (dock_pose_.has_value() && missed_frames_ <= config_.max_missed_frames) {
      state_ = DetectionState::kTracking;
      return holdTrackedPose(state_, *marker_pose_, *dock_pose_, confidence_);
    }
    if (pending_candidate_.has_value() && missed_frames_ <= config_.max_missed_frames) {
      state_ = DetectionState::kCandidate;
      output.state = state_;
      output.confidence = pending_candidate_->confidence;
      return output;
    }

    clearTracking();
    state_ = DetectionState::kLost;
    output.state = state_;
    return output;
  }

  if (dock_pose_.has_value()) {
    if (distance(candidate->dock_pose, *dock_pose_) > config_.max_position_jump) {
      missed_frames_++;
      if (missed_frames_ <= config_.max_missed_frames) {
        state_ = DetectionState::kTracking;
        return holdTrackedPose(state_, *marker_pose_, *dock_pose_, confidence_);
      }
      clearTracking();
      state_ = DetectionState::kLost;
      output.state = state_;
      return output;
    }
    if (std::abs(shortestAngularDistance(dock_pose_->yaw, candidate->dock_pose.yaw)) > config_.max_yaw_jump_rad) {
      missed_frames_++;
      if (missed_frames_ <= config_.max_missed_frames) {
        state_ = DetectionState::kTracking;
        return holdTrackedPose(state_, *marker_pose_, *dock_pose_, confidence_);
      }
      clearTracking();
      state_ = DetectionState::kLost;
      output.state = state_;
      return output;
    }
  }

  missed_frames_ = 0;
  if (!isConsistentWithPending(*candidate)) {
    pending_candidate_ = candidate;
    consecutive_hits_ = 1;
    state_ = DetectionState::kCandidate;
  } else {
    pending_candidate_ = candidate;
    consecutive_hits_++;
  }

  if (!dock_pose_.has_value()) {
    if (consecutive_hits_ < config_.required_consecutive_detections) {
      state_ = DetectionState::kCandidate;
      output.state = state_;
      output.confidence = candidate->confidence;
      return output;
    }

    marker_pose_ = candidate->marker_pose;
    dock_pose_ = candidate->dock_pose;
    confidence_ = candidate->confidence;
  } else {
    *marker_pose_ = lowPassPose(
      *marker_pose_, candidate->marker_pose,
      config_.smoothing_alpha_position, config_.smoothing_alpha_yaw);
    *dock_pose_ = lowPassPose(
      *dock_pose_, candidate->dock_pose,
      config_.smoothing_alpha_position, config_.smoothing_alpha_yaw);
    confidence_ = 0.5 * confidence_ + 0.5 * candidate->confidence;
  }

  state_ = DetectionState::kTracking;
  output.state = state_;
  output.confidence = confidence_;
  output.publish_pose = true;
  output.fresh_pose = true;
  output.marker_pose = *marker_pose_;
  output.dock_pose = *dock_pose_;
  return output;
}

std::optional<Pose2D> DetectionFilter::predictedDockPose() const
{
  if (dock_pose_.has_value()) {
    return dock_pose_;
  }
  return initial_search_pose_;
}

bool DetectionFilter::hasTrackedPose() const
{
  return dock_pose_.has_value();
}

DetectionState DetectionFilter::state() const
{
  return state_;
}

bool DetectionFilter::isConsistentWithPending(const CandidateDebug & candidate) const
{
  if (!pending_candidate_.has_value()) {
    return false;
  }

  return
    distance(candidate.dock_pose, pending_candidate_->dock_pose) <= config_.pending_reacquire_distance &&
    std::abs(shortestAngularDistance(pending_candidate_->dock_pose.yaw, candidate.dock_pose.yaw)) <=
    config_.pending_reacquire_yaw_rad;
}

void DetectionFilter::clearTracking()
{
  consecutive_hits_ = 0;
  pending_candidate_.reset();
  marker_pose_.reset();
  dock_pose_.reset();
  confidence_ = 0.0;
}

}  // namespace lidar_dock_detector
