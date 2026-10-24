#include "lidar_dock_detector/landmark_matcher.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

double normalizedLineScore(const double error, const double tolerance)
{
  if (tolerance <= 0.0) {
    return 0.0;
  }
  return clamp01(1.0 - std::abs(error) / tolerance);
}

void updateBestScores(
  LandmarkMatchDebug * debug,
  const double length_score,
  const double angle_score,
  const double vertex_score,
  const double geometric_score)
{
  if (debug == nullptr) {
    return;
  }
  debug->best_length_score = std::max(debug->best_length_score, length_score);
  debug->best_angle_score = std::max(debug->best_angle_score, angle_score);
  debug->best_vertex_score = std::max(debug->best_vertex_score, vertex_score);
  debug->best_geometric_score = std::max(debug->best_geometric_score, geometric_score);
}

}  // namespace

LandmarkMatcher::LandmarkMatcher(const DetectorConfig & config)
: model_(buildLandmarkModel(config.landmark)),
  config_(config)
{
}

std::vector<MatchCandidate> LandmarkMatcher::match(
  const Region & region,
  const std::vector<LineSegment> & lines) const
{
  LandmarkMatchDebug unused_debug;
  return match(region, lines, unused_debug);
}

std::vector<MatchCandidate> LandmarkMatcher::match(
  const Region & region,
  const std::vector<LineSegment> & lines,
  LandmarkMatchDebug & debug) const
{
  std::vector<MatchCandidate> matches;
  const int max_visible = std::min(
    static_cast<int>(lines.size()),
    static_cast<int>(model_.segment_lengths.size()));

  for (int visible_segments = config_.landmark.min_visible_segments; visible_segments <= max_visible;
    ++visible_segments)
  {
    for (int line_start = 0; line_start + visible_segments <= static_cast<int>(lines.size()); ++line_start) {
      std::vector<LineSegment> line_window(
        lines.begin() + line_start,
        lines.begin() + line_start + visible_segments);

      const auto try_window = [&](const std::vector<LineSegment> & ordered_lines) {
          for (
            int model_start = 0;
            model_start + visible_segments <= static_cast<int>(model_.segment_lengths.size());
            ++model_start)
          {
            const Evaluation evaluation = evaluateWindow(region, ordered_lines, model_start, &debug);
            if (evaluation.valid) {
              matches.push_back(evaluation.candidate);
            }
          }
        };

      try_window(line_window);
      try_window(reverseLines(line_window));
    }
  }

  std::sort(
    matches.begin(), matches.end(),
    [](const MatchCandidate & a, const MatchCandidate & b) {
      return a.geometric_confidence > b.geometric_confidence;
    });
  return matches;
}

LandmarkMatcher::Evaluation LandmarkMatcher::evaluateWindow(
  const Region & region,
  const std::vector<LineSegment> & ordered_lines,
  const int model_start_index,
  LandmarkMatchDebug * debug) const
{
  Evaluation evaluation;
  if (ordered_lines.empty()) {
    return evaluation;
  }
  if (debug != nullptr) {
    debug->windows++;
  }

  std::vector<double> observed_lengths;
  observed_lengths.reserve(ordered_lines.size());
  for (const auto & line : ordered_lines) {
    observed_lengths.push_back(line.length);
  }

  double length_score = 0.0;
  for (std::size_t i = 0; i < ordered_lines.size(); ++i) {
    const double error = observed_lengths[i] -
      model_.segment_lengths[static_cast<std::size_t>(model_start_index) + i];
    length_score += normalizedLineScore(error, config_.landmark.length_tolerance);
  }
  length_score /= static_cast<double>(ordered_lines.size());

  double angle_score = 1.0;
  if (ordered_lines.size() > 1U) {
    angle_score = 0.0;
    for (std::size_t i = 0; i + 1 < ordered_lines.size(); ++i) {
      const double observed_turn = signedTurnAngle(ordered_lines[i].direction, ordered_lines[i + 1].direction);
      const double expected_turn =
        model_.turn_angles_rad[static_cast<std::size_t>(model_start_index) + i];
      angle_score += normalizedLineScore(
        shortestAngularDistance(expected_turn, observed_turn),
        config_.landmark.angle_tolerance_rad);
    }
    angle_score /= static_cast<double>(ordered_lines.size() - 1U);
  }

  const std::vector<Eigen::Vector2d> observed_vertices = buildObservedVertices(ordered_lines);
  std::vector<Eigen::Vector2d> model_vertices;
  model_vertices.reserve(observed_vertices.size());
  for (std::size_t i = 0; i < observed_vertices.size(); ++i) {
    model_vertices.push_back(model_.vertices[static_cast<std::size_t>(model_start_index) + i]);
  }

  const auto transform = estimateRigidTransform(model_vertices, observed_vertices);
  if (!transform.has_value()) {
    if (debug != nullptr) {
      debug->transform_rejects++;
    }
    return evaluation;
  }

  std::vector<Eigen::Vector2d> aligned_vertices;
  aligned_vertices.reserve(model_vertices.size());
  for (const auto & vertex : model_vertices) {
    aligned_vertices.push_back(applyTransform(*transform, vertex));
  }
  const double vertex_rmse = rmse(aligned_vertices, observed_vertices);
  const double vertex_score = normalizedLineScore(vertex_rmse, config_.landmark.length_tolerance);

  const double geometric_score = 0.45 * length_score + 0.35 * angle_score + 0.20 * vertex_score;
  updateBestScores(debug, length_score, angle_score, vertex_score, geometric_score);
  if (geometric_score < config_.matching.min_geometric_score) {
    if (debug != nullptr) {
      if (length_score <= 0.0) {
        debug->length_rejects++;
      } else if (angle_score <= 0.0) {
        debug->angle_rejects++;
      } else if (vertex_score <= 0.0) {
        debug->vertex_rejects++;
      } else {
        debug->geometric_rejects++;
      }
    }
    return evaluation;
  }

  const Pose2D marker_pose = transformToPose(*transform);
  int point_start = ordered_lines.front().start_index;
  int point_end = ordered_lines.front().end_index;
  for (const auto & line : ordered_lines) {
    point_start = std::min(point_start, line.start_index);
    point_start = std::min(point_start, line.end_index);
    point_end = std::max(point_end, line.start_index);
    point_end = std::max(point_end, line.end_index);
  }

  MatchCandidate candidate;
  candidate.lines = ordered_lines;
  candidate.initial_marker_pose = marker_pose;
  candidate.geometric_score = geometric_score;
  candidate.geometric_confidence = clamp01(0.5 * geometric_score + 0.5 * vertex_score);
  candidate.points.insert(
    candidate.points.end(),
    region.points.begin() + point_start,
    region.points.begin() + point_end + 1);

  evaluation.valid = true;
  evaluation.candidate = candidate;
  return evaluation;
}

std::vector<Eigen::Vector2d> LandmarkMatcher::buildObservedVertices(
  const std::vector<LineSegment> & ordered_lines) const
{
  std::vector<Eigen::Vector2d> vertices;
  vertices.reserve(ordered_lines.size() + 1U);
  vertices.push_back(ordered_lines.front().start);
  for (std::size_t i = 0; i + 1 < ordered_lines.size(); ++i) {
    vertices.push_back(0.5 * (ordered_lines[i].end + ordered_lines[i + 1].start));
  }
  vertices.push_back(ordered_lines.back().end);
  return vertices;
}

std::vector<LineSegment> LandmarkMatcher::reverseLines(const std::vector<LineSegment> & lines) const
{
  std::vector<LineSegment> reversed(lines.rbegin(), lines.rend());
  for (auto & line : reversed) {
    std::swap(line.start, line.end);
    line.direction = -line.direction;
  }
  return reversed;
}

}  // namespace lidar_dock_detector
