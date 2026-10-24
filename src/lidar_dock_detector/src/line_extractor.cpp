#include "lidar_dock_detector/line_extractor.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lidar_dock_detector
{

namespace
{

// Returns the interior index with the largest consecutive direction change,
// or -1 if none exceeds the threshold. Used as a fallback when residual-based
// splitting is blocked by min_line_points (e.g. when the apex sits at the
// start of a sub-range and its smoothing bias inflates the residual of index 1,
// preventing the true shoulder/slope junction from being reached).
int findAngleCorner(
  const std::vector<Point2D> & points,
  const int start_index,
  const int end_index,
  const double angle_threshold_rad)
{
  const int point_count = end_index - start_index + 1;
  if (point_count < 5) {
    return -1;
  }

  const int window = std::clamp(point_count / 20, 2, 20);
  double max_change = 0.0;
  int best = -1;
  for (int i = start_index + window; i <= end_index - window; ++i) {
    const Eigen::Vector2d before =
      toEigen(points[static_cast<std::size_t>(i)]) -
      toEigen(points[static_cast<std::size_t>(i - window)]);
    const Eigen::Vector2d after =
      toEigen(points[static_cast<std::size_t>(i + window)]) -
      toEigen(points[static_cast<std::size_t>(i)]);
    if (before.squaredNorm() < 1e-10 || after.squaredNorm() < 1e-10) {
      continue;
    }
    const double diff = std::abs(signedTurnAngle(before.normalized(), after.normalized()));
    if (diff > max_change) {
      max_change = diff;
      best = i;
    }
  }
  return (max_change >= angle_threshold_rad) ? best : -1;
}

bool splitKeepsUsableGeometry(
  const std::vector<Point2D> & points,
  const int start_index,
  const int split_index,
  const int end_index,
  const double min_child_length)
{
  if (split_index <= start_index || split_index >= end_index) {
    return false;
  }

  const double left_length =
    (toEigen(points[static_cast<std::size_t>(split_index)]) -
    toEigen(points[static_cast<std::size_t>(start_index)])).norm();
  const double right_length =
    (toEigen(points[static_cast<std::size_t>(end_index)]) -
    toEigen(points[static_cast<std::size_t>(split_index)])).norm();

  return left_length >= min_child_length && right_length >= min_child_length;
}

}  // namespace

LineExtractor::LineExtractor(LineExtractionConfig config)
: config_(std::move(config))
{
}

LineExtractor::ExtractionResult LineExtractor::extract(const Region & region) const
{
  ExtractionResult result;
  if (static_cast<int>(region.points.size()) < config_.min_line_points) {
    return result;
  }

  const std::vector<Point2D> smoothed = smoothPoints(region.points);

  splitRecursive(smoothed, 0, static_cast<int>(smoothed.size()) - 1, result);
  std::sort(
    result.lines.begin(), result.lines.end(),
    [](const LineSegment & a, const LineSegment & b) {
      return a.start_index < b.start_index;
    });

  result.lines = mergeCollinear(smoothed, result.lines);
  return result;
}

LineExtractor::FitResult LineExtractor::fitLine(
  const std::vector<Point2D> & points,
  const int start_index,
  const int end_index) const
{
  FitResult result;
  result.line.start_index = start_index;
  result.line.end_index = end_index;

  const int count = end_index - start_index + 1;
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (int i = start_index; i <= end_index; ++i) {
    centroid += toEigen(points[static_cast<std::size_t>(i)]);
  }
  centroid /= static_cast<double>(count);

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (int i = start_index; i <= end_index; ++i) {
    const Eigen::Vector2d delta = toEigen(points[static_cast<std::size_t>(i)]) - centroid;
    covariance += delta * delta.transpose();
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  Eigen::Vector2d direction = solver.eigenvectors().col(1).normalized();

  const Eigen::Vector2d raw_start = toEigen(points[static_cast<std::size_t>(start_index)]);
  const Eigen::Vector2d raw_end = toEigen(points[static_cast<std::size_t>(end_index)]);
  if (direction.dot(raw_end - raw_start) < 0.0) {
    direction = -direction;
  }

  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  double squared_error_sum = 0.0;
  double max_error = 0.0;
  double interior_max_error = 0.0;
  int split_index = -1;

  for (int i = start_index; i <= end_index; ++i) {
    const Eigen::Vector2d point = toEigen(points[static_cast<std::size_t>(i)]);
    const Eigen::Vector2d delta = point - centroid;
    const double projection = delta.dot(direction);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);

    const double error = pointLineDistance(point, centroid, direction);
    squared_error_sum += error * error;
    if (error > max_error) {
      max_error = error;
    }
    // Only interior points are valid split candidates — endpoints are sub-range
    // boundaries shared with the parent or sibling call, so splitting there
    // would be blocked by the start/end guard in splitRecursive anyway and
    // silently prevents further subdivision (e.g. the apex after the first split).
    if (i > start_index && i < end_index && error > interior_max_error) {
      interior_max_error = error;
      split_index = i;
    }
  }

  result.line.start = centroid + min_projection * direction;
  result.line.end = centroid + max_projection * direction;
  result.line.centroid = centroid;
  result.line.direction = direction;
  result.line.length = (result.line.end - result.line.start).norm();
  result.line.rms_error = std::sqrt(squared_error_sum / static_cast<double>(count));
  result.max_error = max_error;
  result.split_index = split_index;
  return result;
}

LineSegment LineExtractor::buildEndpointSegment(
  const std::vector<Point2D> & points,
  const int start_index,
  const int end_index) const
{
  LineSegment line;
  line.start_index = start_index;
  line.end_index = end_index;
  line.start = toEigen(points[static_cast<std::size_t>(start_index)]);
  line.end = toEigen(points[static_cast<std::size_t>(end_index)]);
  line.centroid = 0.5 * (line.start + line.end);
  const Eigen::Vector2d delta = line.end - line.start;
  line.length = delta.norm();
  line.direction = line.length > 1e-6 ?
    Eigen::Vector2d(delta / line.length) : Eigen::Vector2d(1.0, 0.0);
  return line;
}

void LineExtractor::splitRecursive(
  const std::vector<Point2D> & points,
  const int start_index,
  const int end_index,
  ExtractionResult & result) const
{
  const int point_count = end_index - start_index + 1;
  if (point_count < config_.min_line_points) {
    if (point_count >= 2) {
      RejectedLineSegmentDebug rejected;
      rejected.line = buildEndpointSegment(points, start_index, end_index);
      rejected.reason = "too_few_points";
      rejected.point_count = point_count;
      result.debug.rejected_segments.push_back(rejected);
    }
    return;
  }

  const FitResult fit = fitLine(points, start_index, end_index);
  if (fit.split_index >= start_index && fit.split_index <= end_index) {
    result.debug.residual_peaks.push_back(points[static_cast<std::size_t>(fit.split_index)]);
  }

  const int left_count = fit.split_index >= 0 ? (fit.split_index - start_index + 1) : 0;
  const int right_count = fit.split_index >= 0 ? (end_index - fit.split_index + 1) : 0;

  if (
    fit.max_error > config_.split_threshold &&
    fit.split_index >= 0 &&
    left_count >= config_.min_line_points &&
    right_count >= config_.min_line_points &&
    splitKeepsUsableGeometry(
      points,
      start_index,
      fit.split_index,
      end_index,
      config_.min_line_length))
  {
    result.debug.split_points.push_back(points[static_cast<std::size_t>(fit.split_index)]);
    splitRecursive(points, start_index, fit.split_index, result);
    splitRecursive(points, fit.split_index, end_index, result);
    return;
  }

  if (config_.corner_angle_threshold_rad > 0.0) {
    const int corner = findAngleCorner(
      points, start_index, end_index, config_.corner_angle_threshold_rad);
    if (corner >= 0 &&
      (corner - start_index + 1) >= config_.min_line_points &&
      (end_index - corner + 1) >= config_.min_line_points &&
      splitKeepsUsableGeometry(
        points,
        start_index,
        corner,
        end_index,
        config_.min_line_length))
    {
      result.debug.split_points.push_back(points[static_cast<std::size_t>(corner)]);
      splitRecursive(points, start_index, corner, result);
      splitRecursive(points, corner, end_index, result);
      return;
    }
  }

  if (fit.line.length < config_.min_line_length) {
    RejectedLineSegmentDebug rejected;
    rejected.line = fit.line;
    rejected.reason = "too_short";
    rejected.max_error = fit.max_error;
    rejected.point_count = point_count;
    result.debug.rejected_segments.push_back(rejected);
    return;
  }

  if (fit.line.rms_error > config_.max_fit_error) {
    RejectedLineSegmentDebug rejected;
    rejected.line = fit.line;
    rejected.reason = "fit_error";
    rejected.max_error = fit.max_error;
    rejected.point_count = point_count;
    result.debug.rejected_segments.push_back(rejected);
    return;
  }

  result.lines.push_back(fit.line);
}

std::vector<LineSegment> LineExtractor::mergeCollinear(
  const std::vector<Point2D> & points,
  const std::vector<LineSegment> & lines) const
{
  if (lines.size() < 2U) {
    return lines;
  }

  std::vector<LineSegment> merged;
  std::size_t i = 0;
  while (i < lines.size()) {
    LineSegment current = lines[i];
    std::size_t next = i + 1;

    while (next < lines.size()) {
      const double angle = std::abs(signedTurnAngle(current.direction, lines[next].direction));
      const double gap = (current.end - lines[next].start).norm();
      if (angle > config_.merge_angle_threshold_rad || gap > config_.merge_gap_threshold) {
        break;
      }

      const FitResult refit = fitLine(points, current.start_index, lines[next].end_index);
      if (refit.line.rms_error > config_.max_fit_error) {
        break;
      }

      current = refit.line;
      next++;
    }

    merged.push_back(current);
    i = next;
  }

  return merged;
}

std::vector<Point2D> LineExtractor::smoothPoints(const std::vector<Point2D> & points) const
{
  const int w = config_.smoothing_window;
  if (w <= 0) {
    return points;
  }

  const int n = static_cast<int>(points.size());
  std::vector<Point2D> smoothed;
  smoothed.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    const int lo = std::max(0, i - w);
    const int hi = std::min(n - 1, i + w);
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int j = lo; j <= hi; ++j) {
      sum_x += points[static_cast<std::size_t>(j)].x;
      sum_y += points[static_cast<std::size_t>(j)].y;
    }
    const double count = static_cast<double>(hi - lo + 1);
    smoothed.push_back({sum_x / count, sum_y / count});
  }

  return smoothed;
}

}  // namespace lidar_dock_detector
