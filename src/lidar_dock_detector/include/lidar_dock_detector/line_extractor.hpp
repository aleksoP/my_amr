#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/types.hpp"

#include <vector>

namespace lidar_dock_detector
{

class LineExtractor
{
public:
  struct ExtractionDebug
  {
    std::vector<Point2D> split_points;
    std::vector<Point2D> residual_peaks;
    std::vector<RejectedLineSegmentDebug> rejected_segments;
  };

  struct ExtractionResult
  {
    std::vector<LineSegment> lines;
    ExtractionDebug debug;
  };

  explicit LineExtractor(LineExtractionConfig config);

  ExtractionResult extract(const Region & region) const;

private:
  struct FitResult
  {
    LineSegment line;
    double max_error {0.0};
    int split_index {-1};
  };

  FitResult fitLine(const std::vector<Point2D> & points, int start_index, int end_index) const;
  LineSegment buildEndpointSegment(const std::vector<Point2D> & points, int start_index, int end_index) const;
  void splitRecursive(
    const std::vector<Point2D> & points,
    int start_index,
    int end_index,
    ExtractionResult & result) const;
  std::vector<LineSegment> mergeCollinear(const std::vector<Point2D> & points, const std::vector<LineSegment> & lines) const;
  std::vector<Point2D> smoothPoints(const std::vector<Point2D> & points) const;

  LineExtractionConfig config_;
};

}  // namespace lidar_dock_detector
