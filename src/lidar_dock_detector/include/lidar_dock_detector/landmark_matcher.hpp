#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/model.hpp"
#include "lidar_dock_detector/types.hpp"

#include <Eigen/Core>

#include <vector>

namespace lidar_dock_detector
{

class LandmarkMatcher
{
public:
  explicit LandmarkMatcher(const DetectorConfig & config);

  std::vector<MatchCandidate> match(
    const Region & region,
    const std::vector<LineSegment> & lines) const;
  std::vector<MatchCandidate> match(
    const Region & region,
    const std::vector<LineSegment> & lines,
    LandmarkMatchDebug & debug) const;

private:
  struct Evaluation
  {
    bool valid {false};
    MatchCandidate candidate;
  };

  Evaluation evaluateWindow(
    const Region & region,
    const std::vector<LineSegment> & ordered_lines,
    int model_start_index,
    LandmarkMatchDebug * debug) const;
  std::vector<Eigen::Vector2d> buildObservedVertices(
    const std::vector<LineSegment> & ordered_lines) const;
  std::vector<LineSegment> reverseLines(const std::vector<LineSegment> & lines) const;

  LandmarkModel model_;
  DetectorConfig config_;
};

}  // namespace lidar_dock_detector
