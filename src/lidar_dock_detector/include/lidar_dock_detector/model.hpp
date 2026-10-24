#pragma once

#include "lidar_dock_detector/config.hpp"

#include <Eigen/Core>

#include <vector>

namespace lidar_dock_detector
{

struct LandmarkModel
{
  std::vector<Eigen::Vector2d> vertices;
  std::vector<double> segment_lengths;
  std::vector<double> turn_angles_rad;
  std::vector<Eigen::Vector2d> samples;
  double total_length {0.0};
};

LandmarkModel buildLandmarkModel(const LandmarkConfig & config);

}  // namespace lidar_dock_detector
