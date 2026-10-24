#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/types.hpp"

#include <vector>

namespace lidar_dock_detector
{

class RegionSplitter
{
public:
  explicit RegionSplitter(SplitConfig config);

  std::vector<Region> split(const std::vector<Point2D> & points) const;

private:
  bool isRegionValid(const Region & region) const;

  SplitConfig config_;
};

}  // namespace lidar_dock_detector
