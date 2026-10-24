#include "lidar_dock_detector/region_splitter.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <cmath>
#include <utility>

namespace lidar_dock_detector
{

RegionSplitter::RegionSplitter(SplitConfig config)
: config_(std::move(config))
{
}

std::vector<Region> RegionSplitter::split(const std::vector<Point2D> & points) const
{
  std::vector<Region> regions;
  if (points.empty()) {
    return regions;
  }

  Region current_region;
  current_region.points.push_back(points.front());

  for (std::size_t i = 1; i < points.size(); ++i) {
    const double gap = std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    if (gap > config_.point_gap) {
      current_region.span = std::hypot(
        current_region.points.back().x - current_region.points.front().x,
        current_region.points.back().y - current_region.points.front().y);
      if (isRegionValid(current_region)) {
        regions.push_back(current_region);
      }
      current_region = Region {};
    }

    current_region.points.push_back(points[i]);
  }

  current_region.span = std::hypot(
    current_region.points.back().x - current_region.points.front().x,
    current_region.points.back().y - current_region.points.front().y);
  if (isRegionValid(current_region)) {
    regions.push_back(current_region);
  }

  return regions;
}

bool RegionSplitter::isRegionValid(const Region & region) const
{
  if (static_cast<int>(region.points.size()) < config_.min_region_points) {
    return false;
  }

  if (region.span < config_.min_region_span) {
    return false;
  }

  if (region.span > config_.max_region_span) {
    return false;
  }

  return true;
}

}  // namespace lidar_dock_detector
