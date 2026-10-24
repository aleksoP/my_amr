#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/types.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

namespace lidar_dock_detector
{

class ScanProcessor
{
public:
  explicit ScanProcessor(RoiConfig config);

  std::vector<Point2D> process(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & transform) const;

private:
  RoiConfig config_;
};

}  // namespace lidar_dock_detector
