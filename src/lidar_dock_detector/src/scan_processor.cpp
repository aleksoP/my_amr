#include "lidar_dock_detector/scan_processor.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>

namespace lidar_dock_detector
{

ScanProcessor::ScanProcessor(RoiConfig config)
: config_(std::move(config))
{
}

std::vector<Point2D> ScanProcessor::process(
  const sensor_msgs::msg::LaserScan & scan,
  const geometry_msgs::msg::TransformStamped & transform) const
{
  std::vector<Point2D> roi_points;
  roi_points.reserve(scan.ranges.size());

  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  tf2::Transform tf_transform;
  tf_transform.setOrigin(tf2::Vector3(translation.x, translation.y, translation.z));
  tf_transform.setRotation(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < config_.min_range || range > config_.max_range) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    tf2::Vector3 point_in_scan(range * std::cos(angle), range * std::sin(angle), 0.0);
    const tf2::Vector3 point_in_output = tf_transform * point_in_scan;

    if (
      point_in_output.x() < config_.min_x || point_in_output.x() > config_.max_x ||
      point_in_output.y() < config_.min_y || point_in_output.y() > config_.max_y)
    {
      continue;
    }

    Point2D point;
    point.x = point_in_output.x();
    point.y = point_in_output.y();
    point.scan_index = static_cast<int>(i);
    roi_points.push_back(point);
  }

  return roi_points;
}

}  // namespace lidar_dock_detector
