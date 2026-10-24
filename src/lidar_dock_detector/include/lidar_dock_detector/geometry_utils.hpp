#pragma once

#include "lidar_dock_detector/types.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace lidar_dock_detector
{

constexpr double kPi = 3.14159265358979323846;

struct RigidTransform2D
{
  Eigen::Matrix2d rotation {Eigen::Matrix2d::Identity()};
  Eigen::Vector2d translation {Eigen::Vector2d::Zero()};
};

inline double degreesToRadians(const double degrees)
{
  return degrees * kPi / 180.0;
}

inline double radiansToDegrees(const double radians)
{
  return radians * 180.0 / kPi;
}

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double shortestAngularDistance(const double from, const double to)
{
  return normalizeAngle(to - from);
}

inline double distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

inline Eigen::Vector2d toEigen(const Point2D & point)
{
  return Eigen::Vector2d(point.x, point.y);
}

inline Point2D toPoint2D(const Eigen::Vector2d & vector, const int scan_index = 0)
{
  Point2D point;
  point.x = vector.x();
  point.y = vector.y();
  point.scan_index = scan_index;
  return point;
}

inline Eigen::Vector2d rotate(const Eigen::Vector2d & vector, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * vector.x() - s * vector.y(), s * vector.x() + c * vector.y());
}

inline Eigen::Vector2d applyTransform(const RigidTransform2D & transform, const Eigen::Vector2d & point)
{
  return transform.rotation * point + transform.translation;
}

inline Pose2D transformToPose(const RigidTransform2D & transform, const Eigen::Vector2d & origin = Eigen::Vector2d::Zero())
{
  const Eigen::Vector2d translated_origin = applyTransform(transform, origin);
  Pose2D pose;
  pose.x = translated_origin.x();
  pose.y = translated_origin.y();
  pose.yaw = std::atan2(transform.rotation(1, 0), transform.rotation(0, 0));
  pose.valid = true;
  return pose;
}

inline RigidTransform2D poseToTransform(const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);

  RigidTransform2D transform;
  transform.rotation << c, -s,
    s, c;
  transform.translation = Eigen::Vector2d(pose.x, pose.y);
  return transform;
}

inline Pose2D composePose(const Pose2D & base, const Pose2D & offset)
{
  Pose2D composed;
  const Eigen::Vector2d offset_xy = rotate(Eigen::Vector2d(offset.x, offset.y), base.yaw);
  composed.x = base.x + offset_xy.x();
  composed.y = base.y + offset_xy.y();
  composed.yaw = normalizeAngle(base.yaw + offset.yaw);
  composed.valid = base.valid && offset.valid;
  return composed;
}

inline Pose2D lowPassPose(const Pose2D & current, const Pose2D & measurement, const double alpha_pos, const double alpha_yaw)
{
  Pose2D filtered = current;
  filtered.x = current.x + alpha_pos * (measurement.x - current.x);
  filtered.y = current.y + alpha_pos * (measurement.y - current.y);
  filtered.yaw = normalizeAngle(current.yaw + alpha_yaw * shortestAngularDistance(current.yaw, measurement.yaw));
  filtered.valid = measurement.valid;
  return filtered;
}

inline std::optional<RigidTransform2D> estimateRigidTransform(
  const std::vector<Eigen::Vector2d> & source,
  const std::vector<Eigen::Vector2d> & target)
{
  if (source.size() != target.size() || source.size() < 2U) {
    return std::nullopt;
  }

  Eigen::Vector2d source_centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d target_centroid = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < source.size(); ++i) {
    source_centroid += source[i];
    target_centroid += target[i];
  }
  source_centroid /= static_cast<double>(source.size());
  target_centroid /= static_cast<double>(target.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (std::size_t i = 0; i < source.size(); ++i) {
    covariance += (source[i] - source_centroid) * (target[i] - target_centroid).transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix2d correction = Eigen::Matrix2d::Identity();
    correction(1, 1) = -1.0;
    rotation = svd.matrixV() * correction * svd.matrixU().transpose();
  }

  RigidTransform2D transform;
  transform.rotation = rotation;
  transform.translation = target_centroid - rotation * source_centroid;
  return transform;
}

inline double pointLineDistance(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & line_point,
  const Eigen::Vector2d & line_direction)
{
  const Eigen::Vector2d delta = point - line_point;
  const Eigen::Vector2d normal(-line_direction.y(), line_direction.x());
  return std::abs(delta.dot(normal));
}

inline double signedTurnAngle(const Eigen::Vector2d & from, const Eigen::Vector2d & to)
{
  return normalizeAngle(std::atan2(from.x() * to.y() - from.y() * to.x(), from.dot(to)));
}

inline double rmse(const std::vector<Eigen::Vector2d> & a, const std::vector<Eigen::Vector2d> & b)
{
  if (a.size() != b.size() || a.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double total = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    total += (a[i] - b[i]).squaredNorm();
  }
  return std::sqrt(total / static_cast<double>(a.size()));
}

}  // namespace lidar_dock_detector
