#pragma once

#include "lidar_dock_detector/config.hpp"
#include "lidar_dock_detector/model.hpp"
#include "lidar_dock_detector/types.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <string>

namespace lidar_dock_detector
{

class Nav2PosePublisher
{
public:
  Nav2PosePublisher(rclcpp::Node & node, const DetectorConfig & config);

  void publish(
    const builtin_interfaces::msg::Time & stamp,
    const FilterOutput & filter_output,
    const DetectionDebugFrame & debug_frame,
    const std::optional<Pose2D> & pose_topic_pose = std::nullopt);
  void clearDebug(const builtin_interfaces::msg::Time & stamp);

private:
  visualization_msgs::msg::Marker makeArrowMarker(
    int id,
    const std::string & ns,
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & pose,
    double r,
    double g,
    double b,
    double scale) const;
  visualization_msgs::msg::Marker makePointsMarker(
    int id,
    const std::string & ns,
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<Point2D> & points,
    double r,
    double g,
    double b,
    double scale) const;
  visualization_msgs::msg::Marker makeLineListMarker(
    int id,
    const std::string & ns,
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<LineSegment> & lines,
    double r,
    double g,
    double b,
    double width) const;
  visualization_msgs::msg::Marker makeTextMarker(
    int id,
    const std::string & ns,
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & pose,
    const std::string & text) const;
  std::vector<LineSegment> transformedModelLines(const Pose2D & pose) const;
  void publishPose(const builtin_interfaces::msg::Time & stamp, const Pose2D & pose) const;
  void publishMarkerTransform(
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & pose,
    const std::string & frame_id) const;
  static std::string stateToString(DetectionState state);

  DetectorConfig config_;
  LandmarkModel model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

}  // namespace lidar_dock_detector
