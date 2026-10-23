#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

class TeleoperationManager : public rclcpp::Node {
 public:
  TeleoperationManager() : Node("teleoperation_manager") {
    timeout_ = declare_parameter<double>("command_timeout", 0.25);
    max_linear_ = declare_parameter<double>("max_linear", 0.1);
    max_reverse_ = declare_parameter<double>("max_reverse", 0.1);
    max_angular_ = declare_parameter<double>("max_angular", 0.2);
    if (!std::isfinite(timeout_) || timeout_ <= 0.0 ||
        !std::isfinite(max_linear_) || max_linear_ < 0.0 ||
        !std::isfinite(max_reverse_) || max_reverse_ < 0.0 ||
        !std::isfinite(max_angular_) || max_angular_ < 0.0) {
      throw std::invalid_argument("timeout must be positive and velocity limits must be nonnegative finite values");
    }
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_teleop", rclcpp::QoS(1));
    subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/teleop/input/keyboard", rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        if (!std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.angular.z)) {
          command_ = geometry_msgs::msg::Twist{};
          has_command_ = false;
          RCLCPP_WARN(get_logger(), "Rejected non-finite command");
          return;
        }
        command_ = geometry_msgs::msg::Twist{};
        command_.linear.x = std::clamp(msg->twist.linear.x, -max_reverse_, max_linear_);
        command_.angular.z = std::clamp(msg->twist.angular.z, -max_angular_, max_angular_);
        last_command_ = SteadyClock::now();
        has_command_ = true;
      });
    timer_ = create_wall_timer(50ms, [this]() { publish_command(); });
  }

 private:
  void publish_command() {
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    if (has_command_ && std::chrono::duration<double>(SteadyClock::now() - last_command_).count() < timeout_) {
      output.twist = command_;
    }
    publisher_->publish(output);
  }
  double timeout_{};
  double max_linear_{};
  double max_reverse_{};
  double max_angular_{};
  bool has_command_{false};
  SteadyClock::time_point last_command_{};
  geometry_msgs::msg::Twist command_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TeleoperationManager>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("teleoperation_manager"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
