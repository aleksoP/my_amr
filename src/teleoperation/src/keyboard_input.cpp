#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "teleoperation/keyboard_state.hpp"

using namespace std::chrono_literals;
using teleoperation::KeyboardState;

class TerminalEcho {
 public:
  TerminalEcho() {
    if (!isatty(STDIN_FILENO)) return;
    if (tcgetattr(STDIN_FILENO, &original_) < 0) {
      throw std::runtime_error(std::string("Cannot read terminal settings: ") + std::strerror(errno));
    }
    auto settings = original_;
    settings.c_lflag &= ~ECHO;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) < 0) {
      throw std::runtime_error(std::string("Cannot disable terminal echo: ") + std::strerror(errno));
    }
    active_ = true;
  }
  ~TerminalEcho() {
    if (active_) {
      // Keys are handled through evdev; discard their terminal input before
      // handing control back to the shell.
      if (tcflush(STDIN_FILENO, TCIFLUSH) < 0) std::perror("Cannot clear terminal input");
      if (tcsetattr(STDIN_FILENO, TCSANOW, &original_) < 0) {
        std::perror("Cannot restore terminal settings");
      }
    }
  }
  TerminalEcho(const TerminalEcho &) = delete;
  TerminalEcho & operator=(const TerminalEcho &) = delete;

 private:
  termios original_{};
  bool active_{false};
};

std::vector<std::string> select_keyboards(const std::string & configured_path) {
  if (!configured_path.empty()) return {configured_path};

  namespace fs = std::filesystem;
  std::vector<std::string> candidates;
  std::vector<fs::path> targets;
  // Prefer stable by-id names, and include built-in devices from by-path.
  for (const auto * directory : {"/dev/input/by-id", "/dev/input/by-path"}) {
    std::error_code ec;
    fs::directory_iterator it(directory, ec), end;
    std::vector<fs::path> paths;
    for (; !ec && it != end; it.increment(ec)) {
      const auto name = it->path().filename().string();
      if (name.size() >= 10 && name.compare(name.size() - 10, 10, "-event-kbd") == 0) {
        paths.push_back(it->path());
      }
    }
    std::sort(paths.begin(), paths.end());
    for (const auto & path : paths) {
      auto target = fs::canonical(path, ec);
      if (ec) { ec.clear(); continue; }
      if (std::find(targets.begin(), targets.end(), target) != targets.end()) continue;
      targets.push_back(target);
      candidates.push_back(path.string());
    }
  }
  if (candidates.empty()) {
    throw std::runtime_error("No keyboard event devices found under /dev/input/by-id or /dev/input/by-path; set the device parameter explicitly");
  }
  return candidates;
}

class InputDevice {
 public:
  explicit InputDevice(const std::string & path) {
    if (path.empty()) throw std::runtime_error("Set the device parameter to a local keyboard event device");
    fd_ = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("Cannot open " + path + ": " + std::strerror(errno));
    // Refuse startup with motion keys already down: begin from a known neutral state.
    unsigned long keys[(KEY_MAX + sizeof(unsigned long) * 8) / (sizeof(unsigned long) * 8)]{};
    if (ioctl(fd_, EVIOCGKEY(sizeof(keys)), keys) < 0) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("Cannot read keyboard state from " + path);
    }
    for (int key : {KEY_W, KEY_S, KEY_A, KEY_D, KEY_SPACE}) {
      if (keys[key / (sizeof(unsigned long) * 8)] & (1UL << (key % (sizeof(unsigned long) * 8)))) {
        close(fd_);
        fd_ = -1;
        throw std::runtime_error("Release movement and stop keys before starting keyboard input");
      }
    }
  }
  ~InputDevice() { if (fd_ >= 0) close(fd_); }
  InputDevice(const InputDevice &) = delete;
  InputDevice & operator=(const InputDevice &) = delete;
  int fd() const { return fd_; }
 private:
  int fd_{-1};
};

class KeyboardInput : public rclcpp::Node {
 public:
  KeyboardInput() : Node("keyboard_input") {
    const auto paths = select_keyboards(declare_parameter<std::string>("device", ""));
    for (const auto & path : paths) {
      devices_.push_back(std::make_unique<InputDevice>(path));
      RCLCPP_INFO(get_logger(), "Using keyboard: %s", path.c_str());
    }
    const auto initial_scale = declare_parameter<double>("initial_speed_scale", 0.2);
    const auto scale_step = declare_parameter<double>("speed_scale_step", 0.1);
    state_ = std::make_unique<teleoperation::MultiKeyboardState>(devices_.size(), initial_scale, scale_step);
    max_linear_ = declare_parameter<double>("max_linear", 0.5);
    max_reverse_ = declare_parameter<double>("max_reverse", 0.3);
    max_angular_ = declare_parameter<double>("max_angular", 0.8);
    if (!std::isfinite(max_linear_) || max_linear_ < 0 || !std::isfinite(max_reverse_) ||
        max_reverse_ < 0 || !std::isfinite(max_angular_) || max_angular_ < 0) {
      throw std::invalid_argument("Keyboard speed limits must be finite and nonnegative");
    }
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/teleop/input/keyboard", rclcpp::QoS(1));
    timer_ = create_wall_timer(20ms, [this]() { poll_device(); });
    RCLCPP_INFO(get_logger(), "Keyboard active globally. W/S: move, A/D: turn, +/-: speed, space: stop, Q: quit. Speed: %.0f%%", state_->scale() * 100.0);
  }

 private:
  void publish(bool stop = false) {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    if (!stop) {
      command.twist.linear.x = state_->linear(max_linear_, max_reverse_);
      command.twist.angular.z = state_->angular(max_angular_);
    }
    publisher_->publish(command);
  }
  void poll_device() {
    for (size_t i = 0; i < devices_.size(); ++i) {
      if (!poll_one_device(i)) return;
    }
    publish();
  }
  bool poll_one_device(size_t index) {
    input_event event{};
    for (;;) {
      const ssize_t count = read(devices_[index]->fd(), &event, sizeof(event));
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (count != sizeof(event)) {
        RCLCPP_ERROR(get_logger(), "Keyboard disconnected or read failed; stopping input");
        publish(true);
        rclcpp::shutdown();
        return false;
      }
      if (event.type == EV_SYN && event.code == SYN_DROPPED) {
        RCLCPP_ERROR(get_logger(), "Keyboard events lost; stopping input. Restart after releasing keys.");
        publish(true);
        rclcpp::shutdown();
        return false;
      }
      if (event.type != EV_KEY || event.value == 2) continue;
      if (event.code == KEY_Q && event.value == 1) {
        publish(true);
        rclcpp::shutdown();
        return false;
      }
      KeyboardState::Key key;
      switch (event.code) {
        case KEY_W: key = KeyboardState::Key::forward; break;
        case KEY_S: key = KeyboardState::Key::reverse; break;
        case KEY_A: key = KeyboardState::Key::left; break;
        case KEY_D: key = KeyboardState::Key::right; break;
        case KEY_SPACE: key = KeyboardState::Key::stop; break;
        case KEY_EQUAL: case KEY_KPPLUS: key = KeyboardState::Key::faster; break;
        case KEY_MINUS: case KEY_KPMINUS: key = KeyboardState::Key::slower; break;
        default: continue;
      }
      state_->update(index, key, event.value == 1);
      if (event.value == 1 && (key == KeyboardState::Key::faster || key == KeyboardState::Key::slower)) {
        RCLCPP_INFO(get_logger(), "Speed: %.0f%%", state_->scale() * 100.0);
      }
    }
    return true;
  }
  std::vector<std::unique_ptr<InputDevice>> devices_;
  std::unique_ptr<teleoperation::MultiKeyboardState> state_;
  double max_linear_{}, max_reverse_{}, max_angular_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    TerminalEcho terminal_echo;
    rclcpp::spin(std::make_shared<KeyboardInput>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
