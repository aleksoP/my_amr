#ifndef TELEOPERATION__KEYBOARD_STATE_HPP_
#define TELEOPERATION__KEYBOARD_STATE_HPP_

#include <algorithm>
#include <array>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace teleoperation {

// Key state is independent of ROS and the input device so it can be tested.
class KeyboardState {
 public:
  enum class Key { forward, reverse, left, right, stop, faster, slower };

  explicit KeyboardState(double initial_scale = 0.2, double scale_step = 0.1)
  : scale_(initial_scale), scale_step_(scale_step) {
    if (!std::isfinite(scale_) || scale_ < 0.0 || scale_ > 1.0 ||
        !std::isfinite(scale_step_) || scale_step_ <= 0.0 || scale_step_ > 1.0) {
      throw std::invalid_argument("initial_speed_scale must be between 0 and 1 and speed_scale_step must be greater than 0 and at most 1");
    }
  }

  void update(Key key, bool pressed) {
    switch (key) {
      case Key::forward: forward_ = pressed; break;
      case Key::reverse: reverse_ = pressed; break;
      case Key::left: left_ = pressed; break;
      case Key::right: right_ = pressed; break;
      case Key::stop:
        stop_ = pressed;
        if (pressed) inhibited_ = true;
        break;
      case Key::faster:
        if (pressed) scale_ = std::min(1.0, scale_ + scale_step_);
        break;
      case Key::slower:
        if (pressed) scale_ = std::max(0.0, scale_ - scale_step_);
        break;
    }
    if (!stop_ && !forward_ && !reverse_ && !left_ && !right_) inhibited_ = false;
  }

  double linear(double forward_limit, double reverse_limit) const {
    if (inhibited_ || stop_ || forward_ == reverse_) return 0.0;
    return forward_ ? scale_ * forward_limit : -scale_ * reverse_limit;
  }
  double angular(double limit) const {
    if (inhibited_ || stop_ || left_ == right_) return 0.0;
    return (left_ ? 1.0 : -1.0) * scale_ * limit;
  }
  double scale() const { return scale_; }

 private:
  bool forward_{false}, reverse_{false}, left_{false}, right_{false};
  bool stop_{false}, inhibited_{false};
  double scale_;
  double scale_step_;
};

// Aggregate held keys across keyboards without letting one device's release
// cancel a key still held on another device.
class MultiKeyboardState {
 public:
  explicit MultiKeyboardState(size_t devices, double initial_scale = 0.2, double scale_step = 0.1)
  : held_(devices), state_(initial_scale, scale_step) {}

  void update(size_t device, KeyboardState::Key key, bool pressed) {
    using Key = KeyboardState::Key;
    if (key == Key::faster || key == Key::slower) {
      state_.update(key, pressed);
      return;
    }
    const size_t index = static_cast<size_t>(key);
    bool was_down = false;
    for (const auto & keys : held_) was_down |= keys[index];
    held_.at(device)[index] = pressed;
    bool is_down = false;
    for (const auto & keys : held_) is_down |= keys[index];
    if (was_down != is_down) state_.update(key, is_down);
  }
  double linear(double forward_limit, double reverse_limit) const {
    return state_.linear(forward_limit, reverse_limit);
  }
  double angular(double limit) const { return state_.angular(limit); }
  double scale() const { return state_.scale(); }

 private:
  std::vector<std::array<bool, 5>> held_;
  KeyboardState state_;
};
}  // namespace teleoperation
#endif  // TELEOPERATION__KEYBOARD_STATE_HPP_
