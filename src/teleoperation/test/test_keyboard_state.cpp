#include <cassert>
#include <cmath>

#include "teleoperation/keyboard_state.hpp"

using teleoperation::KeyboardState;
using Key = KeyboardState::Key;

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

int main() {
  KeyboardState configured(0.4, 0.25);
  configured.update(Key::forward, true);
  configured.update(Key::faster, true);
  assert(near(configured.linear(0.5, 0.3), 0.325));
  bool rejected = false;
  try { KeyboardState invalid(1.1, 0.1); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  rejected = false;
  try { KeyboardState invalid(0.2, 0.0); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  KeyboardState s;
  assert(near(s.linear(0.5, 0.3), 0));
  s.update(Key::forward, true);
  s.update(Key::left, true);
  assert(near(s.linear(0.5, 0.3), 0.1));
  assert(near(s.angular(0.8), 0.16));
  s.update(Key::faster, true);
  assert(near(s.linear(0.5, 0.3), 0.15));
  assert(near(s.angular(0.8), 0.24));
  s.update(Key::left, false);
  assert(near(s.angular(0.8), 0));
  assert(near(s.linear(0.5, 0.3), 0.15));
  s.update(Key::reverse, true);
  assert(near(s.linear(0.5, 0.3), 0));
  s.update(Key::forward, false);
  assert(near(s.linear(0.5, 0.3), -0.09));
  s.update(Key::stop, true);
  assert(near(s.linear(0.5, 0.3), 0));
  s.update(Key::stop, false);
  assert(near(s.linear(0.5, 0.3), 0));  // Must release movement before rearming.
  s.update(Key::reverse, false);
  s.update(Key::reverse, true);
  assert(near(s.linear(0.5, 0.3), -0.09));
  for (int i = 0; i < 20; ++i) s.update(Key::faster, true);
  assert(near(s.scale(), 1.0));
  for (int i = 0; i < 20; ++i) s.update(Key::slower, true);
  assert(near(s.scale(), 0.0));

  teleoperation::MultiKeyboardState keyboards(2);
  keyboards.update(0, Key::forward, true);
  keyboards.update(1, Key::forward, true);
  keyboards.update(0, Key::forward, false);
  keyboards.update(0, Key::left, true);
  assert(near(keyboards.linear(0.5, 0.3), 0.1));
  assert(near(keyboards.angular(0.8), 0.16));
  keyboards.update(0, Key::stop, true);
  keyboards.update(1, Key::stop, true);
  keyboards.update(0, Key::stop, false);
  assert(near(keyboards.linear(0.5, 0.3), 0));
  keyboards.update(1, Key::stop, false);
  assert(near(keyboards.angular(0.8), 0));
  keyboards.update(1, Key::forward, false);
  keyboards.update(0, Key::left, false);
  keyboards.update(0, Key::forward, true);
  assert(near(keyboards.linear(0.5, 0.3), 0.1));
}
