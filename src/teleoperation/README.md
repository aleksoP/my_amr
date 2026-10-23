# Teleoperation prototype

This prototype is for simulation and development. It publishes
`geometry_msgs/msg/TwistStamped`; it does not command the drivetrain. It does
not provide a physical deadman, motor-side watchdog, motion arbitration, or a
safety-rated stop. Do not use it on a physical robot without those systems.

Build and start the manager from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleoperation
source install/setup.bash
ros2 launch teleoperation teleoperation.launch.py
```

The keyboard node reads a **local Linux evdev device**, so it can track multiple
held keys and releases. Run it in another sourced terminal:

```bash
ros2 run teleoperation keyboard_input --ros-args \
  --params-file "$(ros2 pkg prefix --share teleoperation)/config/teleoperation.yaml"
```

It searches `/dev/input/by-id/` and `/dev/input/by-path/` and reads every
detected keyboard at startup, listing the paths it uses. Newly connected
keyboards are not added until the node is restarted. To use just one keyboard,
or if discovery does not find your device, pass its path explicitly:

```bash
ros2 run teleoperation keyboard_input --ros-args \
  --params-file "$(ros2 pkg prefix --share teleoperation)/config/teleoperation.yaml" \
  -p device:=/dev/input/by-id/YOUR_KEYBOARD-event-kbd
```

The process needs read permission for every discovered device. If any device
cannot be opened, the node exits and identifies it; use the explicit device
option to limit which device is opened. Access can expose all keystrokes on
those devices. This input is local to the machine running the node and does not
read keystrokes from an SSH terminal. If any opened keyboard disconnects or
loses events, the node stops publishing and must be restarted.

**The node reads every opened keyboard globally, regardless of which window has
focus.** Keep it running only while intentionally operating the prototype. It
starts only when the movement and stop keys are released on all opened devices.
The node disables echo in its own terminal and discards queued terminal input
on exit. It restores the terminal settings on normal exit and Ctrl+C; after an
uncatchable termination, run `stty sane` if the terminal needs restoring. Other
focused windows can still receive the keystrokes.

- Hold `w`/`s` for forward/reverse and `a`/`d` to turn. Hold one from each
  pair for combined motion. Opposing keys cancel on their axis.
- Press `+` (or `=`) and `-` to increase or decrease both speeds in 10% steps.
  Startup is 20% by default; each adjustment prints the new percentage. Autorepeat is
  ignored, so release and press again for another step.
- Space requests zero. Release space **and all movement keys** before moving
  again. Press `q` to quit.

The keyboard's full-scale speeds are 0.5 m/s forward, 0.3 m/s reverse, and
0.8 rad/s angular. Set keyboard parameters, including `max_linear`, `max_reverse`, `max_angular`,
`initial_speed_scale`, and `speed_scale_step`, in `config/teleoperation.yaml`. The manager applies its own limits
from `config/teleoperation.yaml`; both sets of limits must allow the desired
speed. Both nodes read parameters at startup; rebuild after changing the source
YAML so the installed copy is updated, then restart both nodes. The manager outputs zero if it hasn't received input for 0.25 seconds;
its 20 Hz output may add up to one timer period. Its timeout uses monotonic
receipt time, so a delayed message can be accepted as new.

To inspect commands, use a third sourced terminal:

```bash
ros2 topic echo /cmd_vel_teleop
```

Try combined movement and release each key separately; the corresponding axis
should return to zero. Stop the keyboard process and confirm output becomes
zero. If the manager stops, publication stops altogether, so any future
downstream consumer must independently stop on command loss. A final zero
message on exit or input error is best effort.
