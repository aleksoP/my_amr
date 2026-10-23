## Dependency structure

robot_bringup.launch.py
│
├── platform.launch.py
│     ├── Motors
│     ├── MCU
│     ├── LiDAR
│     ├── IMU
│     ├── Cameras
│     ├── Odometry / EKF
│     ├── TF tree
│     └── Safety / diagnostics
│
├── autonomy.launch.py
│     ├── Map server
│     ├── AMCL / SLAM
│     ├── Nav2
│     ├── Costmaps
│     └── Docking perception/control
│
└── application.launch.py
      ├── Mission manager
      ├── BMC adapter
      ├── WES gateway
      ├── MQTT
      ├── Panel integration
      └── Fleet / task API

## Responsibilies

-- platform (replacable with simulation)

Motor driver / motor node
MCU serial communication
IMU driver
LiDAR driver
Camera drivers
IMU filter / robot_localization EKF
static_transform_publishers
Safety monitor

-- autonomy

AMCL or SLAM
Map server
Nav2 lifecycle manager
Planner server
Controller server
Behavior server
BT navigator
Waypoint follower
Costmaps
Recovery behaviors
Dock detector
Docking controller
Obstacle perception / local obstacle layer

-- application

robot_mission_manager
wes_gateway
MQTT bridge
REST API integration
Panel / Android communication
Fleet management bridge
Task dispatch interface
Logging / mission event recorder


## Velocity command path

`autonomy.launch.py` starts the Nav2 stack and `twist_mux`. Velocity commands use
`geometry_msgs/msg/TwistStamped`:

```text
Nav2 controller / behaviors / docking → /cmd_vel_nav ───┐
teleoperation manager                 → /cmd_vel_teleop ─┤
                                                        ↓
                                              twist_mux → /cmd_vel_mux
                                                        ↓
                                  Nav2 velocity smoother → /cmd_vel_smoothed
                                                        ↓
                                    Nav2 collision monitor → /cmd_vel
```

These topic names are for the default single-robot setup. Nav2's output
`cmd_vel` and `cmd_vel_smoothed` are relative to its namespace; the mux inputs
and output are absolute, so this configuration does not isolate multiple
robots by namespace.

The mux selects by priority and freshness, not by operating mode. Teleoperation
has higher priority and its manager continuously publishes zero when idle, so
it can block navigation while running. The teleoperation manager and keyboard
are launched separately. There is no mode-controlled authority switch yet.

The `cmd_vel` output is not connected to a motor in this workspace. The collision
monitor is a software feature, not a safety-rated stop. A downstream command
watchdog and engineered protective functions are still required before physical
operation. A collision-monitor stop occurs after the smoother; normal source
stops still go through smoothing.

The navigation package maintains small adaptations of the installed Nav2 Jazzy
bringup and navigation launch files so the smoother subscribes to the mux output
and docking publishes to the mux's navigation input. Review them when updating
Nav2. Standalone `robot_navigation` now also requires the mux to supply the
smoother; use the autonomy launch for the complete prototype path.
