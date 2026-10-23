# Robot Navigation

`robot_navigation` owns the robot-facing navigation configuration and the
Nav2 entry point. It uses adapted Nav2 Jazzy bringup and navigation launch
files to route velocity commands through the shared mux.

## Package layout

```text
robot_navigation/
├── config/
│   ├── nav2.yaml          # Complete upstream Nav2 Jazzy default parameters
│   └── ekf.yaml           # robot_localization EKF template: wheel odometry + IMU
└── launch/
    ├── navigation.launch.py
    ├── bringup_mux.launch.py
    └── navigation_mux.launch.py
```

## Starting the navigation stack

The velocity smoother receives `/cmd_vel_mux`, so navigation alone does not
provide its command input; start the mux through `robot_bringup` autonomy for
the complete prototype path.

Build and source the workspace, then start Nav2 with the map to localize
against:

```bash
colcon build --packages-select robot_navigation
source install/setup.bash

ros2 launch robot_navigation navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

`navigation.launch.py` uses `config/nav2.yaml` by default. Override it only
when testing another complete Nav2 parameter file:

```bash
ros2 launch robot_navigation navigation.launch.py \
  map:=/absolute/path/to/map.yaml \
  params_file:=/absolute/path/to/alternative_nav2.yaml
```

## What the Nav2 launch starts

The default launch arguments are `slam:=false`, `use_localization:=true`,
`autostart:=true`, and `use_composition:=true`. Nav2 therefore starts one
component container containing:

```text
map_server                     AMCL
lifecycle_manager_localization controller_server
smoother_server                planner_server
route_server                   behavior_server
velocity_smoother              collision_monitor
bt_navigator                   waypoint_follower
docking_server                 lifecycle_manager_navigation
```

The two lifecycle managers configure and activate their respective Nav2 nodes.
Set `use_composition:=false` to run the same nodes as separate processes while
debugging.

## Configuration ownership

### `config/nav2.yaml`

This is based on the Nav2 Jazzy default parameter file, with stamped velocity
commands enabled for the motion pipeline. It
contains the parameters for AMCL, map server, planners, controller, costmaps,
behavior server, BT navigator, collision monitor, docking, and related Nav2
processes. It is a baseline, not a tuned robot configuration.

### `config/ekf.yaml`

This is based on the `robot_localization` template installed with ROS Jazzy.
Only two measurement inputs are retained:

```text
odom0  wheel odometry
imu0   IMU
```

Their topic values are still the upstream placeholders (`example/odom` and
`example/imu`). Replace them with the actual topics published by this robot's
drivers before starting an EKF node. This package does not yet include an EKF
launch file, so the configuration is installed but not started by
`navigation.launch.py`.

## Required platform interfaces

Nav2 does not start hardware drivers, robot description, sensor transforms, or
the EKF. Start the platform stack first. For map-based navigation, the complete
system must provide:

```text
/scan                         LaserScan input for AMCL and obstacle layers
odom → base_link              Local continuous transform, normally from the EKF
base_link → sensor frames     Fixed transforms from robot description
map → odom                    Produced by AMCL after Nav2 starts
```

The resulting transform chain is:

```text
map → odom → base_link → sensor frames
```

## Dependencies

The package requires `nav2_bringup`. Install the Nav2 Jazzy packages before
launching:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```
