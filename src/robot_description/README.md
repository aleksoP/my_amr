# robot_description

Robot description package for my_amr. Contains the URDF model and a launch file that starts `robot_state_publisher` for use on the **real robot** (no simulation artifacts).

## Structure

```
robot_description/
├── urdf/
│   └── my_amr.urdf.xacro     # Robot kinematic model — no Gazebo plugins
├── meshes/
│   ├── body.stl
│   ├── wheel.stl
│   ├── rplidar_s2.stl
│   └── ellipse_micro_ahrs.stl
├── launch/
│   └── robot_description.launch.py
└── rviz2/                     # RViz configs
```

## What this publishes

| Topic | Type | Description |
|---|---|---|
| `/robot_description` | `std_msgs/String` | Full URDF, latched — any node that subscribes gets it immediately |
| `/tf_static` | `tf2_msgs/TFMessage` | All fixed transforms (base_footprint→base_link, lidar, IMU, casters) |
| `/tf` | `tf2_msgs/TFMessage` | Wheel joint transforms, updated from `/joint_states` |

## Usage

```bash
cd ~/workspace/my_amr
colcon build
source install/setup.bash
ros2 launch robot_description robot_description.launch.py
```

## Relationship to simulation

The URDF here is kinematically identical to `simulation_ws/src/my_amr_sim/my_amr.urdf.xacro` but contains **no Gazebo plugins or sensor definitions**. Keep the two in sync when you change joint positions or link geometry.

Meshes live here (the source of truth) and are referenced by `my_amr_sim` via the `robot_description` package — so there is only one copy.

## Frame tree

```
base_footprint
└── base_link (z = +82.6 mm)
    ├── left_wheel_joint       → left_wheel
    ├── right_wheel_joint      → right_wheel
    ├── front_left_caster_joint  → front_left_caster
    ├── rear_left_caster_joint   → rear_left_caster
    ├── front_right_caster_joint → front_right_caster
    ├── rear_right_caster_joint  → rear_right_caster
    ├── lidar_joint            → lidar_link  (scan plane)
    └── imu_joint              → imu_link
```
