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