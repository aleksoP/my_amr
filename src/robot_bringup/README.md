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