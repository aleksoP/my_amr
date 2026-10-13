## Packages included


| Package name | Fucntion | Topics/Services published | Implementation |
| --- | --- | --- | --- |
| `robot_description` | The place where the robot geometry is defined and the whole stationary TF is defined | `/tf_static`, `/robot_description`, `/tf` | URDF/Xacro, YAML |
| `robot_bringup` | The launch place for all main packages, namespace definitions and configurations | - | Python launch files |
| `mission_orchestrator` | Sole authority for movement tasks and missions | `/mission_state` | Python |
| `base_kinematics` | FK, IK, drivetrain hardware communication, raw wheel odometry | sub`/cmd_vel` `/wheel/odom` `/joint_states` `diagnostics`| C++ |
| `state_estimation` | EKF sensor fusion using robot_localization | Pub: `odom`, `tf` | YAML, Python launch |
| `mcu_driver` | Main MCU communication, peripheral control, MCU telemetry | Battery state, I/O, MCU status, diagnostics (TBD) | C++ |
| `system_manager` | Process health, diagnostics, log/run metadata, controlled restart/update hooks | `system/status`, `diagnostics`, management services | Python |
| `robot_interfaces` | Custom ROS interfaces | - | ROS IDL |
| `robot_config` | Load, validate, and resolve robot/model configuration | Internal library/API; parameters distributed by bringup | Python | 
| `safety_supervisor` | Software fault supervision, movement inhibition, interlocks | `safety/state`, motion inhibit/permit interface | C++ |
| `navigation` | Nav2 integration, planning, localization, docking | Nav2 actions, paths, costmaps, velocity commands | YAML, Python launch |


## Optional 

| Package name | Fucntion | Topics/Services published | Implementation |
| --- | --- | --- | --- |
| `rcs_adapter` | MQTT connection with customer RCS; translates external tasks into mission requests and reports status | Mission requests via action/service clients, external MQTT messages | Python |
| `lidar_dock_detector` | Detects docking geometry using LiDAR | `/dock_pose/lidar` | C++ |
| `apriltag_detector` | Detects visible apritags and outputs their info | `/tag_detections` | C++ or upstream |
|  |  |  |

