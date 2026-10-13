# Legacy workspace package audit

This is an inventory of every unique ROS package found in:

- `/home/aleks/GitHub/roxi_main` — 58 packages
- `/home/aleks/GitHub/roxi/ros_ws` — 64 packages

There are **74 unique package names**: 48 are present in both repositories, 10
exist only in `roxi_main`, and 16 only in `roxi`. Never source both workspaces
in one environment: ROS overlay order would decide which duplicate package is
actually used.

The recommendations are architecture decisions, not a claim that a package is
broken. `Keep` means retain its responsibility in the new project, generally
under a company-neutral name. `Optional` means retain it only with a product
requirement. `Archive` means preserve it as a reference, but do not migrate it
as a production dependency. `Vendor` means pin the upstream project rather
than treating it as company-owned code.

## Shared upstream/vendor packages

These are in both repositories. They should be listed in a `.repos` manifest
or documented fork ledger, not mixed with product ownership.

| Package | Purpose | Decision |
| --- | --- | --- |
| `camera_calibration` | ROS camera-calibration tooling that computes camera intrinsics and writes calibration data. | Vendor; keep only for camera commissioning. |
| `depth_image_proc` | Converts and processes depth images, including depth-to-point-cloud pipelines. | Vendor; retain only if the camera pipeline needs it. |
| `image_pipeline` | Meta-package and shared infrastructure between camera drivers and higher-level vision. | Vendor; depend on required subpackages only. |
| `image_proc` | Image rectification and color/image processing. | Vendor; retain for calibrated monocular or stereo cameras. |
| `image_publisher` | Publishes image files as ROS image topics, mainly useful for tests and demos. | Vendor/development tool; not a production runtime requirement. |
| `image_rotate` | Image rotation/flip processing. | Vendor; use only if a physical camera orientation requires it. |
| `image_view` | GUI visualisation of ROS image topics. | Vendor/development tool; do not deploy on the robot. |
| `laser_filters` | Filter chains for LaserScan data, such as range, angular, and footprint filtering. | Vendor; useful when the LiDAR needs filtering before Nav2. |
| `nav2_bringup` | Standard Nav2 launch/configuration support. | Vendor; use upstream Nav2 and keep robot-specific configuration in `navigation`. |
| `realsense2_camera` | Intel RealSense ROS camera driver. | Vendor; retain only if a RealSense is fitted. |
| `realsense2_camera_msgs` | ROS message/service definitions used by the RealSense driver. | Vendor dependency of `realsense2_camera`; do not own separately. |
| `realsense2_description` | URDF/Xacro models for Intel RealSense cameras. | Vendor; include only when mounting a RealSense in the robot description. |
| `rplidar_ros` | SLAMTEC/RPLIDAR device driver. | Vendor; keep only for the selected LiDAR model. |
| `stereo_image_proc` | Stereo rectification, disparity, and point-cloud processing. | Vendor; retain only for a stereo-camera requirement. |
| `tracetools_image_pipeline` | Tracing/instrumentation support for image-pipeline performance analysis. | Vendor/development tool; omit from the production image unless profiling. |
| `vector_pursuit_controller` | Upstream Nav2 Controller Server plugin implementing Vector Pursuit path tracking and optional collision-aware speed reduction. | Vendor candidate; benchmark it against Nav2 MPPI first. Current navigation configuration uses MPPI, so do not retain both without a measured reason. |
| `witmotion_ros` | Witmotion IMU driver. | Vendor; retain only for the selected Witmotion device. |

## Shared product and prototype packages

### Platform, motion, and system packages

| Package | Purpose | Decision |
| --- | --- | --- |
| `imu_filter` | Preprocesses IMU data with median/notch/EMA filtering and stationary detection before EKF/localization. | Keep the capability if measurements show it improves localisation. Consolidate it with `roxy_filter`; there must be one canonical IMU output. |
| `ing_build_info` | Small C++ build-information/header package used to expose build metadata. | Optional; fold into `system_manager` unless multiple packages genuinely require a shared library. |
| `ing_engineering_mode` | Engineering status/MQTT bridge with a configured external robot ID. | Optional service-tool package; keep out of the production motion path and remove embedded credentials. |
| `ing_log_manager` | Retains, exports, and cleans up structured platform/autonomy/application log runs. | Keep the capability; place it under the future `system_manager` responsibility or retain as a narrowly scoped `log_manager`. |
| `ing_system` | ROS-facing system coordinator for hardware-device readiness, motor-safe requests, systemd restart, and OTA entrypoints. | Keep and evolve into `system_manager` plus `safety_supervisor`; its current separation from the systemd process tree is sound. |
| `joy_controller` | Reads joystick input and maps it for robot use. | Optional development/service tool. It must route through the same safety and command arbitration as Nav2. |
| `roxi_mcu` | Serial MCU integration with sensors, conveyor/lift functions, LEDs, and other external commands. | Keep the hardware-protocol role, but split/rewrite as `mcu_driver` and `io_driver`; do not put robot kinematics here. |
| `roxi_teleop` | Keyboard teleoperation utility. | Optional development/service tool; never a normal production movement authority. |
| `roxy_motor` | Motor-controller communication plus robot kinematics, encoder odometry, calibration tools, and motor services. | Keep its kinematics/odometry role as `base_kinematics`; separate raw motor protocol into `mcu_driver` or a dedicated motor driver. |
| `roxy_filter` | IMU filtering and slip detection utilities. | Keep only validated functions. Merge IMU filtering with `imu_filter`; make slip detection an explicit state-estimation/safety input rather than a loose utility package. |
| `speed_limit_manager` | Arbitrates speed constraints from startup, waypoint, collision, application lock, and reset inputs. | Keep the policy only if it has one clearly defined `cmd_vel` position. It belongs with `safety_supervisor`/navigation policy, not as a competing safety chain. |
| `system_monitor` | Remote Wi-Fi configuration through `nmcli`. Despite its name, it is not a general health monitor. | Optional network-management adapter; rename it and isolate privileges. Do not rely on it for robot health. |

### Navigation, localisation, and docking packages

| Package | Purpose | Decision |
| --- | --- | --- |
| `bt_nodes` | Contains a custom Nav2 Behaviour Tree condition (`final_goal_occupied`). | Optional Nav2 extension; keep only if an acceptance test proves the exact task behaviour is needed. |
| `detection_test` | A Python detection node, currently launched by autonomy but with placeholder metadata and a misleading name. | Investigate before migration. Either rename/rebuild it as a defined perception capability or archive it. |
| `elevator_manager` | Switches Nav2 map/costmaps and AMCL initial pose after an elevator floor transition. | Optional `elevator_adapter`; useful for multi-floor products, but remove fixed paths and couple it to an explicit elevator mission state. |
| `lidar_dock_detector` | Detects a dock from 2D LiDAR geometry and publishes Nav2's external dock pose, confidence, state, and debug markers. | Strong optional capability for LiDAR-based docking. Retain only if it is field-tested against the selected dock geometry. |
| `low_obstacle_detection` | Detects low obstacles; its directory contains several timestamped/back-up implementations. | Do not migrate unchanged. Rebuild from a documented sensor requirement, test set, latency budget, and costmap interface. |
| `nav2_clear_goal_plugin` | Custom Costmap2D layer intended to clear an area around a goal. | Archive unless a reproducible navigation failure requires it; no clear active configuration ownership was found. |
| `nav2_clear_waypoint_plugin` | Custom Costmap2D layer intended to clear around waypoints. | Archive unless proven necessary. It appears only in an alternate/rendered configuration rather than the main robot navigation config. |
| `nav2_fov_cleanup_plugin` | Custom costmap layer that clears stale obstacles outside a limited forward LiDAR field of view. | Optional; retain only with sensor-FOV tests proving it is safer than standard observation persistence/clearing parameters. |
| `qr_localization` | Python QR-code based localisation node. | Optional perception/localisation aid; metadata is incomplete, so define accuracy, failure, and map-frame contracts before reuse. |
| `robot_execution` | Service/status wrapper around Nav2 navigation and docking actions. | Archive after direct-action migration. A sole `mission_orchestrator` should call Nav2 actions directly and publish product task state. |
| `roxy_docking` | AprilTag configuration and publisher for a detected dock pose. | Optional dock-perception implementation. It competes with `lidar_dock_detector`; select one detector per deployment. |
| `roxy_manager` | Demo/Nav2 manager for GPS/waypoint experiments and map operations. | Archive as a demo/reference. It must not become a second mission or motion authority. |
| `roxy_navigation` | Robot Nav2, localisation, SLAM, docking, costmap-filter, and launch configuration package. | Keep the responsibility as `navigation`, but split generic Nav2 configuration from optional perception/application launch logic. |
| `tag_localization` | Python tag-based localisation node. | Optional localisation aid; needs a documented tag family, map alignment, accuracy, and recovery contract before retention. |

### Mission, fleet, and external integration packages

| Package | Purpose | Decision |
| --- | --- | --- |
| `pade_bridge_pkg` | C++/gRPC bridge built around generated PADE protocol definitions. | Archive until a supported external-service contract requires it. Its package metadata does not describe a stable product role. |
| `rmf_gateway` | MQTT/WebSocket gateway that relays robot state and commands between RMF-side services and `robot_execution`. | Optional `fleet_adapter`. If RMF is used, rewrite its downstream side to call/query the sole `mission_orchestrator`, not `robot_execution`. |
| `roxi_bmc` | Legacy ROX-i Base Management Client for a tablet/server-specific WebSocket protocol. | Archive or isolate as a named external adapter. It is not a general base-management foundation and its documentation targets an older ROS release. |
| `roxy_acs` | SS-ACS integration and workflow scripts. | Archive as legacy integration. Its multiple large scripts should not coexist with the future mission orchestrator. Reimplement only the contracted ACS protocol as an adapter. |
| `roxi_interfaces` | Shared custom messages, services, and actions for navigation, MCU state, and integrations. | Keep the concept as a small `robot_interfaces` package, but prune legacy/vendor-specific interfaces and version its public API deliberately. |

## `roxi_main`-only packages

| Package | Purpose | Decision |
| --- | --- | --- |
| `arm_execution` | Station-aware asynchronous executor for named Nova-arm sequences. | Optional; retain for an arm product, keeping task semantics above the vendor driver. |
| `hmi_gateway` | Android/tablet gateway to mission-manager functions, with a legacy implementation intentionally excluded from the build. | Optional `hmi_adapter`; retain only with a supported HMI product contract. |
| `nova_arm_driver` | ROS 2 Jazzy C++ driver for the Dobot Nova 5 TCP/IP protocol. | Optional vendor driver; retain for Nova 5 hardware and pin/test it separately from generic arm tasks. |
| `robot_bringup` | Current top-level platform/autonomy/application launch composition. | Keep as the only production launch entry point. It should load the validated robot manifest and avoid embedding application integrations in navigation launches. |
| `robot_description` | Physical URDF/Xacro descriptions and model profiles for ROX-i/ROT-i variants. | Keep as `robot_description`; generalise model naming and connect it to the central robot manifest. |
| `robot_mission_manager` | Warehouse mission manager with a separate FSM/core and ROS node wiring. | Best candidate for the future `mission_orchestrator` shape, but choose it over—not alongside—`romi_mission_manager`. |
| `romi_mission_manager` | ROM-I V1 sequential pickup-to-conveyor workflow manager. | Archive or extract only reusable workflow ideas; it is product/workflow-specific, not a general company mission core. |
| `roxi_fw_updater` | ROS 2 CLI firmware-update utility for ROXI hardware. | Keep the controlled update capability under `system_manager`; preserve hardware-specific logic behind a defined updater interface. |
| `usb_cam` | Generic V4L USB camera driver. | Vendor dependency; keep only if the deployed camera is a UVC/V4L device. |
| `wes_gateway` | ROS 2 bridge from mission manager to WES MQTT/REST integrations. | Optional `wes_adapter`; retain only for a committed WES contract and have it submit tasks to the orchestrator. |

## `roxi`-only packages

| Package | Purpose | Decision |
| --- | --- | --- |
| `dynamic_amcl_manager` | Adjusts AMCL's LiDAR likelihood parameter when an external dynamic-environment flag is active. | Optional navigation policy. Keep only after localisation testing shows this tuning improves, rather than destabilises, performance. |
| `dynamic_obstacle_tracker` | Clusters LaserScan data and tracks dynamic obstacles/velocity. | R&D candidate. Retain only if prediction measurably improves safety or throughput beyond standard Nav2/collision monitoring. |
| `ing_crash_handler` | Shared crash-handler library using stack traces for C++/Python node failure analysis. | Useful engineering capability; integrate into the build/runtime observability policy rather than keeping an undocumented special package. |
| `localization_recovery` | Monitors IMU, wheel odometry, EKF, and AMCL consistency; can trigger localisation recovery after confirmed faults. | High-value concept, but do not enable automatic recovery until safety analysis and field validation exist. Fold validated health logic into `safety_supervisor`/state estimation. |
| `motor_state_monitor` | Monitors motor errors, lock state, and motor current/torque. | Keep the diagnostics, but merge them into `mcu_driver` diagnostics and `safety_supervisor`; a separate process is unnecessary unless isolation is justified. |
| `multi_robot_obstacle_layer` | Receives peer robot positions via MQTT and synthesises LaserScan obstacles for Nav2. | Optional fleet-coordination feature. Prefer a formal fleet/traffic contract over synthetic scans unless this technique is proven in deployment. |
| `narrow_corridor_manager` | Detects narrow corridors, records trajectories, detects encounters, and plans a retreat path; direct execution remains incomplete. | R&D candidate. Keep only if fleet behaviour requires it; the final authority belongs in fleet coordination and the mission orchestrator. |
| `nav2_theta_star_planner` | Nav2 Theta* any-angle global planner plugin. | Upstream Nav2 component; use the distro package or pin an upstream fork. Benchmark before selecting it over Smac/NavFn. |
| `person_follower` | YOLOv8 camera-based person follower with LiDAR safety filtering and velocity output. | Optional product feature. It introduces heavy Python/ML dependencies and must route through safety arbitration; omit from an AMR baseline. |
| `predicted_obstacle_layer` | Nav2 Costmap2D layer that paints predicted dynamic-obstacle positions with graded costs. | R&D candidate paired with dynamic tracking. Retain only after dataset-based prediction and navigation tests. |
| `roti_bmc` | ROT-i variant of the legacy base-management/tablet WebSocket client. | Archive or treat as a product-specific external adapter; do not retain beside `roxi_bmc`. |
| `roti_device_identity` | Reads hardware-derived identity and creates an immutable ROTI robot identity. | Keep the principle, not necessarily this package. Replace with the validated central `robot_identity` manifest plus optional hardware-serial verification. |
| `roxy_bringup` | Previous generation launch files for the ROXY stack. | Archive; `robot_bringup` supersedes it. |
| `roxy_description` | Previous generation robot-description package. | Archive; migrate required assets/model facts into `robot_description`. |
| `roxy_ssrcs` | Large integrated RCS gateway covering server transport, traffic, peer state, map sync, localisation scan streaming, navigation, elevator sessions, and mission control. | Do not migrate as one package. Extract only explicitly contracted adapters; otherwise it recreates a monolith with multiple motion authorities. |
| `slam_monitor` | Computes a SLAM-health score from graph stability, drift, jumps, and motion metrics. | Optional mapping/development diagnostic. Keep it out of localisation runtime unless validated thresholds have a defined operator/safety response. |

## Target minimum product set

For a first navigation-only AMR, start with:

```text
robot_interfaces       robot_identity        robot_description
robot_bringup          mcu_driver            base_kinematics
io_driver              state_estimation      navigation
mission_orchestrator   safety_supervisor     system_manager
```

Add a package only when a product requirement requires it: dock perception,
fleet/WES/HMI adapters, elevators, arm support, advanced perception, and
teleoperation are extensions—not baseline dependencies.
