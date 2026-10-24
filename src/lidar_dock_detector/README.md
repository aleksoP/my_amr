# `lidar_dock_detector`

LiDAR-only dock pose detector for Nav2 external docking. It replaces the AprilTag pose source with a 2D laser geometry pipeline and publishes a robot-relative `geometry_msgs/PoseStamped` on `/detected_dock_pose`.

## Pipeline

1. `scan_processor`
   Crops `/scan` by range and ROI in `base_link`.
2. `region_splitter`
   Splits the cropped scan into contiguous regions using point-gap distance.
3. `line_extractor`
   Runs split-and-merge line extraction with total least-squares fitting.
4. `landmark_matcher`
   Scores line windows against a YAML-defined multi-line landmark pattern.
5. `pose_estimator`
   Refines the matched landmark pose with model-to-scan ICP and converts marker pose to dock-contact pose.
6. `detection_filter`
   Applies multi-frame validation, pose jump rejection, and low-pass filtering.
7. `nav2_pose_publisher`
   Publishes:
   - `/detected_dock_pose`
   - `/dock_detector/confidence`
   - `/dock_detector/state`
   - `/dock_detector/debug_markers`

## Services

- `/dock_detector/start`
- `/dock_detector/stop`
- `/dock_detector/toggle_detection`

Use `/dock_detector/toggle_detection` for Nav2 docking's
`detector_service_name` hook. Keep `enabled_on_startup: false` so the first
Nav2 call starts detection and the second call stops it. The explicit start and
stop services are still useful for manual testing.

## Parameters

All geometry, ROI, tolerances, offsets, and temporal validation settings live in [config/lidar_dock_detector.yaml](config/lidar_dock_detector.yaml).

The default landmark values are examples. Tune these to your real LiDAR landmark:

- `landmark.segment_lengths`
- `landmark.turn_angles_deg`
- `landmark.vertices_x` / `landmark.vertices_y`
- `landmark.origin_mode`
- `landmark.origin_vertex`
- `landmark.forward_axis_deg`
- `landmark.length_tolerance`
- `landmark.angle_tolerance_deg`
- `landmark.marker_to_dock_offset.*`

You can define the landmark in two ways:

- Polyline by segment lengths and turn angles.
- Explicit polyline vertices with `landmark.vertices_x` and `landmark.vertices_y`.

When you are not using explicit vertices, omit `landmark.vertices_x` and
`landmark.vertices_y` from the YAML entirely. Do not set them to `[]`, because
ROS 2 parameter loading may treat an empty array as "no parameter value set"
and abort node startup.

The detector now canonicalizes that polyline into a pose frame before matching and ICP:

- `landmark.origin_mode: start | end | centroid | apex | vertex`
- `landmark.origin_vertex` is used when `origin_mode: vertex`
- `landmark.forward_axis_deg` defines which raw-model direction becomes the published marker pose `+x`

## Launch

```bash
ros2 launch lidar_dock_detector lidar_dock_detector.launch.py
```
## Toggle

```bash
ros2 service call /dock_detector/toggle_detection std_srvs/srv/Trigger {}
```
