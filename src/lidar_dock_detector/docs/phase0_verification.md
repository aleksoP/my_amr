# Phase 0 verification

## Contract

`/scan` is transformed into `output_frame` at its header timestamp. A successful
geometry evaluation creates one immutable raw marker and contact measurement in
that frame. A fresh measurement is transformed into `tracking_frame` (`odom` by
default) at the same timestamp before the temporal tracker updates. The Nav2
topic `/detected_dock_pose` is published only for a fresh usable tracked update;
held estimates are neither restamped nor republished as observations.

Raw pose topics are `/dock_detector/raw_marker_pose` and
`/dock_detector/raw_dock_pose`. `dock_marker` and `dock_contact` TF are optional
and disabled by default: TF has no validity/retraction semantic and stale dynamic
transforms must not be used as authoritative dock localization.

The watchdog uses `std::chrono::steady_clock`: it remains effective if `/clock`
pauses, but is intentionally independent of ROS measurement timestamps. Scan
timestamps are preserved for all pose data; equal or older measurements are
rejected by the tracker.

## Automated evidence

| Test | Scenario | Result |
|---|---|---|
| `test_detection_filter` | acquisition, held estimate provenance, out-of-order rejection, scan watchdog, startup timeout, strict dropout handling | PASS |
| `test_config_validation` | invalid ROI, sample spacing, confidence ratio, timeout | PASS |
| CTest linters | cppcheck, flake8, CMake, pep257, uncrustify, XML | pending final isolated run |

No real-node synthetic scan/TF launch test exists yet. Consequently stationary
world-frame tracking through the complete geometric node, Nav2's private refined
pose path, ROS time resets, and physical LiDAR behavior remain unverified.

## Nav2 Jazzy 1.3.13 contract

The installed `opennav_docking::SimpleChargingDock` subscribes to
`detected_dock_pose` and has its own pose filter. The workspace config sets all
external detection translation/rotation corrections to zero and `filter_coef` to
zero because this package publishes a calibrated planar contact pose. Its external
detection timeout is `0.25` seconds, equal to the detector stale timeout.

The navigation mux does not currently launch this package. Deployment must launch
`lidar_dock_detector.launch.py` exactly once in the same namespace/TF tree before
docking, with the newly built overlay sourced before older underlays.

## Release verdict

**CONDITIONAL PASS for controlled integration work; not yet ready for controlled
physical docking validation.** The code-level timing, frame, and Nav2-offset
contracts are covered by unit tests, but the required real-node synthetic
scan/TF/ROS integration harness and hardware calibration evidence remain release
gates.
