# open3d_slam_ros

## Online Modes

The ROS 2 wrapper currently supports two online operating modes:

- `standalone_slam`
  - Open3D owns its own `map_o3d -> odom_o3d -> sensor` TF tree.
  - Internal ICP odometry drives scan matching and map publication.
  - Use this when Open3D itself should be the localization authority, or when no trusted external estimator exists.
- `external_pose_mapping`
  - The wrapper looks up `external_pose_frame -> <incoming cloud frame>` for every cloud and uses it as the online motion prior.
  - Map outputs and saved maps are transformed into that external frame.
  - Open3D TF publication should stay disabled so the external estimator remains the single owner of the global robot tree.

The implementation keeps the standalone path intact while adding the robot-integration mode in the ROS wrapper.

## Design Review

For robots that already have a trusted estimator, running a second SLAM-owned global TF tree is the wrong default:

- `map_o3d` is disconnected from the robot `map -> odom -> base` chain, so downstream consumers need ad-hoc bridging.
- If Open3D also publishes into the live robot TF tree, there are two localization authorities with the same semantics.

The reviewed options were:

1. Bridge `map_o3d` back into `map`.
   Rejected: this hides the symptom while keeping two competing localization trees alive.
2. Replace standalone Open3D SLAM with estimator-following behavior.
   Rejected: this would compromise the package by deleting its self-contained SLAM mode.
3. Keep standalone SLAM intact and add a dedicated robot-integration mode in the ROS wrapper.
   Implemented: `standalone_slam` remains available, and `external_pose_mapping` is the robot-facing mode.

## Target Architecture

The robot architecture is:

- Open3D publishes smooth local ICP odometry from `scan2scan_odometry`.
- the external estimator fuses that ICP odometry together with its other inputs.
- the external estimator remains the only owner of `map -> odom -> base`.
- Open3D publishes and saves map products in the estimator/global frame.

This is the clean split of responsibilities:

- Open3D provides local LiDAR motion information and mapping.
- the external estimator owns the globally consistent robot pose.
- Consumers see a single robot TF authority and a georeferenced map.

## Current State

The current wrapper now supports the intended split for online robot mapping:

- `external_pose_mapping` keeps using the external pose as the mapping motion prior and map-output frame.
- `scan2scan_odometry` remains an independent Open3D ICP odometry stream in the Open3D odometry frame.
- `scan2map_odometry`, `assembled_map`, `dense_map`, `submaps`, and saved map products stay aligned with the configured external pose frame.

The mode split is now:

- `standalone_slam`: use when you want Open3D to own localization and publish its own ICP odometry/TF tree.
- `external_pose_mapping`: use when you want the map to already live in the robot/global `map` frame.

Do not run standalone Open3D and a live estimator together in normal robot operation unless you intentionally want two localization trees.

## Recommended Runtime Usage

If you want a georeferenced map on the robot today, run the estimator and Open3D in `external_pose_mapping`:

```bash
ros2 launch open3d_slam_ros mapping.launch.py \
  cloud_topic:=/colored_point_cloud \
  external_pose_frame:=map
```

Effects:

- `assembled_map`, `dense_map`, and `submaps` are published in `map`
- `save_map` and `save_submaps` export clouds in `map`
- uncolored exports default to `.pcd` and colored exports to `.ply`
- Open3D does not create a competing global TF tree

If you want to evaluate Open3D as a standalone SLAM/localization system, run it without `external_pose_frame` and without a competing estimator.

## Estimator Fusion Notes

The clean estimator input is smooth local ICP odometry, not a loop-closure-corrected global SLAM pose.

Use this rule:

- `scan2scan_odometry`: appropriate candidate for external-estimator fusion in both `standalone_slam` and `external_pose_mapping`
- `scan2map_odometry`: do not fuse blindly as ordinary LIO, because scan-to-map corrections can jump when the map is re-anchored

In `external_pose_mapping`, `scan2scan_odometry` stays in the Open3D odometry frame while map products are published in the external/global frame.
The wrapper also fixes the `scan2scan_odometry` parent frame so it is stamped in the odometry frame instead of the map frame.

## Parameters

- `external_pose_frame` (`string`, default: empty)
  - Empty: use standalone Open3D odometry/TF.
  - Non-empty: look up `external_pose_frame -> <incoming cloud frame>` for every cloud and use it as the motion prior.
- `external_pose_lookup_timeout_sec` (`double`, default: `0.1`)
  - TF lookup timeout for the external pose query.
- cloud input QoS
  - Online cloud subscribers use sensor-data QoS so they can consume both best-effort sensor topics and reliable processed clouds without a relay.
- `publish_tf` (`bool`, default: mode-aware when left empty)
  - Empty: `mapping.launch.py` lets the node keep `true` in standalone mode and `false` when `external_pose_frame` is set.
  - Controls whether Open3D publishes its own TF worker outputs.
  - On robots with an existing estimator, keep this `false`.
