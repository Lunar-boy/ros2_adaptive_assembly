# TF2 Panda pose adapter mode

PR22 adds optional TF2 support to `panda_pre_grasp_pose_adapter_node`.

The default behavior remains unchanged. With `use_tf_transform: false`, the
adapter keeps the existing simple behavior:

- read `/pre_grasp_pose`
- copy the numeric position
- apply configured x/y/z offsets
- override `header.frame_id` with `output_frame_id` when configured
- apply the fixed Panda-oriented quaternion by default
- publish `/panda_pre_grasp_pose`

This mode is intentionally simple and remains the default for current demos and
benchmarks.

## Optional TF2 transform mode

When `use_tf_transform: true`, the adapter uses TF2 to transform the incoming
`geometry_msgs/msg/PoseStamped` into `target_frame_id` before applying offsets
and orientation adaptation.

Default TF2-related parameters:

- `use_tf_transform: false`
- `target_frame_id: panda_link0`
- `tf_lookup_timeout_sec: 0.2`
- `status_topic: /panda_pose_adapter_status`

If the input frame is empty, the adapter skips publishing and reports
`skipped_empty_frame`. If TF lookup fails, it skips publishing and reports
`tf_lookup_failed`.

## Status topic

The adapter publishes one status event for each input on
`/panda_pose_adapter_status` as `std_msgs/msg/String`.

Fields:

- `event`: `adapted`, `tf_lookup_failed`, or `skipped_empty_frame`
- `input_frame`
- `output_frame`
- `use_tf_transform`
- `x`
- `y`
- `z`
- `fixed_orientation`

Example:

```text
event=adapted;input_frame=world;output_frame=panda_link0;use_tf_transform=false;x=0.450;y=0.000;z=0.350;fixed_orientation=true
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Validate

With the Panda planning demo already running:

```bash
bash scripts/check_panda_pose_adapter_tf2_params.sh
bash scripts/check_panda_adapted_pose.sh
bash scripts/check_panda_adapted_pose_frame.sh
python3 scripts/check_panda_pose_adapter_orientation.py --expected-frame panda_link0
python3 scripts/check_panda_pose_adapter_status.py
```

## Scope

This PR adds pose adaptation diagnostics and optional TF2 transform support only.
It does not execute trajectories, add Gazebo, add ros2_control, or require real
hardware.
