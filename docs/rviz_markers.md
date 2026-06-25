# RViz marker visualization

PR25 adds a lightweight MarkerArray publisher for inspecting the adaptive
assembly pose pipeline in RViz2.

This is visualization-only:

- no PlanningScene modification
- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware

## Node and topics

`adaptive_assembly_marker_node` is provided by
`adaptive_assembly_planning`.

Subscribed pose topics:

- `/target_pose`
- `/pre_grasp_pose`
- `/assembly_pose`
- `/panda_pre_grasp_pose`

Published topics:

- `/adaptive_assembly_markers`: `visualization_msgs/msg/MarkerArray`
- `/adaptive_assembly_marker_status`: `std_msgs/msg/String`

Default parameters:

- `marker_topic: /adaptive_assembly_markers`
- `status_topic: /adaptive_assembly_marker_status`
- `marker_scale: 0.05`
- `arrow_length: 0.12`
- `marker_lifetime_sec: 0.0`

## Marker namespaces and types

Each incoming pose updates one marker with the incoming pose frame.

| Source topic | Marker namespace | Marker type |
| --- | --- | --- |
| `/target_pose` | `target_pose` | sphere |
| `/pre_grasp_pose` | `pre_grasp_pose` | arrow |
| `/assembly_pose` | `assembly_pose` | cube |
| `/panda_pre_grasp_pose` | `panda_pre_grasp_pose` | arrow |

If an incoming pose has an empty frame, the node skips that marker and
publishes a `skipped_empty_frame` status event.

## Status topic

The marker node publishes a semicolon-separated status string on
`/adaptive_assembly_marker_status`.

Fields:

- `event`: `marker_updated` or `skipped_empty_frame`
- `source`
- `frame`
- `x`
- `y`
- `z`

Example:

```text
event=marker_updated;source=panda_pre_grasp_pose;frame=panda_link0;x=0.450;y=0.000;z=0.350
```

## Launch

The Panda planning demo starts marker visualization by default:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

It can be disabled if needed:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py use_rviz_markers:=false
```

The package-local launch is:

```bash
ros2 launch adaptive_assembly_planning adaptive_assembly_markers.launch.py
```

## Display in RViz2

In RViz2:

1. Click `Add`.
2. Select `By topic`.
3. Choose `/adaptive_assembly_markers`.
4. Use a fixed frame that can see the relevant marker frames, such as
   `world` for task-level poses or `panda_link0` for Panda-adapted poses.

The standard Panda MoveIt2 demo may already open RViz2. The MarkerArray display
can be added to that RViz session.

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
bash scripts/check_adaptive_assembly_markers_available.sh
ros2 topic list | grep adaptive_assembly_markers
python3 scripts/check_adaptive_assembly_marker_status.py
```

The marker status checker confirms that marker status messages are parseable.
