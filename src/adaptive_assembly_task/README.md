# Adaptive Assembly Task

This ROS 2 Jazzy package converts perceived target poses into simple
task-level poses for adaptive assembly. The `assembly_task_node` subscribes to
`/target_pose`, publishes vertically offset poses on `/pre_grasp_pose`,
`/grasp_pose`, `/assembly_pose`, and `/object_place_pose`, and reports when
target movement exceeds the replanning threshold. `/assembly_pose` is the
current robot hand target; `/object_place_pose` is the desired final object
pose. It does not perform motion planning or call MoveIt 2.

## Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `pre_grasp_height_offset` | `0.20` | Pre-grasp height above the target in meters |
| `grasp_height_offset` | `0.05` | Grasp height above the target in meters |
| `grasp_pose_topic` | `/grasp_pose` | Output topic for the explicit grasp pose |
| `assembly_height_offset` | `0.05` | Assembly height above the target in meters |
| `replan_distance_threshold` | `0.03` | Target movement requiring replanning in meters |
| `object_place_pose_topic` | `/object_place_pose` | Desired final object pose output topic |
| `assembly_pose_mode` | `target_offset` | Use `target_offset` or a configured `fixed_socket` pose |
| `socket_x`, `socket_y`, `socket_z` | `0.62`, `-0.18`, `0.10` | Fixed object/socket target position in meters |
| `socket_yaw` | `0.0` | Fixed object/socket target yaw in radians |
| `socket_frame_id` | `world` | Fixed object/socket target frame |

The output poses retain the target pose header and orientation.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select adaptive_assembly_task
source install/setup.bash
```

## Run

Launch the node:

```bash
ros2 launch adaptive_assembly_task assembly_task.launch.py
```

Run it directly:

```bash
ros2 run adaptive_assembly_task assembly_task_node
```

Override parameters from the launch command if needed:

```bash
ros2 launch adaptive_assembly_task assembly_task.launch.py \
  replan_distance_threshold:=0.05
```

## Test

With the perception and task packages built and the workspace sourced, run in
separate terminals:

```bash
ros2 launch adaptive_assembly_perception fake_perception.launch.py
```

```bash
ros2 launch adaptive_assembly_task assembly_task.launch.py
```

Inspect the outputs:

```bash
ros2 topic echo /pre_grasp_pose
ros2 topic echo /grasp_pose
ros2 topic echo /assembly_pose
ros2 topic echo /object_place_pose
```

Run package tests:

```bash
colcon test --packages-select adaptive_assembly_task
colcon test-result --verbose
```
