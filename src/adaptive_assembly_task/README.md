# Adaptive Assembly Task

This ROS 2 Jazzy package converts perceived target poses into simple
task-level poses for adaptive assembly. The `assembly_task_node` subscribes to
`/target_pose` and publishes a deterministic explicit grasp sequence on
`/grasp_candidates`, `/selected_grasp_pose`, `/pre_grasp_pose`, `/grasp_pose`,
`/lift_pose`, `/object_place_pose`, `/pre_place_pose`, `/place_pose`, and
`/retreat_pose`, with status on `/grasp_sequence_status`. `/grasp_pose` is a
backward-compatible alias of `/selected_grasp_pose`. After the first target,
updates at or below `replan_distance_threshold` are skipped; decisions are
published on `~/status`. A threshold of zero or less disables this gate. In fixed-socket mode,
`/assembly_pose` is a legacy alias of `/place_pose`. `/assembly_pose` is the
current robot hand target; `/object_place_pose` is the desired final object
pose. It does not perform motion planning or call MoveIt 2.

## Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `pre_grasp_height_offset` | `0.20` | Pre-grasp height above the target in meters |
| `grasp_height_offset` | `0.05` | Grasp height above the target in meters |
| `grasp_pose_topic` | `/grasp_pose` | Output topic for the explicit grasp pose |
| `grasp_candidates_topic` | `/grasp_candidates` | Stable `std_msgs/String` candidate schema |
| `selected_grasp_pose_topic` | `/selected_grasp_pose` | Selected candidate pose |
| `lift_pose_topic` | `/lift_pose` | Lift pose above the selected grasp |
| `grasp_sequence_status_topic` | `/grasp_sequence_status` | Stable sequence status schema |
| `grasp_candidate_count` | `4` | Number of deterministic yaw candidates |
| `grasp_candidate_yaw_step_rad` | `1.5707963267948966` | Candidate yaw increment in radians |
| `selected_grasp_candidate_index` | `0` | Candidate published as the selected/grasp pose |
| `lift_height_offset` | `0.20` | Lift height above the selected grasp in meters |
| `preserve_target_orientation_for_candidates` | `false` | Copy target orientation instead of generating yaw-only candidates |
| `assembly_height_offset` | `0.05` | Assembly height above the target in meters |
| `replan_distance_threshold` | `0.03` | Target movement requiring replanning in meters; `<= 0.0` disables filtering |
| `object_place_pose_topic` | `/object_place_pose` | Desired final object pose output topic |
| `assembly_pose_mode` | `target_offset` | Use `target_offset` or a configured `fixed_socket` pose |
| `socket_x`, `socket_y`, `socket_z` | `0.62`, `-0.18`, `0.10` | Fixed object/socket target position in meters |
| `socket_yaw` | `0.0` | Fixed object/socket target yaw in radians |
| `socket_frame_id` | `world` | Fixed object/socket target frame |

The candidate position is the target position plus `grasp_height_offset` in z.
By default, candidate yaw is target yaw plus the candidate index times the yaw
step. The explicit schema is deterministic and planning-interface-only. It is
not physical grasping and adds no gripper control, contact-rich insertion, or
real hardware execution. `/grasp_candidates` intentionally uses
`std_msgs/String`, rather than a custom generated message, for this PR.

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
ros2 topic echo /grasp_candidates
ros2 topic echo /selected_grasp_pose
ros2 topic echo /lift_pose
ros2 topic echo /grasp_sequence_status
ros2 topic echo /assembly_pose
ros2 topic echo /object_place_pose
```

Run package tests:

```bash
colcon test --packages-select adaptive_assembly_task
colcon test-result --verbose
```

With the perception and task nodes running, validate the schema with:

```bash
bash scripts/check_grasp_sequence_schema_topics.sh
python3 scripts/check_grasp_sequence_schema_status.py
```
