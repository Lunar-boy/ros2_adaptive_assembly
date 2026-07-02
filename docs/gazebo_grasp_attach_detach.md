# Gazebo grasp attach/detach

## Purpose

This simulator-only layer consumes the existing logical grasp lifecycle. While
attached, it looks up `world -> panda_hand` and repeatedly sets the Gazebo
`target_object` model pose to that transform. Detach stops pose updates, leaving
the model at its last world pose. This is deterministic kinematic following,
not physics-accurate grasping.

## Launch

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_grasp_attach_demo.launch.py
```

The package-local launch can be used without Gazebo for fixture validation:

```bash
ros2 launch adaptive_assembly_manipulation gazebo_attach_detach.launch.py \
  enable_service_calls:=false
```

## Topics

Subscribed:

- `/object_grasp_state` (`std_msgs/msg/String`): semicolon-delimited
  `event=attached` and `event=detached` lifecycle events.
- `/object_grasp_attached` (`std_msgs/msg/Bool`): optional companion state.

Published with transient-local durability:

- `/gazebo_attach_detach_status` (`std_msgs/msg/String`)
- `/gazebo_object_attached` (`std_msgs/msg/Bool`)
- `/gazebo_attach_pose_error_mm` (`std_msgs/msg/Float64`)

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `object_grasp_state_topic` | `/object_grasp_state` | Lifecycle event input |
| `object_grasp_attached_topic` | `/object_grasp_attached` | Optional bool input; empty disables it |
| `status_topic` | `/gazebo_attach_detach_status` | Retained status output |
| `gazebo_object_attached_topic` | `/gazebo_object_attached` | Retained attachment bool |
| `pose_error_mm_topic` | `/gazebo_attach_pose_error_mm` | Retained diagnostic |
| `target_entity_name` | `target_object` | Gazebo model name |
| `world_frame` | `world` | Gazebo world TF frame |
| `gripper_frame` | `panda_hand` | Followed TF frame |
| `attach_update_period_sec` | `0.05` | Follow update period |
| `service_timeout_sec` | `2.0` | Set-pose response timeout |
| `enable_service_calls` | `true` | Enable Gazebo set-pose calls |
| `simulated_only` | `true` | Safety boundary; `false` is rejected |

## Status format

Statuses are semicolon-delimited key/value fields. Examples:

```text
event=attached;mode=gazebo_attach_detach;object=target_object;parent=panda_hand;simulated_only=true;real_hardware=false
event=detached;mode=gazebo_attach_detach;object=target_object;parent=world;simulated_only=true;real_hardware=false
event=skipped;mode=gazebo_attach_detach;reason=tf_unavailable;object=target_object;simulated_only=true;real_hardware=false
event=skipped;mode=gazebo_attach_detach;reason=gazebo_service_unavailable;object=target_object;simulated_only=true;real_hardware=false
```

## Validation

After building and sourcing the workspace:

```bash
bash scripts/check_gazebo_attach_detach_available.sh
python3 scripts/check_gazebo_attach_detach_success_path.py
python3 scripts/check_gazebo_attach_detach_failure_path.py
python3 scripts/check_gazebo_object_attached_status.py
```

The first two Python fixture checks do not require Gazebo. The retained-status
check expects the launch to be running.

## Limitations

The object origin is mirrored directly to the gripper frame; no grasp offset,
collision constraint, contact detection, force control, tactile feedback, or
physics-accurate joint is modeled. Detach only stops kinematic updates. This
feature has no real-robot or hardware path.
