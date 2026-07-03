# Gazebo grasp attach/detach

## Purpose

This simulator-only layer consumes the existing logical grasp lifecycle. While
attached, it looks up `world -> panda_hand` and repeatedly sets the Gazebo
`target_object` model pose to the hand transform composed with a fixed local
hand-to-object offset. Detach stops pose updates, leaving the model at its last
world pose. This is deterministic kinematic following, not physics-accurate
grasping.

The node arbitrates Gazebo pose ownership on the retained
`/target_object_control_owner` topic:

- startup/free: `target_sync`
- attached: `gripper_attach`, published before the first attach pose update
- final detached/released: `released`

An explicit detached event with `trigger=startup` selects `target_sync`.
Later detached events select `released`; a missing trigger also selects
`released` as the safe deterministic default, preventing target sync from
snapping the object back to a continuously published perception pose.

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
- `/target_object_control_owner` (`std_msgs/msg/String`)

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `object_grasp_state_topic` | `/object_grasp_state` | Lifecycle event input |
| `object_grasp_attached_topic` | `/object_grasp_attached` | Optional bool input; empty disables it |
| `status_topic` | `/gazebo_attach_detach_status` | Retained status output |
| `control_owner_topic` | `/target_object_control_owner` | Retained pose-write owner output |
| `gazebo_object_attached_topic` | `/gazebo_object_attached` | Retained attachment bool |
| `pose_error_mm_topic` | `/gazebo_attach_pose_error_mm` | Retained diagnostic |
| `target_entity_name` | `target_object` | Gazebo model name |
| `world_frame` | `world` | Gazebo world TF frame |
| `gripper_frame` | `panda_hand` | Followed TF frame |
| `attached_object_offset_x` | `0.0` | Local hand-frame object X offset (m) |
| `attached_object_offset_y` | `0.0` | Local hand-frame object Y offset (m) |
| `attached_object_offset_z` | `0.0` | Local hand-frame object Z offset (m) |
| `attached_object_use_hand_orientation` | `true` | Inherit hand orientation; otherwise use identity |
| `attach_update_period_sec` | `0.05` | Follow update period |
| `service_timeout_sec` | `2.0` | Set-pose response timeout |
| `enable_service_calls` | `true` | Enable Gazebo set-pose calls |
| `simulated_only` | `true` | Safety boundary; `false` is rejected |

## Status format

Statuses are semicolon-delimited key/value fields. Examples:

```text
event=attached;mode=gazebo_attach_detach;owner=gripper_attach;object=target_object;hand_to_object_offset=0.0,0.0,0.1;parent=panda_hand;simulated_only=true;real_hardware=false
event=detached;mode=gazebo_attach_detach;owner=released;object=target_object;parent=world;simulated_only=true;real_hardware=false
event=skipped;mode=gazebo_attach_detach;reason=tf_unavailable;owner=gripper_attach;object=target_object;simulated_only=true;real_hardware=false
```

## Validation

After building and sourcing the workspace:

```bash
bash scripts/check_gazebo_attach_detach_available.sh
python3 scripts/check_gazebo_attach_detach_success_path.py
python3 scripts/check_gazebo_attach_detach_failure_path.py
python3 scripts/check_gazebo_object_attached_status.py
python3 scripts/check_gazebo_attach_owner_transitions.py
python3 scripts/check_gazebo_attachment_offset.py
```

The first two Python fixture checks do not require Gazebo. The retained-status
check expects the launch to be running.

### Live Gazebo validation

The optional live check starts the minimal workcell headlessly, supplies a
static `world -> panda_hand` transform, runs the attachment node with service
calls enabled, and verifies successful attach and detach state transitions
through Gazebo's real set-pose service path. Run it after building and sourcing
the workspace:

```bash
python3 scripts/check_live_gazebo_attach_detach.py
```

The check uses explicit startup and event timeouts, terminates all processes it
starts, and fails if the Gazebo service or a confirmed set-pose response is
missing. It requires Gazebo Harmonic and `ros-jazzy-ros-gz-sim`, so it is kept
optional for environments where a live simulator is too heavy.

## Limitations

The configurable offset is rotated by the hand orientation before being added
to the hand position. Zero offset preserves direct origin mirroring. The visual
single-trial demo uses a tunable `0.10 m` local Z offset to place the cylinder
at the simplified Panda tool geometry. This is only a visual-correctness aid:
no collision constraint, contact detection, force control, tactile feedback,
or physics-accurate joint is modeled. Detach only stops kinematic updates. This
feature has no real-robot or hardware path.
