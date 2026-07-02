# Gazebo target pose synchronization

## Purpose

`gazebo_target_pose_sync_node` makes `/target_pose` the pose source for the
Gazebo `target_object` model. It converts each valid world-frame
`geometry_msgs/msg/PoseStamped` into a Gazebo Harmonic
`ros_gz_interfaces/srv/SetEntityPose` request. This keeps fake perception, TF,
PlanningScene consumers, and Gazebo driven by the same source pose.

The feature is simulator-only. It does not execute robot motion or implement
gripper control, attachment, contacts, force control, cameras, or hardware.

## Launch

Build and source the workspace, then run the bounded headless demo:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_target_sync_demo.launch.py
```

To validate the adapter and topics without starting Gazebo:

```bash
ros2 launch adaptive_assembly_sim gazebo_target_pose_sync.launch.py \
  enable_service_calls:=false
```

## Topics

| Direction | Default topic | Type |
|---|---|---|
| Input | `/target_pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/gazebo_target_sync_status` | `std_msgs/msg/String` |
| Output | `/gazebo_target_pose_error_mm` | `std_msgs/msg/Float64` |
| Output | `/gazebo_target_pose_error_deg` | `std_msgs/msg/Float64` |

All outputs use reliable, transient-local QoS with depth one. Pose error is
zero after Gazebo accepts the requested pose. It is `NaN` when synchronization
is skipped or fails because no simulator pose was confirmed.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `target_pose_topic` | `/target_pose` | Input pose topic |
| `target_entity_name` | `target_object` | Gazebo model name |
| `world_frame` | `world` | Required input frame |
| `status_topic` | `/gazebo_target_sync_status` | Retained status topic |
| `pose_error_mm_topic` | `/gazebo_target_pose_error_mm` | Translation diagnostic |
| `pose_error_deg_topic` | `/gazebo_target_pose_error_deg` | Rotation diagnostic |
| `service_timeout_sec` | `2.0` | Bounded response timeout |
| `simulated_only` | `true` | Safety boundary; false is rejected |
| `enable_service_calls` | `true` | Enable Gazebo mutation requests |

The Gazebo Harmonic world service is
`/world/adaptive_assembly_workcell/set_pose`. The package-local launch starts
the `ros_gz_bridge` service bridge for this endpoint.

## Status format

Status is semicolon-delimited and always includes `mode`, `entity`,
`source_topic`, `simulated_only=true`, and `real_hardware=false`. Examples:

```text
event=success;mode=gazebo_target_sync;entity=target_object;source_topic=/target_pose;simulated_only=true;real_hardware=false
event=skipped;mode=gazebo_target_sync;reason=gazebo_service_unavailable;entity=target_object;source_topic=/target_pose;simulated_only=true;real_hardware=false
event=failure;mode=gazebo_target_sync;reason=invalid_frame;entity=target_object;source_topic=/target_pose;simulated_only=true;real_hardware=false
```

## Validation

```bash
bash scripts/check_gazebo_target_pose_sync_available.sh
python3 scripts/check_target_pose_to_gazebo_entity_consistency.py
python3 scripts/check_gazebo_target_pose_sync_status.py
```

The consistency check starts the node with service calls disabled, verifies
the request adapter preserves the pose, and verifies deterministic retained
`skipped` status and diagnostics without a Gazebo GUI.

## Limitations

The error topics describe whether Gazebo accepted the request; they do not
measure independent state feedback. The service endpoint currently targets the
fixed `adaptive_assembly_workcell` world. There is no interpolation, object
attachment, gripper behavior, contact simulation, camera perception, force
control, or real-hardware path.
