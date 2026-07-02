# Simulated Vision Perception

## Purpose

`simulated_marker_pose_node` is a simulator-only, camera-frame marker-pose
perception emulator for the adaptive assembly pipeline. It expresses the
configured target observation in `simulated_camera`, publishes the converted
world pose on `/target_pose`, and broadcasts `world -> simulated_camera` and
`world -> target_object`. This is not pixel-based vision. It requires no Gazebo
GUI, image transport, OpenCV, camera device, or robot hardware.

The emulator is deterministic when its noise standard deviations are zero.
With noise enabled, its pseudo-random sequence is seeded by `marker_id`, so
identical parameters produce identical sequences.

## Launch

Run perception alone for headless validation:

```bash
ros2 launch adaptive_assembly_perception \
  simulated_vision_perception.launch.py
```

Run perception, task pose generation, the headless Gazebo workcell, and target
object synchronization:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_simulated_vision_demo.launch.py
```

## Interfaces

| Topic | Type | Behavior |
| --- | --- | --- |
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | Existing pipeline input |
| `/perceived_target_pose` | `geometry_msgs/msg/PoseStamped` | Optional raw observation in `simulated_camera`; set its parameter to an empty string to disable |
| `/simulated_vision_perception_status` | `std_msgs/msg/String` | Reliable, transient-local status |
| `/tf` | `tf2_msgs/msg/TFMessage` | `world -> simulated_camera` and `world -> target_object` transforms |

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `target_pose_topic` | `/target_pose` | Pipeline output topic |
| `perceived_pose_topic` | `/perceived_target_pose` | Raw pose output; empty disables it |
| `status_topic` | `/simulated_vision_perception_status` | Retained status topic |
| `world_frame` | `world` | Pose and TF parent frame |
| `camera_frame` | `simulated_camera` | Logical simulated sensor frame |
| `target_frame_id` | `target_object` | TF child frame |
| `marker_id` | `0` | Emulated marker ID and deterministic noise seed |
| `target_entity_name` | `target_object` | Logical simulator entity name |
| `publish_period_sec` | `1.0` | Observation period |
| `camera_x`, `camera_y`, `camera_z` | `0.0`, `0.0`, `1.0` | Simulated camera position in `world` |
| `camera_yaw` | `0.0` | Simulated camera yaw in `world`, in radians |
| `x`, `y`, `z` | `0.45`, `0.0`, `0.15` | Base world position in metres |
| `yaw` | `0.0` | Base world yaw in radians |
| `position_noise_std` | `0.0` | Gaussian position noise standard deviation |
| `yaw_noise_std` | `0.0` | Gaussian yaw noise standard deviation |
| `publish_immediately` | `true` | Publish during startup |
| `enable_camera_topics` | `false` | Reserved opt-in for future simulator camera input; the emulator remains active |
| `simulated_only` | `true` | Safety boundary; `false` is rejected |

## Status format

Status is semicolon-delimited `key=value` data. A successful observation is:

```text
event=success;mode=simulated_vision_perception;source=marker_pose_emulator;perceived_frame=simulated_camera;target_frame=target_object;simulated_only=true;real_hardware=false
```

When camera topics are disabled, startup also emits a skipped event before the
first successful emulated observation. The retained terminal status is the
latest successful observation.

## Validation

Build and source the workspace, launch the standalone perception launch in one
terminal, then run:

```bash
bash scripts/check_simulated_vision_perception_available.sh
python3 scripts/check_simulated_vision_perception_status.py
python3 scripts/check_simulated_vision_to_target_pose.py
python3 scripts/check_simulated_vision_to_tf_consistency.py
```

These checks need neither Gazebo nor a camera.

## Limitations and fake perception comparison

This first version emulates camera-frame marker-pose output; it does not process
pixels, detect ArUco markers, model occlusion, or perform visual servoing. The existing
fake perception node remains unchanged and is useful for randomized workspace
and benchmark inputs. Simulated vision instead exposes camera/marker semantics,
raw perceived output, retained diagnostics, deterministic measurement noise,
and a strict simulator-only boundary.
