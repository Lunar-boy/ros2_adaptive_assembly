# Adaptive Assembly Perception

This ROS 2 Jazzy package provides fake and simulated-vision inputs for the
adaptive assembly pipeline. The `fake_object_pose_node` publishes a randomized
`geometry_msgs/msg/PoseStamped` on `/target_pose` and broadcasts the same pose
as the `world` to `target_object` TF transform.

The simulator-only `simulated_marker_pose_node` provides a deterministic
camera-frame marker pose emulator, not pixel-based vision. It publishes the raw
observation in `simulated_camera`, converts it to the existing world-frame
pipeline interface, broadcasts matching camera and target TF frames, and emits
retained status. See
[`docs/simulated_vision_perception.md`](../../docs/simulated_vision_perception.md).

The optional `aruco_detector_node` detects configured ArUco markers in
simulator-published images when `cv2.aruco` is available. OpenCV is loaded
lazily and is not a package dependency. See
[`docs/opencv_aruco_perception.md`](../../docs/opencv_aruco_perception.md).

## Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `publish_period_sec` | `5.0` | Seconds between randomized poses |
| `x_min` | `0.35` | Minimum target x position in meters |
| `x_max` | `0.55` | Maximum target x position in meters |
| `y_min` | `-0.25` | Minimum target y position in meters |
| `y_max` | `0.25` | Maximum target y position in meters |
| `z` | `0.15` | Fixed target z position in meters |
| `random_seed` | `-1` | Seed for deterministic pose generation; negative values keep non-deterministic behavior |
| `frame_id` | `world` | Frame used by `/target_pose` and TF parent |
| `target_frame_id` | `target_object` | TF child frame for the target object |
| `yaw_min` | `-pi` | Minimum sampled target yaw in radians |
| `yaw_max` | `pi` | Maximum sampled target yaw in radians |
| `publish_immediately` | `true` | Publish one pose immediately on startup before the timer |
| `publish_target_pose_service` | `/publish_target_pose_once` | Trigger service that publishes one fresh pose through the same path |

The published pose uses `frame_id`. Its orientation is a sampled yaw represented
by a unit quaternion.

For deterministic benchmark mode, set `random_seed` to a non-negative integer.
The same seed and parameter ranges produce the same pose sequence. A negative
`random_seed` preserves non-deterministic behavior.

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select adaptive_assembly_perception
source install/setup.bash
```

## Run

Launch with the default parameters:

```bash
ros2 launch adaptive_assembly_perception fake_perception.launch.py
```

Run the node directly:

```bash
ros2 run adaptive_assembly_perception fake_object_pose_node
```

Run the node directly with deterministic pose generation:

```bash
ros2 run adaptive_assembly_perception fake_object_pose_node --ros-args -p random_seed:=42
```

Parameters can be overridden from the launch command, for example:

```bash
ros2 launch adaptive_assembly_perception fake_perception.launch.py \
  publish_period_sec:=2.0
```

## Test

In separate terminals after building and sourcing the workspace:

```bash
ros2 topic echo /target_pose
```

```bash
ros2 run tf2_ros tf2_echo world target_object
```

Publish one fresh simulated target pose on demand:

```bash
ros2 service call /publish_target_pose_once std_srvs/srv/Trigger '{}'
```

Run the package tests with:

```bash
colcon test --packages-select adaptive_assembly_perception
colcon test-result --verbose
```
