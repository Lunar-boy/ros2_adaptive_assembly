# Optional OpenCV ArUco perception

## Purpose

`aruco_detector_node` is an optional simulator-only pixel detection path. It
consumes simulated `sensor_msgs/msg/Image` and `CameraInfo` messages, estimates
one configured ArUco marker pose, and feeds the existing perception interfaces.
The deterministic `simulated_marker_pose_node` remains the recommended default
for headless environments.

No real camera, robot hardware, trajectory execution, visual servoing, or
contact/force behavior is provided.

## Run

After building and sourcing the workspace:

```bash
ros2 launch adaptive_assembly_perception opencv_aruco_perception.launch.py
```

To include the existing task pose generation pipeline:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_opencv_aruco_demo.launch.py
```

These launches expect a simulator to publish `/camera/image_raw` and
`/camera/camera_info`; they do not open a camera device. Gazebo target pose
synchronization can be composed separately when a Gazebo workcell is already
in use.

## Interfaces

Inputs:

- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

Outputs:

- `/perceived_target_pose` (`geometry_msgs/msg/PoseStamped`) in
  `simulated_camera`
- `/target_pose` (`geometry_msgs/msg/PoseStamped`) in `world`
- `/aruco_detection_status` (`std_msgs/msg/String`, retained)
- `/aruco_marker_detected` (`std_msgs/msg/Bool`, retained)
- TF `world -> simulated_camera` and `world -> target_object`

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `image_topic` | `/camera/image_raw` | Simulated image input |
| `camera_info_topic` | `/camera/camera_info` | Camera model input |
| `target_pose_topic` | `/target_pose` | World-frame output |
| `perceived_pose_topic` | `/perceived_target_pose` | Raw camera-frame output |
| `status_topic` | `/aruco_detection_status` | Retained status output |
| `marker_detected_topic` | `/aruco_marker_detected` | Retained detection bool |
| `world_frame` | `world` | Output/TF parent frame |
| `camera_frame` | `simulated_camera` | Raw pose and camera TF frame |
| `target_frame_id` | `target_object` | Target TF child frame |
| `marker_id` | `0` | Marker ID to accept |
| `marker_size_m` | `0.05` | Physical marker side length |
| `aruco_dictionary` | `DICT_4X4_50` | OpenCV dictionary constant |
| `camera_x/y/z` | `0/0/1` | Camera origin in `world` |
| `camera_yaw` | `0` | Camera yaw in `world` |
| `publish_tf` | `true` | Publish camera and target TF |
| `fallback_to_emulator` | `true` | Documents the recommended fallback policy |
| `detection_timeout_sec` | `2.0` | Input/detection stale timeout |
| `simulated_only` | `true` | Safety boundary; `false` is rejected |

The image decoder accepts `mono8`, `8UC1`, `bgr8`, and `rgb8`. Camera position
and yaw define the same simple world-to-camera model used by the existing
marker-pose emulator.

## Optional dependency behavior

OpenCV is imported lazily. Neither OpenCV nor `cv_bridge` is a package runtime
dependency. If `cv2` or its ArUco module is unavailable, the node stays alive
and publishes:

```text
event=skipped;mode=opencv_aruco_detector;reason=opencv_unavailable;simulated_only=true;real_hardware=false
```

Missing camera info, missing images, absent markers, and invalid camera models
also produce deterministic retained status. The node does not automatically
start a second publisher: launch `simulated_vision_perception.launch.py` to use
the fallback, avoiding competing `/target_pose` publishers.

## Validation

```bash
bash scripts/check_opencv_aruco_detector_available.sh
python3 scripts/check_opencv_aruco_detector_status.py
python3 scripts/check_opencv_aruco_synthetic_detection.py
python3 scripts/check_opencv_aruco_to_target_pose.py
```

The synthetic check generates a marker image without Gazebo or camera hardware.
It reports `SKIP` with exit code zero when OpenCV ArUco support is unavailable.
The status and pose/TF checks are intended to run while the relevant detector
launch and simulated camera publisher are active.

## Limitations

- Single configured marker ID and square-marker pose estimation only.
- Camera extrinsics support position plus yaw, not arbitrary roll/pitch.
- No image synchronization, filtering, tracking, calibration, or occlusion
  handling.
- Simulator inputs only; no real camera or hardware integration.
