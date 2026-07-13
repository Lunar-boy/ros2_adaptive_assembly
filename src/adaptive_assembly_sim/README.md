# adaptive_assembly_sim

ROS 2 package for the minimal Gazebo Harmonic adaptive assembly workcell and
the simulator-only Panda ros2_control fixture. The installed SDF world contains
a floor, table, target support, cylindrical target object, and assembly socket
fixture made from primitive geometry.

## Launch

```bash
ros2 launch adaptive_assembly_sim adaptive_assembly_workcell.launch.py
```

Launch the workcell plus Panda-like arm and ros2_control controllers:

```bash
ros2 launch adaptive_assembly_sim adaptive_assembly_panda_gazebo.launch.py
```

Launch target synchronization independently:

```bash
ros2 launch adaptive_assembly_sim gazebo_target_pose_sync.launch.py
```

Launch the achieved object pose observer independently:

```bash
ros2 launch adaptive_assembly_sim gazebo_entity_pose_observer.launch.py
```

The physical workcell attaches a bounded 30 Hz Gazebo `PosePublisher` directly
to `target_object`. It publishes a single `gz.msgs.Pose` on
`/model/target_object/pose`; the full physical launch bridges that to
`/gazebo_target_object_pose_raw` as `geometry_msgs/msg/PoseStamped` and feeds
the existing observer in its explicit `pose_stamped` input mode. Other demos
keep the existing Pose_V/TFMessage name-matching mode.

Adapt that observed model-center pose into the task pipeline independently:

```bash
ros2 launch adaptive_assembly_sim gazebo_target_pose_adapter.launch.py
```

`gazebo_target_pose_adapter_node` subscribes to
`/gazebo_target_object_pose` and publishes `/target_pose` only after a valid
observation arrives. Its parameters are:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `input_pose_topic` | `/gazebo_target_object_pose` | Observed Gazebo model pose |
| `output_pose_topic` | `/target_pose` | Task-compatible target pose |
| `target_reference_z_offset` | `0.05` | Offset from model center to task reference |
| `output_frame_id` | `world` | Output frame label |

The physical workcell target is a `0.10 m` cylinder whose Gazebo model pose is
at its center, so the `0.05 m` default selects its top/reference pose. XY and
orientation are preserved. `output_frame_id` replaces the frame label only;
the adapter does not perform a TF coordinate transform.

The launches require Gazebo and ros2_control integration packages. On ROS 2
Jazzy they can be installed with:

```bash
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

The Panda launch spawns a lightweight Panda-compatible arm model and starts
`joint_state_broadcaster`, `panda_arm_controller`, and
`panda_gripper_controller`. Full two-stage execution is composed from
`adaptive_assembly_bringup`.

The local Gazebo description is intentionally not unified with the standard
MoveIt resources Panda in the current diagnostic PR. After building and
sourcing the workspace, compare their kinematic contracts without launching a
ROS graph or Gazebo:

```bash
ros2 run adaptive_assembly_sim check_robot_model_parity \
  --current-panda-models
```

The current expected exit is `1` because the diagnostic detects structural and
FK mismatches. See `docs/robot_model_parity.md` from the workspace root for the
full command interface and interpretation.

Gazebo starts paused. Panda creation completes before both controllers are
configured; launch then unpauses and activates them. Only the base is anchored.

## Panda gripper model

The Gazebo Panda fixture includes simulator-only parallel gripper finger links
and prismatic finger joints:

- `panda_leftfinger`
- `panda_rightfinger`
- `panda_finger_joint1`
- `panda_finger_joint2`

The finger joints are exposed through the Gazebo ros2_control hardware
interface and controlled together by the simulator-only
`panda_gripper_controller`, a two-joint trajectory controller. This provides
finger-joint actuation and state visibility only. It does not verify contact,
lift, slip, or physical grasp success, and it does not add force control,
MoveIt Servo, real hardware support, or a physical pick-place executor.

The target synchronization node updates the static Gazebo model pose from
`/target_pose`; `static` means physics does not move the model autonomously.
This package does not add object attach/detach, physical grasp verification,
contact-rich insertion, force control, camera perception, real robot drivers,
or hardware execution.

## Validate

After building and sourcing the workspace:

```bash
bash scripts/check_gazebo_workcell_assets.sh
bash scripts/check_gazebo_workcell_launch_available.sh
bash scripts/check_gazebo_panda_spawned.sh
bash scripts/check_ros2_control_controllers_active.sh
bash scripts/check_gazebo_target_pose_sync_available.sh
python3 scripts/check_target_pose_to_gazebo_entity_consistency.py
bash scripts/check_gazebo_entity_pose_observer_available.sh
python3 scripts/check_gazebo_entity_pose_observer_synthetic.py
python3 scripts/check_gazebo_entity_pose_observer_stale.py
python3 scripts/check_gazebo_entity_pose_observer_pose_stamped.py
python3 scripts/check_physical_target_object_pose_transport.py
python3 -m pytest \
  src/adaptive_assembly_sim/test/test_gazebo_target_pose_adapter.py \
  src/adaptive_assembly_sim/test/test_gazebo_entity_pose_observer.py \
  src/adaptive_assembly_sim/test/test_physical_target_pose_transport_config.py
python3 scripts/check_panda_gripper_urdf_contains_fingers.py
python3 scripts/check_gripper_controller_config.py
```

With the simulator-only Panda Gazebo launch already running, the optional live
joint-state check can be run separately:

```bash
bash scripts/check_gazebo_gripper_joints_spawned.sh
bash scripts/check_gripper_controller_active.sh
```
