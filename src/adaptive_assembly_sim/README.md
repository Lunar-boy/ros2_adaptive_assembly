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
`joint_state_broadcaster` plus `panda_arm_controller`. Full two-stage execution
is composed from `adaptive_assembly_bringup`.

Gazebo starts paused. Panda creation completes before both controllers are
configured; launch then unpauses and activates them. Only the base is anchored.

The target synchronization node updates the static Gazebo model pose from
`/target_pose`; `static` means physics does not move the model autonomously.
This package does not add gripper control, object attach/detach, contact-rich
insertion, force control, camera perception, real robot drivers, or hardware
execution.

## Validate

After building and sourcing the workspace:

```bash
bash scripts/check_gazebo_workcell_assets.sh
bash scripts/check_gazebo_workcell_launch_available.sh
bash scripts/check_gazebo_panda_spawned.sh
bash scripts/check_ros2_control_controllers_active.sh
bash scripts/check_gazebo_target_pose_sync_available.sh
python3 scripts/check_target_pose_to_gazebo_entity_consistency.py
```
