# Full Gazebo Panda execution

PR37 adds a self-contained simulator-only execution path that physically moves
a Panda-like arm in Gazebo Harmonic through `ros2_control`.

The demo composes:

- the existing deterministic known-reachable sequence planning profile;
- the existing Gazebo workcell world;
- a lightweight Panda-like URDF/xacro model with Panda joint names;
- `gz_ros2_control` with `joint_state_broadcaster`;
- `panda_arm_controller`, a `FollowJointTrajectory` controller;
- the existing `ros2_control_sequence_executor_node`.

It reuses `/pre_grasp_trajectory` and `/assembly_trajectory`. The executor
sends the two stages to
`/panda_arm_controller/follow_joint_trajectory` and publishes the retained
terminal result topics already used by the ros2_control execution bridge.

## Required packages

Install Gazebo and ros2_control integration packages when they are not already
available:

```bash
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro
```

The planning side still requires the MoveIt2 Panda demo resources used by the
existing known-reachable sequence profile.

## Launch

Build and source the workspace:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Start the full simulator execution demo:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_gazebo_execution_demo.launch.py
```

Gazebo starts paused. The Panda is spawned first, and the controller spawner is
started only after entity creation completes. Both controllers are loaded and
configured while paused; launch then deterministically unpauses Gazebo and
activates them. Do not manually unpause before controller configuration. The
Panda base is simulator-only kinematic, so it remains anchored while the seven
arm joints remain controllable.

For server-only validation, pass a full Gazebo argument string with the world
path. Omit `-r` during startup; unpause only after controller readiness:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_gazebo_execution_demo.launch.py \
  gz_args:="-s $(ros2 pkg prefix adaptive_assembly_sim)/share/adaptive_assembly_sim/worlds/adaptive_assembly_workcell.sdf"
```

## Interfaces

Spawned Gazebo entity:

- `panda`

Controllers:

- `joint_state_broadcaster`
- `panda_arm_controller`

Action:

- `/panda_arm_controller/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`)

Inputs:

- `/pre_grasp_trajectory` (`moveit_msgs/msg/RobotTrajectory`)
- `/assembly_trajectory` (`moveit_msgs/msg/RobotTrajectory`)

Controller state:

- `/joint_states` (`sensor_msgs/msg/JointState`)

Terminal execution outputs:

- `/assembly_ros2_control_execution_status` (`std_msgs/msg/String`)
- `/assembly_ros2_control_execution_success` (`std_msgs/msg/Bool`)
- `/assembly_ros2_control_execution_duration_ms` (`std_msgs/msg/Float64`)
- `/assembly_ros2_control_execution_stage_status` (`std_msgs/msg/String`)

Expected successful terminal status includes:

```text
event=success;mode=ros2_control;stage_count=2;...;execution=true;simulated_execution_only=true;real_hardware=false
```

If the controller action is unavailable, the executor publishes a deterministic
skipped result after `wait_for_controller_sec`. If a controller accepts a goal
but does not return a result before `result_timeout_sec`, the executor publishes
a deterministic failure with `reason=result_timeout`.

## Validation

With the full demo running:

```bash
bash scripts/check_gazebo_panda_spawned.sh
bash scripts/check_ros2_control_controllers_active.sh
bash scripts/check_full_gazebo_execution_topics.sh
python3 scripts/check_full_gazebo_execution_status.py
```

The topic/action check verifies ROS graph interfaces only. A visible
`/joint_states` topic or `follow_joint_trajectory` action is not proof that the
Gazebo ros2_control controllers are active. Valid PR37 success requires both
controllers to be reported `active` by
`ros2 control list_controllers -c /controller_manager`.

## Troubleshooting

### Missing `/controller_manager/list_controllers`

The controller manager is created by the `gz_ros2_control` model plugin, not a
standalone launch node. Confirm the service and relevant graph entries:

```bash
ros2 service list | grep controller_manager
ros2 node list | grep -E "controller|panda|gz"
ros2 action list | grep follow_joint_trajectory
```

If the service is absent, inspect Gazebo output for plugin load errors. The
bounded controller check prints these diagnostics and exits.

### `gz_ros2_control` plugin not loading

Confirm `ros-jazzy-gz-ros2-control` is installed and Gazebo logs show
`gz_ros2_control::GazeboSimROS2ControlPlugin` loading from the spawned Panda.
The plugin must exist in the expanded `robot_description`; seeing the entity
alone does not prove the plugin loaded.

### Controller YAML path not expanded

Launch passes the installed absolute `panda_ros2_control.yaml` path into xacro
as `controllers_file`. If logs show a literal `$(find ...)` or a missing file,
rebuild and source `install/setup.bash` before relaunching.

### Controller manager namespace mismatch

PR37 uses `/controller_manager`. If the service list shows a namespaced
manager, pass the matching `controller_manager_name` to the simulator launch
and configure the check consistently:

```bash
CONTROLLER_MANAGER=/panda/controller_manager \
  bash scripts/check_ros2_control_controllers_active.sh
```

### Panda falls before controllers are active

Do not add `-r` to the Gazebo arguments. Launch unpauses only after both
controllers are configured, then activates them. If running these steps
manually, do not unpause before controller readiness. Only `panda_link0` is
kinematic; the complete model is not static, so arm motion remains available.

### Panda spawned but the spawn check fails

Gazebo may format its model list as `- panda`. The check accepts that exact
entry after removing only the list marker. Set `ENTITY_NAME` when using a
different `robot_name`.

To validate only terminal-result formatting when dependencies are unavailable
or a controller skip/failure path is expected:

```bash
python3 scripts/check_full_gazebo_execution_status.py --allow-non-success
```

The new launch is discoverable with:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_gazebo_execution_demo.launch.py --show-args
```

## Limitations

This is simulator-only Gazebo execution. It does not add gripper control,
object attach/detach, target-pose-to-Gazebo synchronization, contact-rich
insertion, force control, camera perception, real robot drivers, hardware
execution, or real hardware flags. The Panda model is a lightweight
Panda-compatible kinematic and controller fixture, not a visually exact robot
asset.
