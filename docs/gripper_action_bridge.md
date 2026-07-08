# Simulator-only gripper action bridge

## Purpose

PR63 adds controller-level actuation for the two independent Panda finger
joints introduced in PR62. `panda_gripper_controller` is a two-joint
`JointTrajectoryController`. `gripper_action_bridge_node` translates the
existing semicolon-delimited `/gripper_command` open/close interface into
`FollowJointTrajectory` goals.

This is simulator-only actuation. Controller success does not establish
contact, object attachment, lift, slip resistance, or physical grasp success.

## Interfaces and parameters

The bridge subscribes to `/gripper_command` (`std_msgs/msg/String`) and targets
`/panda_gripper_controller/follow_joint_trajectory`. It publishes retained
status on `/physical_gripper_command_status`, command success on
`/physical_gripper_command_success`, and commanded closed state on
`/physical_gripper_closed`.

Key parameters are `controller_action_name`, `joint_names`, `open_position`,
`close_position`, `goal_time_sec`, `wait_for_controller_sec`,
`result_timeout_sec`, `send_goals`, and `simulated_only`. The defaults command
both finger joints to `0.04` for open and `0.0` for close. `send_goals=false`
provides deterministic controller-free validation. `simulated_only=false` is
rejected because real hardware is outside this package's scope.

## Manual demo

From `~/ros2_adaptive_assembly_ws` after building and sourcing:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_gripper_controller_demo.launch.py
ros2 topic pub --once /gripper_command std_msgs/msg/String "{data: 'event=command;command=close;gripper=panda_hand;simulated=true;real_hardware=false'}"
ros2 topic pub --once /gripper_command std_msgs/msg/String "{data: 'event=command;command=open;gripper=panda_hand;simulated=true;real_hardware=false'}"
```

## Validation

```bash
python3 scripts/check_panda_gripper_urdf_contains_fingers.py
python3 scripts/check_gripper_controller_config.py
python3 scripts/check_gripper_action_bridge_static.py
python3 scripts/check_gripper_action_bridge_dry_run.py
```

With the Gazebo Panda demo running, validate controller activation separately:

```bash
bash scripts/check_gripper_controller_active.sh
```
