# Simulator-only gripper action bridge

## Purpose

`panda_gripper_controller` commands the canonical Panda primary finger joint;
the second finger follows through the installed model's mimic relation. It is a
`JointTrajectoryController`. `gripper_action_bridge_node` translates the
existing semicolon-delimited `/gripper_command` open/close interface into
`FollowJointTrajectory` goals.

This is simulator-only actuation. In the physical profile, close completion
may additionally use fresh bilateral Gazebo contact on the configured target.
That evidence establishes only a plausible close contact; the separate grasp
and lift verifier still determines object retention and task progress.

## Interfaces and parameters

The bridge subscribes to `/gripper_command` (`std_msgs/msg/String`) and targets
`/panda_gripper_controller/follow_joint_trajectory`. It publishes retained
status on `/physical_gripper_command_status`, command success on
`/physical_gripper_command_success`, and commanded closed state on
`/physical_gripper_closed`.

Key parameters are `controller_action_name`, `joint_names`, `open_position`,
`close_position`, `goal_time_sec`, `wait_for_controller_sec`,
`result_timeout_sec`, `send_goals`, and `simulated_only`. The defaults command
`panda_finger_joint1` to `0.04` for open and `0.0` for close.
`panda_finger_joint2` follows through the standard Panda URDF mimic relation.
`send_goals=false`
provides deterministic controller-free validation. `simulated_only=false` is
rejected because real hardware is outside this package's scope.

The physical profile adds these bridge parameters:

| Parameter | Physical default | Purpose |
|---|---:|---|
| `open_position` | `0.04` | Primary finger open target in metres |
| `close_position` | `0.0` | Nominal primary finger close target |
| `result_timeout_sec` | `5.0` | Overall accepted action deadline |
| `contact_wait_timeout_sec` | `1.0` | Maximum post-abort evidence wait |
| `contact_freshness_timeout_sec` | `0.25` | Maximum raw contact receipt age |
| `contact_settle_duration_sec` | `0.20` | Continuous bilateral contact interval |
| `allow_contact_limited_close` | `true` | Enable physical close reinterpretation |
| `expected_target_object` | `target_object` | Exact Gazebo model scope token |
| `contact_status_topic` | `/grasp_contact_status` | Normalized contact input |

The reusable bridge default for `allow_contact_limited_close` is `false`, so
legacy launches without contact sensing retain strict action-only behavior.
Opening is always strict action-only behavior.

Close status carries an explicit `result`. Success values are `success` and
`contact_limited_success`. Failures distinguish `goal_rejected`,
`action_aborted`, `action_canceled`, `action_timeout`, `contact_timeout`,
`contact_stale`, `unilateral_contact`, `wrong_object_contact`,
`no_target_contact`, and `internal_error`. Diagnostics include goal acceptance,
action state and error code, expected target, per-finger contacts and ages,
contacted entities, settling duration, and finger positions when feedback
provides them.

Only an accepted close action that ends as `ABORTED` with structured
`FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED` can become
`contact_limited_success`. Human-readable controller error text is diagnostic
only. Other abort codes, cancellation, timeout, rejection, stale or unilateral
contact, and wrong-object contact remain failures.

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
python3 scripts/check_contact_aware_gripper_close_integration.py
```

With the Gazebo Panda demo running, validate controller activation separately:

```bash
bash scripts/check_gripper_controller_active.sh
```
