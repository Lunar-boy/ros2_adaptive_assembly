# Simulator-only physical pick-place execution

PR66 adds `physical_pick_place_executor_node`, a simulator-only executor that
consumes the PR65 multi-stage arm trajectory exports and interleaves PR63
gripper bridge commands. PR67 adds optional simulator-only contact, lift, and
slip verification after gripper close and after the lift stage.

The default sequence is:

```text
pre_grasp -> grasp -> close gripper -> grasp verification
          -> lift -> lift/slip verification
          -> pre_place -> place -> open gripper -> retreat
```

It subscribes to `moveit_msgs/msg/RobotTrajectory` stages on:

- `/pre_grasp_trajectory`
- `/grasp_trajectory`
- `/lift_trajectory`
- `/pre_place_trajectory`
- `/place_trajectory`
- `/retreat_trajectory`

The stage list is configurable with `stage_names`, defaulting to
`pre_grasp,grasp,lift,pre_place,place,retreat`. The `close_after_stage` and
`open_after_stage` parameters default to `grasp` and `place`; both stages must
be present and `open_after_stage` cannot come before `close_after_stage`.

Arm trajectories are validated before execution. By default, the executor
requires non-empty trajectories with exactly `panda_joint1` through
`panda_joint7`, finite values, valid point field lengths, and non-negative
strictly increasing `time_from_start`. Outgoing arm goals normalize the
trajectory header stamp to zero before sending to the simulated
`/panda_arm_controller/follow_joint_trajectory` action.

Gripper commands are published to `/gripper_command` as semicolon-delimited
`std_msgs/msg/String` messages:

```text
event=command;command=close;source=physical_pick_place_executor;stage=grasp;simulated=true;real_hardware=false
event=command;command=open;source=physical_pick_place_executor;stage=place;simulated=true;real_hardware=false
```

The executor waits for `/physical_gripper_command_status` from
`gripper_action_bridge_node`. `event=success;command=close` and
`event=success;command=open` advance the sequence. If
`require_gripper_success=true`, `event=failure;command=<active_command>` fails
the run. Retained gripper statuses are ignored unless a command is active.

After a successful close command, the executor publishes:

```text
event=request;verification=grasp;stage=grasp;source=physical_pick_place_executor;simulated=true;real_hardware=false
```

After the lift arm stage succeeds, it publishes:

```text
event=request;verification=lift;stage=lift;source=physical_pick_place_executor;simulated=true;real_hardware=false
```

The executor waits up to `verification_timeout_sec` for
`/grasp_verification_status`, `/grasp_verified`, or `/lift_verified`. Required
verification failures terminate with `grasp_verification_failed`,
`grasp_verification_timeout`, `lift_verification_failed`, or
`lift_verification_timeout`. Set `require_grasp_verification=false` and
`require_lift_verification=false` for message-only dry runs that do not depend
on Gazebo contact or pose topics.

Published executor topics:

- `/physical_pick_place_execution_status` (`std_msgs/msg/String`, retained)
- `/physical_pick_place_execution_success` (`std_msgs/msg/Bool`, retained)
- `/physical_pick_place_execution_duration_ms` (`std_msgs/msg/Float64`, retained)
- `/physical_pick_place_stage_status` (`std_msgs/msg/String`, transient local)

All status strings include `mode=physical_pick_place` and
`real_hardware=false`.

Launch the composed simulator-only entry point:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_physical_pick_place_execution.launch.py
```

For a message-only executor dry run, provide trajectories and joint states with
the validation script:

```bash
python3 scripts/check_physical_pick_place_executor_dry_run.py
```

See `docs/gazebo_contact_grasp_verification.md` for the Gazebo contact sensor
plumbing and verifier status schemas. This remains simulator-only. It does not
add force control, tactile feedback, camera perception, MoveIt Servo, learned
grasping, kinematic-attachment success claims, or real hardware execution.
