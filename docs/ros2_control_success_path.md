# ros2_control success-path fixture

PR35 provides a deterministic, simulator-only `FollowJointTrajectory` action
server so the two-stage ros2_control bridge can be verified locally:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_ros2_control_success_demo.launch.py
```

The launch combines the known-reachable Panda sequence planning profile, the
simulated action server, and the existing sequence executor. It returns success
for the pre-grasp stage and then the assembly stage after `result_delay_sec`.
Aggregate status, success, and duration outputs are transient-local, so the
terminal result remains available to late subscribers.

The server node defaults to
`/panda_arm_controller/follow_joint_trajectory`. The composed demo overrides
both server and client to the isolated
`/simulated_panda_arm_controller/follow_joint_trajectory` action so it cannot
collide with a controller from the standard MoveIt Panda demo.

The server parameters are `action_name`, `result_mode` (`success`, `reject`,
`fail`, or `timeout`), `result_delay_sec`, `require_non_empty_trajectory`, and
`expected_joint_prefix`. The executor parameters `result_timeout_sec` and
`cancel_on_timeout` bound every accepted action goal. A timeout produces:

```text
event=failure;mode=ros2_control;stage=<stage>;reason=result_timeout;execution=false;simulated_execution_only=true;real_hardware=false
```

Validate without MoveIt or Gazebo:

```bash
bash scripts/check_ros2_control_success_demo_available.sh
python3 scripts/check_ros2_control_execution_success_path.py
python3 scripts/check_ros2_control_execution_timeout_path.py
```

The optional
`adaptive_assembly_gazebo_ros2_control_success_demo.launch.py` wrapper adds the
PR34 workcell for visualization. Gazebo remains visual only: no Panda is
spawned or moved by the action server. This fixture adds no physical controller,
real hardware support, gripper control, attach/detach lifecycle, force control,
or contact-rich insertion.
