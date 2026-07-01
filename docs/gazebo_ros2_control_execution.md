# Gazebo/ros2_control execution bridge

The optional ros2_control profile is the first simulator execution bridge for
the exported two-stage Panda sequence. It consumes the existing
`moveit_msgs/msg/RobotTrajectory` messages, validates them, and sends their
`joint_trajectory` fields to a `control_msgs/action/FollowJointTrajectory`
server in this order:

1. `pre_grasp`
2. `assembly`

The assembly goal is sent only after the pre-grasp goal is accepted and returns
success. Aggregate success is true only when both goals are accepted and both
results report `FollowJointTrajectory` success.

## Scope and prerequisites

PR35 adds a deterministic local action server at
`/panda_arm_controller/follow_joint_trajectory` by default. It validates the
action-level success and timeout paths, but it is not a ros2_control hardware
plugin or physical Gazebo controller. The bridge can still connect to an
externally running compatible simulated controller.

The profile is simulator-only. The `simulated_execution_only` parameter must
remain `true`, and all terminal statuses include `real_hardware=false`. This
profile adds no real hardware execution, gripper control, force control,
grasp/contact physics, contact-rich insertion, or industrial workcell model.

## Launch

Build and source the workspace, start the compatible Gazebo/ros2_control Panda
stack separately if one is available, then run:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_ros2_control_execution.launch.py
```

For the self-contained success fixture, run:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_ros2_control_success_demo.launch.py
```

The optional `adaptive_assembly_gazebo_ros2_control_success_demo.launch.py`
adds the PR34 world. That world is visual only; the action fixture does not
spawn or physically move a Panda in Gazebo.

The launch composes the deterministic known-reachable sequence planning
profile and the execution bridge. If the configured action server is missing,
the process remains alive and publishes a deterministic skipped result after
`wait_for_controller_sec`.

To exercise validation without sending controller goals:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_ros2_control_execution.launch.py \
  send_goals:=false
```

## Interfaces

Inputs:

- `/pre_grasp_trajectory` (`moveit_msgs/msg/RobotTrajectory`)
- `/assembly_trajectory` (`moveit_msgs/msg/RobotTrajectory`)

Outputs:

- `/assembly_ros2_control_execution_status` (`std_msgs/msg/String`)
- `/assembly_ros2_control_execution_success` (`std_msgs/msg/Bool`)
- `/assembly_ros2_control_execution_duration_ms` (`std_msgs/msg/Float64`)
- `/assembly_ros2_control_execution_stage_status` (`std_msgs/msg/String`)

Aggregate outputs use transient-local durability. Stage events report accepted
goals and successful results. The bridge processes the first pair of
trajectories once and ignores later trajectory messages.

## Parameters

| Parameter | Default |
| --- | --- |
| `pre_grasp_trajectory_topic` | `/pre_grasp_trajectory` |
| `assembly_trajectory_topic` | `/assembly_trajectory` |
| `controller_action_name` | `/panda_arm_controller/follow_joint_trajectory` |
| `status_topic` | `/assembly_ros2_control_execution_status` |
| `success_topic` | `/assembly_ros2_control_execution_success` |
| `duration_topic` | `/assembly_ros2_control_execution_duration_ms` |
| `stage_status_topic` | `/assembly_ros2_control_execution_stage_status` |
| `wait_for_controller_sec` | `5.0` |
| `result_timeout_sec` | `10.0` |
| `cancel_on_timeout` | `true` |
| `send_goals` | `true` |
| `require_non_empty_trajectory` | `true` |
| `require_panda_joints` | `true` |
| `expected_joint_prefix` | `panda_joint` |
| `simulated_execution_only` | `true` |

With default validation, both trajectories require non-empty joint names,
non-empty points, and at least one joint beginning with `panda_joint`.

Terminal status examples include:

```text
event=skipped;mode=ros2_control;reason=controller_unavailable;controller=/panda_arm_controller/follow_joint_trajectory;execution=false;simulated_execution_only=true;real_hardware=false
event=success;mode=ros2_control;stage_count=2;duration_ms=...;execution=true;simulated_execution_only=true;real_hardware=false
event=failure;mode=ros2_control;stage=assembly;reason=action_result_failed;execution=false;simulated_execution_only=true;real_hardware=false
event=failure;mode=ros2_control;stage=pre_grasp;reason=result_timeout;execution=false;simulated_execution_only=true;real_hardware=false
```

## Validation

With the launch running, interface and terminal status checks are:

```bash
bash scripts/check_ros2_control_execution_topics.sh
python3 scripts/check_ros2_control_execution_status.py
```

The missing-controller check is self-contained after the workspace is built and
sourced. It starts an isolated bridge, supplies two synthetic valid Panda
trajectories, and verifies the skipped result without Gazebo or MoveIt:

```bash
python3 scripts/check_ros2_control_execution_unavailable_path.py
```

The local success and timeout checks use the simulated action server and do not
require MoveIt or Gazebo:

```bash
bash scripts/check_ros2_control_success_demo_available.sh
python3 scripts/check_ros2_control_execution_success_path.py
python3 scripts/check_ros2_control_execution_timeout_path.py
```

Each accepted goal has a `result_timeout_sec` watchdog. On expiry, the bridge
optionally requests cancellation (`cancel_on_timeout`) and immediately
publishes the deterministic terminal `reason=result_timeout` failure. Late
action results cannot replace that retained terminal result.
