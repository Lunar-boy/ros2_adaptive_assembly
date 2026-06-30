# Dry-run assembly sequence execution

The `adaptive_assembly_execution` package provides a one-shot execution
abstraction for exported plan-only sequence trajectories. The executor consumes
`/pre_grasp_trajectory` and `/assembly_trajectory`, validates both messages, and
simulates the stages in order without commanding a robot.

## Interfaces

Inputs:

- `/pre_grasp_trajectory` (`moveit_msgs/msg/RobotTrajectory`)
- `/assembly_trajectory` (`moveit_msgs/msg/RobotTrajectory`)

Aggregate outputs:

- `/assembly_execution_status` (`std_msgs/msg/String`)
- `/assembly_execution_success` (`std_msgs/msg/Bool`)
- `/assembly_execution_duration_ms` (`std_msgs/msg/Float64`)

Optional per-stage output:

- `/assembly_execution_stage_status` (`std_msgs/msg/String`)

Aggregate outputs use transient-local durability so a validator or observer can
read the one retained result after the dry run completes.

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `pre_grasp_trajectory_topic` | `/pre_grasp_trajectory` | Pre-grasp input |
| `assembly_trajectory_topic` | `/assembly_trajectory` | Assembly input |
| `status_topic` | `/assembly_execution_status` | Aggregate status output |
| `success_topic` | `/assembly_execution_success` | Aggregate success output |
| `duration_topic` | `/assembly_execution_duration_ms` | Aggregate duration output |
| `require_panda_joints` | `true` | Require an expected joint prefix |
| `expected_joint_prefix` | `panda_joint` | Expected joint-name prefix |
| `simulate_real_time` | `false` | Sleep during each simulated stage |
| `simulated_stage_duration_ms` | `10.0` | Per-stage sleep when enabled |
| `publish_stage_status` | `true` | Publish stage-level status |

Each trajectory must contain joint names and trajectory points. With Panda
joint validation enabled, at least one joint name must begin with
`expected_joint_prefix`. The executor simulates `pre_grasp` before `assembly`
and publishes aggregate success only after both stages validate.

`execution=true` in a successful status means the dry-run abstraction processed
both stages. `real_execution=false` states that no trajectory was sent to
MoveIt, a controller, a simulator, or hardware.

## Run and validate

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_dry_run_execution.launch.py
```

In another sourced shell:

```bash
bash scripts/check_dry_run_execution_topics.sh
python3 scripts/check_dry_run_execution_status.py
```

This launch composes the deterministic known-reachable plan-only sequence. It
does not add Gazebo, `ros2_control`, controller APIs, hardware interfaces, or
real trajectory execution.
