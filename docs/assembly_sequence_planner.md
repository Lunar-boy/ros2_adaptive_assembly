# Plan-only Panda assembly sequence planner

The assembly sequence path extends the single pre-grasp planning pipeline with
a second robot-aware target and a two-stage plan-only workflow. It does not
execute either trajectory.

## Data flow

```text
/pre_grasp_pose                         /assembly_pose
      |                                      |
      v                                      v
panda_pre_grasp_pose_adapter_node      panda_assembly_pose_adapter_node
      |                                      |
      v                                      v
/panda_pre_grasp_pose                  /panda_assembly_pose
      |                                      |
      +------------------+-------------------+
                         v
            assembly_sequence_planning_node
                         |
                         +-- plan pre_grasp from current state
                         +-- plan assembly from pre_grasp plan end state
                         |
                         +-- /assembly_sequence_plan_success
                         +-- /assembly_sequence_planning_status
                         +-- /assembly_sequence_planning_duration_ms
```

The Panda assembly adapter mirrors the pre-grasp adapter defaults: it copies
the numeric task pose into frame `panda_link0`, applies the fixed quaternion
`(1, 0, 0, 0)`, and can optionally use TF2 instead of frame override. Adapter
events are published on `/panda_assembly_pose_adapter_status`.

The sequence planner waits until both adapted inputs have been refreshed. It
plans `pre_grasp` first. If that succeeds, the final joint positions from that
plan become the start state for the `assembly` plan. `execute()` and `move()`
are never called.

Sequence status contains:

- `event`: `success` or `failure`
- `failed_stage`: `none`, `pre_grasp`, or `assembly`
- `planned_stage_count`: number of successful stages
- `total_duration_ms`: wall-clock planning time across attempted stages
- `start_state_mode`: `current` or `fixed`
- `execution=false`

## Build and run

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_sequence_planning_demo.launch.py
```

The dedicated launch disables the existing single-pose pre-grasp planner and
starts the assembly adapter and sequence planner instead. The regular
`adaptive_assembly_panda_planning_demo.launch.py` still starts the original
pre-grasp planner by default.

Planner settings can be overridden at launch:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_sequence_planning_demo.launch.py \
  planner_id:="" num_planning_attempts:=1 planning_time_sec:=5.0 \
  position_tolerance:=0.01 orientation_tolerance:=0.10 \
  publish_diagnostics:=true
```

## Deterministic fixed-start fallback

The default `start_state_mode` is `current`, preserving the normal behavior of
requesting the current Panda state before planning `pre_grasp`. Some local
Panda demo installations do not provide a stable current joint state because
their controller/current-state plugin is unavailable.

For reproducible plan-only validation, use the fixed-start profile:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_fixed_start.launch.py
```

This sets `panda_joint1` through `panda_joint7` to the deterministic planning
configuration `[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` before the
pre-grasp planning request. The assembly stage still starts from the final
joint state of the successful pre-grasp plan. The fixed state is planning input
only and is never commanded or executed.

## Validate

With the sequence demo running:

```bash
bash scripts/check_assembly_sequence_available.sh
bash scripts/check_assembly_sequence_topics.sh
python3 scripts/check_assembly_sequence_status.py
bash scripts/check_assembly_sequence_fixed_start_launch.sh
bash scripts/check_assembly_sequence_start_state_status.sh
ros2 topic echo /panda_assembly_pose
ros2 topic echo /assembly_sequence_planning_status
```

This feature is plan-only. It adds no Gazebo, `ros2_control`, real hardware,
or trajectory execution support.

## Trajectory export

By default, each successful stage exports its complete
`moveit_msgs/msg/RobotTrajectory` for future consumers:

- `/pre_grasp_trajectory`
- `/assembly_trajectory`
- `/assembly_sequence_trajectory_status` (`std_msgs/msg/String`)

The assembly trajectory is published only when assembly planning succeeds. Set
`publish_trajectories:=false` to disable both trajectory publishers. Topic names
are configurable with `pre_grasp_trajectory_topic`,
`assembly_trajectory_topic`, and `trajectory_status_topic`. Status events report
the stage, topic, available point and joint counts, and always include
`execution=false`.

With the deterministic known-reachable profile running, validate the export:

```bash
bash scripts/check_sequence_trajectory_topics.sh
python3 scripts/check_sequence_trajectory_status.py
```

Exporting a trajectory does not command or execute it.

## Dry-run execution consumer

The exported trajectories can be validated and processed by the message-only
dry-run executor:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_dry_run_execution.launch.py
```

This abstraction simulates the two stage transitions but sends no commands and
performs no real execution. See
[dry_run_execution.md](dry_run_execution.md) for its interfaces and validation.

## Known-reachable sequence profile

The known-reachable profile combines the PR27 fixed Panda start state with a
fixed task target and known working planning settings. It exists to provide
repeatable successful coverage of both planning stages without depending on a
controller/current-state plugin:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_reachable.launch.py
```

The profile uses target `(x=0.442, y=0.148, z=0.15, yaw=0.0)`, random seed `42`,
one planning attempt, a 5-second per-stage planning limit, `0.01` position
tolerance, and `0.10` orientation tolerance. The target is fixed by setting
equal minimum and maximum x/y/yaw values.

The profile disables the optional dynamic target collision object because that
box occupies the fixed assembly goal itself. Static table/support collision
checking remains active. The normal sequence demo still enables the dynamic
target scene by default.

Validate installation and the successful second-stage path:

```bash
bash scripts/check_reachable_sequence_profile_available.sh
python3 scripts/check_assembly_sequence_success_path.py
```

Success requires `event=success`, `failed_stage=none`,
`planned_stage_count=2`, `start_state_mode=fixed`, and `execution=false`.
This profile improves plan-only reproducibility and coverage; it does not
command or execute either planned trajectory.

## Stage-level diagnostics

Each attempted sequence stage publishes an independent result:

- `/assembly_sequence_stage_status` (`std_msgs/msg/String`)
- `/assembly_sequence_stage_success` (`std_msgs/msg/Bool`)
- `/assembly_sequence_stage_duration_ms` (`std_msgs/msg/Float64`)

Stage status identifies `pre_grasp` or `assembly` and includes the individual
duration, planner settings, tolerances, start-state mode, and
`execution=false`. A pre-grasp failure emits only the pre-grasp result. An
assembly failure follows a successful pre-grasp result with an assembly failure
result. Existing aggregate sequence topics and status fields are unchanged.

With the known-reachable profile running:

```bash
bash scripts/check_assembly_sequence_stage_diagnostics.sh
python3 scripts/check_assembly_sequence_stage_status.py
```

When `publish_diagnostics=false`, stage String/Float64 messages are suppressed,
but per-stage Bool results remain available. These diagnostics are plan-only;
they do not execute either trajectory.
