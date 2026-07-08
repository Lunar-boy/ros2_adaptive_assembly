# Plan-only Panda assembly sequence planner

`assembly_sequence_planning_node` plans an ordered list of arm pose stages with
MoveIt2. The default remains the legacy two-stage profile:

```text
pre_grasp -> assembly
```

Set the `stage_names` launch argument to a comma-separated sequence. The
physical pick-place planning profile is:

```text
pre_grasp -> grasp -> lift -> pre_place -> place -> retreat
```

The backward-compatible place profile omits lift:

```text
pre_grasp -> grasp -> pre_place -> place -> retreat
```

The planner waits until every configured pose has been received or refreshed.
It plans the first stage from `start_state_mode=current|fixed`, then plans each
later stage from the final joint state of the preceding successful plan. A
failure stops the sequence and publishes skipped trajectory status for all
remaining stages.

## Configuration

Each stage uses `<stage>_topic` and `<stage>_trajectory_topic` parameters.
Unknown stage names are supported and default to `/panda_<stage>_pose` and
`/<stage>_trajectory`.

| Stage | Pose topic | Trajectory topic |
| --- | --- | --- |
| `pre_grasp` | `/panda_pre_grasp_pose` | `/pre_grasp_trajectory` |
| `grasp` | `/panda_grasp_pose` | `/grasp_trajectory` |
| `lift` | `/panda_lift_pose` | `/lift_trajectory` |
| `assembly` | `/panda_assembly_pose` | `/assembly_trajectory` |
| `pre_place` | `/panda_pre_place_pose` | `/pre_place_trajectory` |
| `place` | `/panda_place_pose` | `/place_trajectory` |
| `retreat` | `/panda_retreat_pose` | `/retreat_trajectory` |

`require_grasp_pose` and `require_place_sequence` remain deprecated node-level
compatibility shims. When `stage_names` is absent, `require_place_sequence=true`
selects the five-stage place profile; otherwise `require_grasp_pose=true`
selects `pre_grasp,grasp,assembly`. An explicit `stage_names` value wins.

Launch examples:

```bash
ros2 launch adaptive_assembly_planning assembly_sequence_planning.launch.py

ros2 launch adaptive_assembly_planning assembly_sequence_planning.launch.py \
  stage_names:=pre_grasp,grasp,lift,pre_place,place,retreat
```

Aggregate status includes `event`, `failed_stage`, `planned_stage_count`,
`requested_stage_count`, `stage_sequence`, `total_duration_ms`,
`start_state_mode`, and `execution=false`. Per-stage and trajectory status add
`stage_index` and `requested_stage_count` so consumers can validate arbitrary
sequences.

## Validation

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 scripts/check_multi_stage_sequence_parameters.py
python3 scripts/check_multi_stage_sequence_status_schema.py
bash scripts/check_assembly_sequence_available.sh
bash scripts/check_sequence_trajectory_topics.sh
```

## Limitations

This node only calls `MoveGroupInterface::plan()` and exports staged arm
trajectories. It never calls `execute()` or `move()`. It provides no trajectory
execution, gripper command, force control, contact sensing, physical grasp
verification, Gazebo contact behavior, or real-hardware support.

PR65 exports arm trajectories only. Gripper open/close interleaving belongs to
PR66.
