# Contact-lite Insertion Benchmark

This benchmark adds deterministic geometric insertion evaluation to the
existing adaptive assembly pipeline. It checks whether a target insertion pose
and an
achieved insertion pose are within configurable position and orientation
tolerances.

This is contact-lite evaluation only. It does not add force control, tactile
sensing, real contact-rich peg-in-hole behavior, real hardware execution, Isaac
Sim, Gazebo contact physics, or high-fidelity contact plugins.

## Node

`contact_lite_insertion_evaluator_node` is provided by
`adaptive_assembly_benchmark`.

Inputs:

- `/assembly_sequence_planning_status` (`std_msgs/msg/String`)
- `/assembly_ros2_control_execution_status` (`std_msgs/msg/String`)
- `/assembly_ros2_control_execution_success` (`std_msgs/msg/Bool`)
- target insertion pose, default `/panda_assembly_pose`
  (`geometry_msgs/msg/PoseStamped`)
- achieved insertion pose, default `/panda_assembly_pose`
  (`geometry_msgs/msg/PoseStamped`)

When execution feedback is unavailable, the default configuration treats the
planned Panda assembly pose as the achieved pose. The status explicitly reports
`achieved_pose_source=planned_pose`.

Outputs:

- `/assembly_insertion_status` (`std_msgs/msg/String`)
- `/assembly_insertion_success` (`std_msgs/msg/Bool`)
- `/assembly_insertion_error_mm` (`std_msgs/msg/Float64`)
- `/assembly_insertion_error_deg` (`std_msgs/msg/Float64`)

The status string uses semicolon-delimited key/value fields and includes
`event`, `mode=contact_lite_insertion`, `success`, position/orientation errors,
tolerances, execution gating, achieved pose source, and
`real_hardware=false`.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `target_pose_topic` | `/panda_assembly_pose` | Target insertion pose topic |
| `achieved_pose_topic` | `/panda_assembly_pose` | Achieved pose topic |
| `position_tolerance_mm` | `5.0` | Maximum position error |
| `orientation_tolerance_deg` | `5.0` | Maximum quaternion angular error |
| `require_execution_success` | `false` | Gate on execution success |
| `achieved_pose_source` | `planned_pose` | Achieved source status label |
| `status_topic` | `/assembly_insertion_status` | Status output topic |
| `success_topic` | `/assembly_insertion_success` | Success output topic |
| `position_error_topic` | `/assembly_insertion_error_mm` | Position error topic |
| `orientation_error_topic` | `/assembly_insertion_error_deg` | Orientation error topic |

## Success Rule

Insertion succeeds only when the target and achieved poses are available, both
pose errors are within tolerance, and execution success is true when
`require_execution_success=true`.

Orientation error is computed as quaternion angular distance in degrees with a
clamped dot product before `acos`.

## Launch

Run the benchmark on top of the deterministic known-reachable plan-only
sequence:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_contact_lite_insertion_benchmark.launch.py
```

By default this does not execute trajectories. It evaluates the planned
`/panda_assembly_pose` as both target and achieved pose.

## CSV and Markdown Reports

Record insertion status events and export a Markdown summary:

```bash
MAX_TRIALS=20 TIMEOUT_SEC=120 \
  bash scripts/run_contact_lite_insertion_benchmark.sh
```

The CSV records trial ID, timestamp, success, position/orientation errors,
tolerances, execution gating, achieved pose source, and raw status. The
Markdown summary reports total trials, success/failure counts, success rate,
mean/max errors, tolerance settings, and the contact-lite limitation.

## Validation

Focused checks:

```bash
bash scripts/check_contact_lite_insertion_topics.sh
python3 scripts/check_contact_lite_insertion_success_path.py
python3 scripts/check_contact_lite_insertion_failure_path.py
bash scripts/check_contact_lite_insertion_report_export.sh
```

The success and failure path checks use synthetic `PoseStamped` messages and do
not require Gazebo, contact physics, controllers, or real hardware.
