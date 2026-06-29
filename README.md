# ROS2 Adaptive Assembly

A ROS2 Jazzy project for simulated adaptive robotic assembly.

## Target

This project aims to build a minimal, testable adaptive manipulation pipeline:

1. Fake perception publishes randomized target object poses.
2. TF2 broadcasts the object frame.
3. A task node computes pre-grasp and assembly poses.
4. Future versions will add MoveIt2 planning, PlanningScene updates, and
   replanning behavior.

## Environment

- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Python / rclpy
- colcon

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current pipeline

The current non-MoveIt pipeline provides:

- `/target_pose`: randomized target object pose from fake perception
- `/pre_grasp_pose`: task-level pose above the target object
- `/assembly_pose`: task-level pose near the assembly target
- TF `world -> target_object`: transform matching the target object pose

MoveIt2 planning, Gazebo, RViz, robot models, and ros2_control are not launched
by the current pipeline.

## Run the current pipeline

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py
```

## Validate the current pipeline

Start the pipeline in one terminal, then run the validation scripts from another
terminal:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
bash scripts/check_pipeline_topics.sh
python3 scripts/check_pipeline_offsets.py
bash scripts/run_pipeline_validation.sh
bash scripts/echo_pipeline_once.sh
```

See [docs/current_pipeline.md](docs/current_pipeline.md) for the architecture
and validation workflow.

## Optional MoveIt2 Panda demo

If MoveIt2 and the Panda demo resources are installed, the current adaptive
assembly pipeline can be launched alongside the standard Panda MoveIt2 demo:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_demo.launch.py
```

This is a demo bringup only; it does not yet send `/pre_grasp_pose` or
`/assembly_pose` to MoveIt2 for planning. See
[docs/moveit2_panda_demo.md](docs/moveit2_panda_demo.md).

## Plan-only MoveIt2 planning bridge

The plan-only bridge launches the current adaptive assembly pipeline, the Panda
MoveIt2 demo, and a minimal node that asks MoveIt2 to plan to the adapted
pre-grasp pose:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

This launch plans only. It does not execute robot motion, use Gazebo, or require
real hardware. See
[docs/pre_grasp_planning_bridge.md](docs/pre_grasp_planning_bridge.md).

## Plan-only Panda assembly sequence

The sequence demo adds a Panda-aware `/panda_assembly_pose` path and plans two
stages: `pre_grasp`, then `assembly`. The assembly-stage plan starts from the
final joint state of the pre-grasp plan. Neither trajectory is executed.

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_sequence_planning_demo.launch.py
```

Validation helpers:

```bash
bash scripts/check_assembly_sequence_available.sh
bash scripts/check_assembly_sequence_topics.sh
python3 scripts/check_assembly_sequence_status.py
```

The existing Panda planning demo keeps its single pre-grasp behavior by
default. See
[docs/assembly_sequence_planner.md](docs/assembly_sequence_planner.md).

For local environments without a stable Panda current-state source, the
fixed-start profile provides deterministic plan-only sequence validation:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_fixed_start.launch.py
```

The default sequence launch still uses `start_state_mode=current`. The fixed
profile uses a known Panda joint configuration as planning input only; it does
not command or execute robot motion.

## Panda-adapted pre-grasp planning

`/pre_grasp_pose` is task-level. `/panda_pre_grasp_pose` is robot-specific and
uses a configurable Panda-oriented end-effector orientation. The Panda planning
demo uses `/panda_pre_grasp_pose` as the planning bridge input.
By default, `/panda_pre_grasp_pose` uses frame `panda_link0` for compatibility
with the standard Panda MoveIt2 demo.

See [docs/panda_pre_grasp_pose_adapter.md](docs/panda_pre_grasp_pose_adapter.md).

## Optional TF2 Panda pose adapter mode

The Panda pose adapter can optionally transform incoming poses into the Panda
planning frame with TF2. The default launch behavior remains unchanged:
`use_tf_transform` is `false`, so the adapter keeps the existing frame override
and numeric-copy behavior. The adapter now also publishes
`/panda_pose_adapter_status`.

Validation helpers:

```bash
bash scripts/check_panda_pose_adapter_tf2_params.sh
python3 scripts/check_panda_pose_adapter_status.py
```

See [docs/tf2_pose_adapter.md](docs/tf2_pose_adapter.md).

## RViz marker visualization

The Panda planning demo publishes lightweight pose markers by default:

- `/target_pose`: sphere
- `/pre_grasp_pose`: arrow
- `/assembly_pose`: cube
- `/panda_pre_grasp_pose`: arrow

Marker topics:

- `/adaptive_assembly_markers`
- `/adaptive_assembly_marker_status`

Validation helpers:

```bash
bash scripts/check_adaptive_assembly_markers_available.sh
python3 scripts/check_adaptive_assembly_marker_status.py
```

This is visualization-only; it does not modify the PlanningScene or execute
trajectories. See [docs/rviz_markers.md](docs/rviz_markers.md).

## Static PlanningScene objects

The Panda planning demo also starts a static PlanningScene node that applies
simple collision objects for the table/workcell:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

This adds static collision objects only and still does not execute trajectories.
The static objects can also be reset without restarting the demo through:

- `/clear_static_planning_scene`
- `/reapply_static_planning_scene`
- `/static_planning_scene_status`

Validation helpers:

```bash
bash scripts/check_static_planning_scene_services.sh
bash scripts/clear_static_planning_scene_once.sh
bash scripts/reapply_static_planning_scene_once.sh
python3 scripts/check_static_planning_scene_status.py
```

See [docs/static_planning_scene.md](docs/static_planning_scene.md).

## Planning diagnostics

The plan-only bridge publishes compatibility and diagnostic topics:

- `/pre_grasp_plan_success`
- `/pre_grasp_planning_status`
- `/pre_grasp_planning_duration_ms`

Validation helpers:

```bash
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
```

See [docs/planning_diagnostics.md](docs/planning_diagnostics.md).

## Planning request guard

The plan-only bridge can optionally reject invalid requests before calling
MoveIt2. The guard is disabled by default. Guarded status messages include:

- `guard_enabled`
- `guard_passed`
- `guard_reason`

Validate guard metadata:

```bash
bash scripts/check_guarded_benchmark_launch.sh
bash scripts/check_planning_request_guard_status.sh
```

Run the guarded seeded benchmark profile:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_guarded.launch.py
```

See [docs/planning_request_guard.md](docs/planning_request_guard.md).

## Planner settings for benchmarking

The plan-only MoveIt2 bridge exposes planner settings for benchmark metadata:

- `planner_id`
- `num_planning_attempts`
- `max_velocity_scaling_factor`
- `max_acceleration_scaling_factor`

These fields are published in `/pre_grasp_planning_status`, recorded in
benchmark CSV output, and included in Markdown report metadata when available.

Validation helper:

```bash
bash scripts/check_planner_parameter_status.sh
```

See [docs/planner_settings.md](docs/planner_settings.md).

## Planner-settings benchmark profiles

Deterministic benchmark profiles compare different planner settings:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts1.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts5.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_slow_scaling.launch.py
```

Validate installed profiles:

```bash
bash scripts/check_planner_benchmark_profiles.sh
```

Manual workflow hint:

```bash
bash scripts/run_planner_settings_benchmark_hint.sh
```

See
[docs/planner_settings_benchmark_profiles.md](docs/planner_settings_benchmark_profiles.md).

## Planning benchmark recording

Planning diagnostics can be recorded to CSV and summarized into
success/failure/skipped counts plus duration statistics:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/planning_diagnostics.csv bash scripts/run_planning_benchmark.sh
```

See [docs/planning_benchmark.md](docs/planning_benchmark.md).

## Reproducible benchmark profile

The seeded benchmark profile uses deterministic fake perception with
`random_seed: 42`. Default demo behavior remains unchanged and
non-deterministic.

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py
```

Record the seeded benchmark:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/seeded_planning_diagnostics.csv bash scripts/run_seeded_planning_benchmark.sh
```

See
[docs/reproducible_benchmark_profile.md](docs/reproducible_benchmark_profile.md).

## Benchmark profile suite

The benchmark suite provides deterministic baseline, narrow, wide, and
fixed-yaw target-pose profiles for comparing planning diagnostics CSV files.

Validate the installed profile suite:

```bash
bash scripts/check_benchmark_profile_suite.sh
bash scripts/check_benchmark_profile_params.sh
```

Compare recorded benchmark CSV files:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input baseline=benchmark_results/baseline.csv \
  --input narrow=benchmark_results/narrow.csv \
  --input wide=benchmark_results/wide.csv \
  --input fixed_yaw=benchmark_results/fixed_yaw.csv
```

See [docs/benchmark_profile_suite.md](docs/benchmark_profile_suite.md).

## Benchmark report export

Benchmark CSV comparisons can also be exported as Markdown:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input no_dynamic=benchmark_results/no_dynamic_target.csv \
  --input with_dynamic=benchmark_results/with_dynamic_target.csv \
  --output-markdown benchmark_results/dynamic_target_ab_report.md
```

Validate report export:

```bash
bash scripts/check_benchmark_report_export.sh
```

See [docs/benchmark_report_export.md](docs/benchmark_report_export.md).

## Dynamic target PlanningScene object

The Panda planning demo now starts a dynamic target scene node. It subscribes
to `/panda_pre_grasp_pose`, applies or updates the `target_object_dynamic`
collision object, and publishes:

- `/dynamic_target_scene_ready`
- `/dynamic_target_scene_status`
- `/clear_dynamic_target_scene`

Validation helpers:

```bash
bash scripts/check_dynamic_target_scene_available.sh
bash scripts/check_dynamic_target_scene_ready.sh
python3 scripts/check_dynamic_target_scene_status.py
bash scripts/check_dynamic_target_clear_service.sh
bash scripts/clear_dynamic_target_scene_once.sh
```

The clear service removes `target_object_dynamic` and resets the ready flag,
which is useful before repeated demos or benchmark comparisons. This remains
plan-only; no trajectory execution is added. See
[docs/dynamic_target_scene.md](docs/dynamic_target_scene.md).

Static and dynamic PlanningScene objects can now be reset independently.

## PlanningScene audit

The Panda planning demo starts a read-only audit node by default. It checks
whether expected collision objects are visible in MoveIt2:

- `work_table`
- `target_support`
- `target_object_dynamic`

Published audit topics:

- `/planning_scene_audit_ready`
- `/planning_scene_audit_status`

Validation helpers:

```bash
bash scripts/check_planning_scene_audit_available.sh
python3 scripts/check_planning_scene_audit_status.py
bash scripts/check_planning_scene_audit_ready.sh
```

This is introspection-only; it does not modify the PlanningScene or execute
trajectories. See [docs/planning_scene_audit.md](docs/planning_scene_audit.md).

## Unified PlanningScene reset workflow

Before repeated demos or A/B benchmark recording, the scene can be reset with:

```bash
bash scripts/reset_planning_scene_once.sh
```

This clears the dynamic target object, clears static objects, and reapplies the
static table/support objects. The dynamic target may be recreated automatically
when new `/panda_pre_grasp_pose` messages arrive.

Validate the workflow scripts:

```bash
bash scripts/check_planning_scene_reset_workflow.sh
```

See
[docs/planning_scene_reset_workflow.md](docs/planning_scene_reset_workflow.md).

## Dynamic target A/B benchmark

The dynamic target scene can be enabled or disabled from launch for benchmark
comparison:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_no_dynamic_target.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_with_dynamic_target.launch.py
bash scripts/run_dynamic_target_ab_benchmark_hint.sh
```

See
[docs/dynamic_target_ab_benchmark.md](docs/dynamic_target_ab_benchmark.md).

## Roadmap

- PR1: fake perception node
- PR2: task pose generation node
- PR3: bringup launch for the non-MoveIt pipeline
- PR4: validation scripts and documentation cleanup
- PR5: optional Panda MoveIt2 demo bringup
- PR6: plan-only MoveIt2 pre-grasp planning bridge
- PR7: Panda pre-grasp pose adapter for robot-aware planning targets
- PR8: frame-aware Panda pre-grasp pose adapter
- PR9: static PlanningScene collision objects for Panda planning demo
- PR10: planning diagnostics and timing topics
- PR11: planning diagnostics CSV benchmark recorder
- PR12: reproducible seeded planning benchmark profile
- PR13: deterministic benchmark profile suite and CSV comparison tools
- PR14: dynamic target collision object in PlanningScene
- PR15: dynamic target PlanningScene toggle and A/B benchmark profiles
- PR16: dynamic target PlanningScene clear/reset service
- PR17: static PlanningScene clear/reapply services
- PR18: unified PlanningScene reset workflow
- PR19: Markdown benchmark report export
- PR20: configurable MoveIt2 planner settings for benchmarks
- PR21: planner-settings benchmark profiles
- PR22: TF2-based Panda pose adapter with status diagnostics
- PR23: planning request guard and safety filter
- PR24: PlanningScene object audit tool
- PR25: simple RViz marker visualization
- PR26: plan-only Panda pre-grasp and assembly sequence planning
- PR27: deterministic fixed-start assembly sequence planning fallback
- Future PR: assembly sequence diagnostics and planning refinements
