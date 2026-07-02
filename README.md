# ROS2 Adaptive Assembly

A ROS2 Jazzy + MoveIt2 project for perception-driven adaptive robotic assembly in simulation.

This repository builds a lightweight adaptive manipulation pipeline that converts randomized object poses into robot-aware Panda planning targets, maintains TF and PlanningScene state, performs plan-only two-stage assembly planning, exports trajectories, records planning diagnostics, and provides deterministic benchmark profiles for evaluating robustness under target-pose variation.

> **Current scope:** reproducible adaptive assembly planning plus simulator-only Gazebo Panda arm execution.
> **Not yet included:** real camera hardware, contact-rich insertion, force
> control, or real robot hardware execution.

---

## Why this project

Industrial assembly systems are brittle when object poses, fixtures, tolerances, or workspace conditions deviate from fixed assumptions. This project explores a small, testable ROS2 architecture for adaptive manipulation, where target poses are generated from perception, transformed into robot-specific planning frames, checked against a MoveIt2 PlanningScene, and evaluated through reproducible planning benchmarks.

The project is intentionally designed as a lightweight software stack rather than a high-fidelity simulator-first demo. This keeps the pipeline portable to local machines while preserving clear extension points for Gazebo, `ros2_control`, perception modules, and real robot hardware interfaces.

---

## Key features

- ROS2 Jazzy workspace with modular Python packages
- Fake perception node publishing randomized target object poses
- Simulated marker-pose perception emulator (simulator-only)
- TF2 target frame broadcasting
- Task-level pre-grasp and assembly pose generation
- Panda-specific pose adapters for MoveIt2 planning
- Plan-only MoveIt2 pre-grasp planning
- Plan-only two-stage `pre_grasp -> assembly` sequence planning
- Static PlanningScene collision objects
- Dynamic target PlanningScene object
- PlanningScene audit and reset workflows
- Planning request guard and safety filter
- Stage-level planning diagnostics
- Trajectory export for downstream execution consumers
- Message-only dry-run execution abstraction
- Closed-loop recovery supervisor with deterministic recovery actions
- Deterministic benchmark profiles and CSV/Markdown report export
- Contact-lite geometric insertion benchmark and report export
- Lightweight Gazebo Harmonic workcell visualization
- Simulator-only Gazebo Panda arm execution through ros2_control
- Simulator-only Gazebo target object synchronization from `/target_pose`

---

## System architecture

```text
fake_object_pose_node
    ├── /target_pose
    └── TF: world -> target_object
              │
              ▼
assembly_task_node
    ├── /pre_grasp_pose
    └── /assembly_pose
              │
              ▼
panda_pose_adapter_node
    ├── /panda_pre_grasp_pose
    └── /panda_assembly_pose
              │
              ▼
MoveIt2 sequence planner
    ├── pre-grasp plan
    ├── assembly plan
    ├── trajectory export
    └── diagnostics / benchmark CSV
              │
              ▼
dry-run execution / simulator-only ros2_control execution
              │
              ▼
contact-lite insertion evaluator
```

The current pipeline separates perception, task-level pose generation, robot-specific pose adaptation, planning, diagnostics, and execution abstraction. This makes the system easy to test incrementally and extend toward simulator or hardware execution.

---

## Current capability matrix

| Area | Status |
|---|---|
| Fake perception | Implemented |
| Simulated marker-pose perception emulator | Implemented / simulator-only |
| TF2 target broadcasting | Implemented |
| Task-level pose generation | Implemented |
| Panda-specific pose adaptation | Implemented |
| Optional TF2 pose adapter mode | Implemented |
| MoveIt2 plan-only pre-grasp planning | Implemented |
| MoveIt2 two-stage `pre_grasp -> assembly` planning | Implemented |
| Static PlanningScene objects | Implemented |
| Dynamic target PlanningScene object | Implemented |
| PlanningScene audit/reset | Implemented |
| Planning request guard | Implemented |
| Planning diagnostics | Implemented |
| CSV benchmark recording | Implemented |
| Markdown benchmark reports | Implemented |
| Contact-lite insertion benchmark | Implemented |
| Trajectory export | Implemented |
| Message-only dry-run execution | Implemented |
| Recovery supervisor | Implemented |
| Gazebo Harmonic workcell visualization | Implemented |
| ros2_control trajectory bridge | Implemented / simulator-only |
| Full Gazebo Panda arm execution | Implemented / simulator-only |
| Gazebo target object synchronization | Implemented / simulator-only |
| Gripper control and object attachment | Implemented / simulator-only |
| Contact-rich insertion | Not implemented |
| Real camera hardware and force control | Not implemented |
| Real robot execution | Not implemented |

---

## Repository layout

```text
ros2_adaptive_assembly/
├── docs/                         # Design notes, feature documentation, validation workflows
├── scripts/                      # Validation, benchmark, and utility scripts
├── benchmark_results/            # Optional benchmark CSV/Markdown outputs
└── src/
    ├── adaptive_assembly_bringup/     # Launch files and integration entry points
    ├── adaptive_assembly_benchmark/   # Contact-lite geometric benchmark nodes
    ├── adaptive_assembly_execution/   # Dry-run and execution-bridge abstractions
    ├── adaptive_assembly_perception/  # Fake perception and target-pose generation
    ├── adaptive_assembly_recovery/    # Recovery supervisor and deterministic actions
    ├── adaptive_assembly_sim/         # Gazebo workcell assets and launch files
    └── adaptive_assembly_task/        # Task pose generation and planning adapters
```

---

## Environment

Tested target environment:

- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Gazebo Harmonic / `ros_gz_sim`
- Python 3
- `rclpy`
- `colcon`

Recommended optional packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

Depending on your local MoveIt2 installation, the standard Panda demo resources may also be required for Panda planning profiles.

---

## Build

```bash
mkdir -p ~/ros2_adaptive_assembly_ws/src
cd ~/ros2_adaptive_assembly_ws/src
git clone https://github.com/Lunar-boy/ros2_adaptive_assembly.git

cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
## Docker headless workflow

A headless Docker environment is provided for reproducible ROS2 Jazzy + MoveIt2 builds, plan-only demos, validation scripts, and benchmark workflows.

This container does not provide RViz2 GUI, Gazebo GUI, NVIDIA GPU acceleration, real robot access, camera devices, gripper hardware, or contact-rich insertion simulation.

Build and start the development container:

```bash
docker compose -f docker/compose.yaml up -d --build dev
```
See [docker/README.md](docker/README.md) for the full container workflow.

---

## Recommended demo

The best current end-to-end demonstration is the deterministic known-reachable Panda assembly sequence profile:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_reachable.launch.py
```

This profile demonstrates the current plan-only adaptive assembly path:

- deterministic fake target pose generation;
- task-level pre-grasp and assembly pose generation;
- Panda-specific pose adaptation;
- static scene collision checking;
- two-stage MoveIt2 planning: `pre_grasp -> assembly`;
- successful trajectory export;
- stage-level diagnostics;
- reproducible validation without requiring real hardware or a running controller.

Validate the successful planning path:

```bash
python3 scripts/check_assembly_sequence_success_path.py
```

---

## Core demos

### 1. Minimal non-MoveIt pipeline

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py
```

Publishes:

- `/target_pose`
- `/pre_grasp_pose`
- `/assembly_pose`
- TF: `world -> target_object`

Validate:

```bash
bash scripts/check_pipeline_topics.sh
python3 scripts/check_pipeline_offsets.py
bash scripts/run_pipeline_validation.sh
bash scripts/echo_pipeline_once.sh
```

### Simulated vision perception

Run the deterministic no-camera marker-pose emulator by itself:

```bash
ros2 launch adaptive_assembly_perception \
  simulated_vision_perception.launch.py
```

For the headless Gazebo target synchronization demo:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_simulated_vision_demo.launch.py
```

The camera-frame marker pose emulator publishes `/perceived_target_pose` in
`simulated_camera`, then feeds the existing world-frame `/target_pose` and
`world -> target_object` TF interfaces. It is not pixel-based vision and does
not use a real camera or hardware. See
[`docs/simulated_vision_perception.md`](docs/simulated_vision_perception.md).

---

### 2. Plan-only Panda pre-grasp planning

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

This starts the adaptive pose pipeline, Panda pose adaptation, MoveIt2 plan-only planning, static PlanningScene objects, optional dynamic target collision objects, diagnostics, and visualization markers.

Useful diagnostics:

- `/pre_grasp_plan_success`
- `/pre_grasp_planning_status`
- `/pre_grasp_planning_duration_ms`

Validate:

```bash
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
```

---

### 3. Plan-only Panda assembly sequence

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_demo.launch.py
```

This demo plans two stages:

1. `pre_grasp`
2. `assembly`

The assembly-stage plan starts from the final joint state of the pre-grasp plan. Neither trajectory is executed.

Validate:

```bash
bash scripts/check_assembly_sequence_available.sh
bash scripts/check_assembly_sequence_topics.sh
python3 scripts/check_assembly_sequence_status.py
bash scripts/check_assembly_sequence_stage_diagnostics.sh
python3 scripts/check_assembly_sequence_stage_status.py
```

Stage-level diagnostics:

- `/assembly_sequence_stage_status`
- `/assembly_sequence_stage_success`
- `/assembly_sequence_stage_duration_ms`

Exported trajectories:

- `/pre_grasp_trajectory`
- `/assembly_trajectory`
- `/assembly_sequence_trajectory_status`

---

### 4. Dry-run sequence execution

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_dry_run_execution.launch.py
```

The dry-run executor consumes exported trajectories and validates the two-stage execution abstraction without commanding MoveIt2, Gazebo, controllers, or hardware.

Published topics:

- `/assembly_execution_status`
- `/assembly_execution_success`
- `/assembly_execution_duration_ms`
- `/assembly_execution_stage_status`

Validate:

```bash
bash scripts/check_dry_run_execution_topics.sh
python3 scripts/check_dry_run_execution_status.py
```

---

### 5. Recovery supervisor

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_recovery_supervisor_demo.launch.py
```

The recovery supervisor observes planning, dry-run execution, dynamic scene, and PlanningScene audit statuses. It publishes deterministic recovery decisions but does not automatically retry planning by itself.

Published topics:

- `/assembly_recovery_status`
- `/assembly_recovery_action`
- `/assembly_recovery_success`

Validate:

```bash
bash scripts/check_recovery_supervisor_topics.sh
python3 scripts/check_recovery_supervisor_success_path.py
python3 scripts/check_recovery_supervisor_failure_injection.py
```

---

### 5a. Recovery orchestration and bounded retry (PR37)

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_recovery_orchestration_demo.launch.py
```

The simulator-only orchestrator consumes `/assembly_recovery_action`, runs the
selected logical PlanningScene reset sequence, and calls
`/publish_target_pose_once` to restart the existing pose-to-planning data flow.
It publishes retained results on `/recovery_orchestration_status` and
`/recovery_retry_requested`. Retry means scene reset plus a fresh fake target
pose; it does not execute trajectories or command hardware.

Validate without MoveIt2 or Gazebo:

```bash
bash scripts/check_recovery_orchestrator_available.sh
python3 scripts/check_fake_pose_trigger_service.py
python3 scripts/check_recovery_orchestrator_retry_path.py
python3 scripts/check_recovery_orchestrator_exhausted_path.py
```

See `docs/recovery_orchestration_retry_loop.md` for action mappings,
parameters, and limitations.

---

### 6. Gazebo workcell visualization

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_workcell_demo.launch.py
```

This starts a lightweight Gazebo Harmonic workcell with primitive geometry:

- floor;
- work table;
- raised target support;
- static cylindrical target object;
- four-sided assembly socket fixture.

This workcell launch is visualization-only. It does not spawn the Panda robot,
execute trajectories, attach objects, simulate contact-rich insertion, or run a
`ros2_control` hardware stack. Use the full Gazebo execution demo below for
simulated Panda arm motion.

Validate installed Gazebo assets and launch files:

```bash
bash scripts/check_gazebo_workcell_assets.sh
bash scripts/check_gazebo_workcell_launch_available.sh
```

### 7. PR35 ros2_control success-path execution

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_ros2_control_success_demo.launch.py
```

The simulator-only action fixture drives both trajectory stages to retained
terminal outputs on `/assembly_ros2_control_execution_status`,
`/assembly_ros2_control_execution_success`,
`/assembly_ros2_control_execution_duration_ms`, and stage events on
`/assembly_ros2_control_execution_stage_status`.

This validates the action-level ros2_control execution path. It does not move a
Panda in Gazebo and adds no gripper, attach/detach, contact-rich insertion,
force control, real robot, or real hardware support. See
[`docs/ros2_control_success_path.md`](docs/ros2_control_success_path.md).

### 8. Full Gazebo Panda arm execution

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_gazebo_execution_demo.launch.py
```

This simulator-only demo starts the Gazebo workcell, spawns a lightweight
Panda-like arm, activates `joint_state_broadcaster` and
`panda_arm_controller`, plans the known-reachable two-stage sequence, and sends
the exported `/pre_grasp_trajectory` then `/assembly_trajectory` to
`/panda_arm_controller/follow_joint_trajectory`.

Gazebo starts paused and the Panda base is simulator-only anchored. Launch
unpauses only after controller configuration, then activates both controllers;
topic or action visibility alone is insufficient proof of activation.

Validate with the launch running:

```bash
bash scripts/check_gazebo_panda_spawned.sh
bash scripts/check_ros2_control_controllers_active.sh
bash scripts/check_full_gazebo_execution_topics.sh
python3 scripts/check_gazebo_trajectory_compatibility.py
python3 scripts/check_full_gazebo_execution_status.py
```

This does not add gripper control, object attachment, contact-rich insertion,
force control, camera perception, real robot drivers, or hardware execution. See
[`docs/full_gazebo_panda_execution.md`](docs/full_gazebo_panda_execution.md).

### 9. PR38 Gazebo target object synchronization

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_target_sync_demo.launch.py
```

This headless-by-default simulator demo composes the fake target-pose pipeline,
Gazebo workcell, and a simulator-only synchronizer. `/target_pose` drives the
Gazebo `target_object` model through the Harmonic set-pose service. Status and
pose-error diagnostics are retained on `/gazebo_target_sync_status`,
`/gazebo_target_pose_error_mm`, and `/gazebo_target_pose_error_deg`.

Validate without a Gazebo GUI:

```bash
bash scripts/check_gazebo_target_pose_sync_available.sh
python3 scripts/check_target_pose_to_gazebo_entity_consistency.py
```

See [`docs/gazebo_target_pose_sync.md`](docs/gazebo_target_pose_sync.md).

### 10. PR39 Gazebo grasp attach/detach

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_grasp_attach_demo.launch.py
```

The demo composes full simulator execution, the logical grasp lifecycle, and a
simulator-only kinematic attachment layer. While logically attached, Gazebo's
`target_object` follows `panda_hand`; detach leaves it at its last world pose.
Retained state and diagnostics are published on `/gazebo_object_attached`,
`/gazebo_attach_detach_status`, and `/gazebo_attach_pose_error_mm`.

```bash
bash scripts/check_gazebo_attach_detach_available.sh
python3 scripts/check_gazebo_attach_detach_success_path.py
python3 scripts/check_gazebo_attach_detach_failure_path.py
python3 scripts/check_live_gazebo_attach_detach.py
```

The last command is an optional bounded, headless live Gazebo validation; the
fixture checks remain available without a running simulator. Live attachment
is kinematic set-pose mirroring only.

See [`docs/gazebo_grasp_attach_detach.md`](docs/gazebo_grasp_attach_detach.md).

### 11. PR36 logical gripper and grasp lifecycle

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_logical_grasp_demo.launch.py
```

The demo adds retained `/gripper_command`, `/gripper_command_status`,
`/object_grasp_state`, `/object_grasp_attached`, and
`/logical_grasp_lifecycle_status` topics to the PR35 success fixture. The
logical sequence is open/detached, close/attached after pre-grasp success, then
open/detached after aggregate execution success.

This is state-level grasping only. It does not physically control a gripper,
send gripper action goals, attach Gazebo objects, alter contact physics, or
support real hardware. See
[`docs/gripper_abstraction_logical_grasp.md`](docs/gripper_abstraction_logical_grasp.md).

---

## PlanningScene tools

### Static PlanningScene objects

The Panda planning demo applies simple static collision objects for the workcell.

Services and topics:

- `/clear_static_planning_scene`
- `/reapply_static_planning_scene`
- `/static_planning_scene_status`

Validate:

```bash
bash scripts/check_static_planning_scene_services.sh
bash scripts/clear_static_planning_scene_once.sh
bash scripts/reapply_static_planning_scene_once.sh
python3 scripts/check_static_planning_scene_status.py
```

### Dynamic target object

The dynamic target scene node subscribes to `/panda_pre_grasp_pose`, applies or updates the `target_object_dynamic` collision object, and provides a clear service for repeated demos or benchmark comparisons.

Topics and services:

- `/dynamic_target_scene_ready`
- `/dynamic_target_scene_status`
- `/clear_dynamic_target_scene`

Validate:

```bash
bash scripts/check_dynamic_target_scene_available.sh
bash scripts/check_dynamic_target_scene_ready.sh
python3 scripts/check_dynamic_target_scene_status.py
bash scripts/check_dynamic_target_clear_service.sh
bash scripts/clear_dynamic_target_scene_once.sh
```

### PlanningScene audit

The audit node checks whether expected collision objects are visible in MoveIt2:

- `work_table`
- `target_support`
- `target_object_dynamic`

Published topics:

- `/planning_scene_audit_ready`
- `/planning_scene_audit_status`

Validate:

```bash
bash scripts/check_planning_scene_audit_available.sh
python3 scripts/check_planning_scene_audit_status.py
bash scripts/check_planning_scene_audit_ready.sh
```

### Unified PlanningScene reset

```bash
bash scripts/reset_planning_scene_once.sh
```

This clears the dynamic target object, clears static objects, and reapplies the static table/support objects.

Validate:

```bash
bash scripts/check_planning_scene_reset_workflow.sh
```

---

## Benchmarks

### Seeded planning benchmark

Launch the seeded benchmark profile:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py
```

Record benchmark diagnostics in another terminal:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/seeded_planning_diagnostics.csv \
  bash scripts/run_seeded_planning_benchmark.sh
```

### Benchmark profile suite

The benchmark suite provides deterministic baseline, narrow, wide, and fixed-yaw target-pose profiles for comparing planning diagnostics CSV files.

Validate the installed profile suite:

```bash
bash scripts/check_benchmark_profile_suite.sh
bash scripts/check_benchmark_profile_params.sh
```

Compare recorded CSV files:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input baseline=benchmark_results/baseline.csv \
  --input narrow=benchmark_results/narrow.csv \
  --input wide=benchmark_results/wide.csv \
  --input fixed_yaw=benchmark_results/fixed_yaw.csv
```

Export a Markdown report:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input baseline=benchmark_results/baseline.csv \
  --input narrow=benchmark_results/narrow.csv \
  --input wide=benchmark_results/wide.csv \
  --output-markdown benchmark_results/benchmark_report.md
```

### Contact-lite insertion benchmark

Run the deterministic known-reachable plan-only sequence with geometric
insertion evaluation:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_contact_lite_insertion_benchmark.launch.py
```

The evaluator publishes:

- `/assembly_insertion_status`
- `/assembly_insertion_success`
- `/assembly_insertion_error_mm`
- `/assembly_insertion_error_deg`

By default, the achieved pose is the planned `/panda_assembly_pose`; status
therefore reports `achieved_pose_source=planned_pose`. This benchmark checks
only final-pose geometry. It adds no force control, tactile sensing,
contact-rich peg-in-hole behavior, real hardware execution, or Gazebo contact
physics requirement.

Record CSV data and export a Markdown summary:

```bash
MAX_TRIALS=20 TIMEOUT_SEC=120 \
  bash scripts/run_contact_lite_insertion_benchmark.sh
```

Validate with synthetic poses and report export:

```bash
bash scripts/check_contact_lite_insertion_topics.sh
python3 scripts/check_contact_lite_insertion_success_path.py
python3 scripts/check_contact_lite_insertion_failure_path.py
bash scripts/check_contact_lite_insertion_report_export.sh
```

See
[`docs/contact_lite_insertion_benchmark.md`](docs/contact_lite_insertion_benchmark.md)
for parameters, status fields, and limitations.

### Suggested result table

After recording benchmark data, add a small result table here:

| Profile | Events | Success | Failed | Skipped | Mean planning time |
|---|---:|---:|---:|---:|---:|
| baseline | TBD | TBD | TBD | TBD | TBD |
| narrow target range | TBD | TBD | TBD | TBD | TBD |
| wide target range | TBD | TBD | TBD | TBD | TBD |
| fixed yaw | TBD | TBD | TBD | TBD | TBD |
| guarded planner | TBD | TBD | TBD | TBD | TBD |

Replace `TBD` values with locally recorded benchmark results.

---

## Scope and limitations

This repository currently focuses on reproducible software architecture and plan-only evaluation.

Implemented:

- ROS2 topic and TF pipeline
- task-level pose generation
- Panda-specific pose adaptation
- MoveIt2 plan-only planning
- two-stage assembly sequence planning
- static and dynamic PlanningScene objects
- PlanningScene audit/reset workflows
- planning diagnostics and benchmarks
- contact-lite geometric insertion benchmark
- trajectory export
- dry-run execution abstraction
- recovery supervisor
- lightweight Gazebo workcell visualization
- simulator-only Gazebo Panda arm execution through ros2_control
- simulator-only Gazebo target object synchronization from `/target_pose`
- gripper control and object attachment (simulator-only kinematic layer)

Not yet implemented:

- camera-based perception;
- contact-rich insertion physics;
- force-controlled or tactile insertion behavior;
- force/torque feedback control;
- real robot hardware execution.

These are planned extensions after the planning, scene, diagnostics, and benchmark layers are stable.

---

## Relevance to adaptive industrial robotics

This project is motivated by industrial assembly scenarios where fixed, hard-coded trajectories are brittle under target-pose variation, manufacturing tolerance, and workspace uncertainty.

The current implementation focuses on the software architecture required for adaptive manipulation:

- perception-driven target poses;
- TF-based frame handling;
- robot-specific pose adaptation;
- collision-aware MoveIt2 planning;
- PlanningScene updates;
- diagnostic topics;
- reproducible benchmark profiles;
- recovery-state supervision.

This makes the repository suitable as a portfolio project for robotics software engineering, adaptive manipulation, ROS2/MoveIt2 development, and applied research internships in industrial robotics.

---

## License

Add a license before external reuse. Recommended options:

- MIT License for permissive open-source reuse;
- Apache-2.0 if patent clauses and robotics/industry compatibility matter.
