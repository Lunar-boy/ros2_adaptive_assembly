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

## Panda-adapted pre-grasp planning

`/pre_grasp_pose` is task-level. `/panda_pre_grasp_pose` is robot-specific and
uses a configurable Panda-oriented end-effector orientation. The Panda planning
demo uses `/panda_pre_grasp_pose` as the planning bridge input.
By default, `/panda_pre_grasp_pose` uses frame `panda_link0` for compatibility
with the standard Panda MoveIt2 demo.

See [docs/panda_pre_grasp_pose_adapter.md](docs/panda_pre_grasp_pose_adapter.md).

## Static PlanningScene objects

The Panda planning demo also starts a static PlanningScene node that applies
simple collision objects for the table/workcell:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

This adds static collision objects only and still does not execute trajectories.
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

## Planning benchmark recording

Planning diagnostics can be recorded to CSV and summarized into
success/failure/skipped counts plus duration statistics:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/planning_diagnostics.csv bash scripts/run_planning_benchmark.sh
```

See [docs/planning_benchmark.md](docs/planning_benchmark.md).

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
- Future PR: dynamic PlanningScene updates and planning refinements
