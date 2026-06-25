# AGENTS.md

This repository is a ROS2 Jazzy project for simulated adaptive robotic assembly.

## Environment

- Repository root: `~/ros2_adaptive_assembly_ws`
- GitHub repository: `Lunar-boy/ros2_adaptive_assembly`
- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2 Panda demo resources when planning features are involved
- Build with `colcon build --symlink-install`
- C++ packages use `ament_cmake`
- Python packages use `ament_python`

## Repository rules

- Keep each PR small, focused, and reviewable.
- Do not modify `build/`, `install/`, or `log/`.
- Do not add large binary files.
- Do not change unrelated package behavior.
- Do not change the license field unless explicitly requested.
- Every package should have a README.
- Every new ROS2 node should include:
  - clear parameters;
  - launch integration;
  - validation script or validation instructions;
  - documentation update when behavior changes.

## Hard project boundaries

Unless the task explicitly says otherwise:

- Do not add Isaac Sim.
- Do not add Gazebo.
- Do not add `ros2_control`.
- Do not require a real robot.
- Do not execute robot trajectories.
- Do not add real robot drivers.
- Do not add learning, VLA, grasp-policy, or hardware-control code to infrastructure PRs.
- Keep MoveIt2 integration plan-only unless execution is explicitly requested.

## Current architecture

The project currently contains:

- `adaptive_assembly_perception`
  - publishes `/target_pose`
  - broadcasts TF `world -> target_object`
  - supports deterministic seeded benchmark profiles

- `adaptive_assembly_task`
  - subscribes to `/target_pose`
  - publishes `/pre_grasp_pose`
  - publishes `/assembly_pose`

- `adaptive_assembly_planning`
  - adapts `/pre_grasp_pose` to Panda-compatible `/panda_pre_grasp_pose`
  - provides static PlanningScene collision objects
  - provides a plan-only MoveIt2 pre-grasp planning bridge
  - publishes planning diagnostics:
    - `/pre_grasp_plan_success`
    - `/pre_grasp_planning_status`
    - `/pre_grasp_planning_duration_ms`

- `adaptive_assembly_bringup`
  - launches the non-MoveIt pipeline
  - launches the optional Panda MoveIt2 demo
  - launches the plan-only Panda planning demo
  - launches deterministic planning benchmark profiles

## Core data flow

```text
/target_pose
  -> /pre_grasp_pose
  -> /panda_pre_grasp_pose
  -> plan-only MoveIt2 planning
```

Planning diagnostics:

```text
/pre_grasp_plan_success
/pre_grasp_planning_status
/pre_grasp_planning_duration_ms
```

## MoveIt2 conventions

- Use MoveIt2 for planning and PlanningScene updates only.
- Do not call trajectory execution APIs unless explicitly requested.
- Avoid changing existing node behavior unless the prompt explicitly requires it.
- PlanningScene updates should be minimal, deterministic, and validated by topic-level checks.
- Static collision objects and dynamic collision objects should be implemented as separate, focused nodes when practical.
- Do not remove or reset existing PlanningScene objects unless the PR explicitly requires it.

## File placement conventions

- C++ ROS2 nodes: `src/<package>/src/*.cpp`
- Python ROS2 nodes: package module directory
- Launch files: `src/<package>/launch/*.launch.py`
- Package READMEs: `src/<package>/README.md`
- Project docs: `docs/*.md`
- Validation scripts: `scripts/check_*.sh` or `scripts/check_*.py`
- Benchmark helpers: `scripts/run_*benchmark*.sh` or `scripts/compare_*.py`

## C++ conventions

- Keep C++17 enabled.
- Prefer simple `rclcpp::Node` implementations.
- Declare parameters with clear defaults.
- Log startup configuration clearly.
- Log important runtime decisions such as update, skip, failure, and validation-relevant events.
- Prefer explicit topic names and parameter names over hidden behavior.
- Do not add unnecessary dependencies.

## Python conventions

- Use Python 3.
- Use `rclpy` for ROS2 validation nodes or subscribers.
- Use clear non-zero exits for validation failures.
- Keep scripts deterministic and easy to run from the repository root.
- Validation scripts should print clear PASS/FAIL messages.

## Launch conventions

- Launch files should be minimal and composable.
- Prefer package-local launch files for package-local nodes.
- Bringup launch files may include package launch files.
- Use screen output for development and validation launch files.
- Keep launch order explicit when startup order matters for readability.

## Documentation policy

When behavior changes:

- Update root `README.md`.
- Update or add a focused file under `docs/`.
- Keep roadmap entries current.
- Preserve plan-only/no-execution messaging for MoveIt2 features.
- Mention validation commands for new features.
- Explain whether existing node behavior changed.

## Validation policy

Always run or explain why you could not run:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
git diff --check
```

For non-MoveIt pipeline changes, also validate:

```bash
bash scripts/run_pipeline_validation.sh
```

For Panda/MoveIt planning changes, validate relevant checks such as:

```bash
bash scripts/check_panda_adapted_pose.sh
bash scripts/check_panda_adapted_pose_frame.sh
python3 scripts/check_panda_pose_adapter_orientation.py --expected-frame panda_link0
bash scripts/check_static_planning_scene_ready.sh
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
```

For benchmark changes, validate relevant benchmark scripts and at least one smoke test when practical.

## Final response format

After implementing a PR, summarize:

1. files added;
2. files modified;
3. whether existing behavior changed;
4. build result;
5. validation results;
6. known limitations;
7. whether the PR is safe to commit.

## PR prompt style

PR prompts should stay focused on the current PR. Do not repeat all repository rules from this file.

A good PR prompt should include:

- PR goal;
- files to add or modify;
- required ROS2 topics, parameters, message types, and launch files;
- behavior that must not change;
- validation commands specific to the PR;
- final summary requirements only if they differ from this file.

Avoid duplicating:

- global environment details;
- general project boundaries;
- full package history;
- unrelated previous PR descriptions;
- generic validation commands already listed here.
