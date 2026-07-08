# AGENTS.md

This repository is a ROS2 Jazzy project for simulated adaptive robotic assembly.

The project is intended to be a portfolio-quality ROS2 / MoveIt2 / Gazebo simulation project. It should remain simulator-only unless a task explicitly requests real hardware support.

## Environment

- Repository and workspace root: `~/ros2_adaptive_assembly_ws`
- GitHub repository: `Lunar-boy/ros2_adaptive_assembly`
- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Gazebo / ros_gz for simulator-only demos
- `ros2_control` only for simulated controllers
- Build with `colcon build --symlink-install`
- C++ packages use `ament_cmake`
- Python packages use `ament_python`


Use `~/ros2_adaptive_assembly_ws` as the only project root in commands, scripts, documentation, and examples.

## Repository rules

- Keep each PR small, focused, and reviewable.
- Do not modify generated build artifacts:
  - `build/`
  - `install/`
  - `log/`
- Do not add large binary files.
- Do not change unrelated package behavior.
- Do not change license fields unless explicitly requested.
- Every package should have a README or a clear documentation entry.
- Every new ROS2 node should include:
  - clear parameters;
  - documented topics, services, or actions;
  - launch integration when applicable;
  - validation script or validation instructions;
  - documentation update when behavior changes.
- Do not add Docker files, Docker Compose files, devcontainer files, or container-only scripts unless explicitly requested.

## Current project scope

The repository supports:

- ROS2 topic and TF based adaptive assembly pipelines;
- deterministic fake target-pose generation;
- task-level pre-grasp and assembly-pose generation;
- Panda-specific pose adaptation;
- MoveIt2 planning and PlanningScene updates;
- simulator-only Gazebo execution demos;
- simulator-only `ros2_control` integration;
- kinematic or logical simulated grasp attach/detach behavior;
- validation scripts and benchmark-style smoke tests;
- recovery and episode-level supervision when implemented as simulator-only logic.

The repository does not currently claim to implement:

- real robot execution;
- real robot drivers;
- real camera perception;
- marker detection from live images;
- visual servoing with a real camera;
- VLA policies;
- Isaac Sim based workflows.

## Hard project boundaries

Unless the task explicitly says otherwise:

- Do not add Docker or container workflows.
- Do not add Isaac Sim.
- Do not require a real robot.
- Do not add real robot drivers.
- Do not add camera, USB, or hardware passthrough assumptions.
- Do not add learning, VLA, grasp-policy, or hardware-control code to infrastructure PRs.
- Keep all execution features simulator-only.
- Keep Gazebo features headless-capable where practical.
- Keep `ros2_control` usage limited to simulation.
- Do not describe fake perception as real perception.

Gazebo, `ros2_control`, and trajectory execution are allowed only when they are clearly simulator-only and documented as such.

## Current architecture

The project may contain packages such as:

- `adaptive_assembly_perception`
  - publishes simulated or fake target poses;
  - may broadcast TF such as `world -> target_object`;
  - supports deterministic seeded benchmark profiles.

- `adaptive_assembly_task`
  - subscribes to target-pose inputs;
  - computes task-level pre-grasp, grasp, assembly, place, or retreat poses;
  - publishes task poses for downstream planning and execution.

- `adaptive_assembly_planning`
  - adapts task poses to Panda-compatible poses;
  - manages MoveIt2 planning requests;
  - updates static or dynamic PlanningScene objects;
  - publishes planning diagnostics.

- `adaptive_assembly_execution`
  - bridges planned trajectories to simulated controllers;
  - must remain simulator-only unless explicitly requested otherwise.

- `adaptive_assembly_manipulation`
  - provides simulated grasp, attach, detach, or object-pose update logic;
  - must clearly distinguish logical or kinematic attachment from physical grasping.

- `adaptive_assembly_sim`
  - contains Gazebo simulation assets, simulated controller configuration, robot descriptions, or simulator launch support.

- `adaptive_assembly_recovery`
  - implements recovery policies, retry logic, timeout handling, or failure classification;
  - should keep recovery semantics explicit and testable.

- `adaptive_assembly_episode`
  - coordinates multi-stage assembly episodes;
  - should expose stable episode status and validation-friendly outputs.

- `adaptive_assembly_benchmark`
  - runs deterministic benchmark profiles;
  - records reproducible metrics;
  - compares success, duration, failure reasons, and final pose quality.

- `adaptive_assembly_bringup`
  - composes package-level launch files into runnable demos;
  - should keep launch arguments documented and validation-friendly.

## Core data flow

The expected high-level data flow is:

```text
target pose
  -> task-level assembly poses
  -> Panda / robot-specific pose adaptation
  -> MoveIt2 planning
  -> simulator-only execution
  -> simulated grasp/object update
  -> episode status and validation
```

Use ROS2 topics, services, actions, and TF consistently. Avoid hidden state that cannot be validated from logs, topic output, or deterministic scripts.

## MoveIt2 conventions

- Use MoveIt2 for planning, IK, collision checking, trajectory generation, and PlanningScene updates.
- Trajectory execution is allowed only through simulator-only launch files and simulated controllers.
- Keep plan-only launch paths available when practical.
- Avoid changing existing node behavior unless the task explicitly requires it.
- PlanningScene updates should be minimal, deterministic, and validated by topic-level or launch-level checks.
- Static collision objects and dynamic collision objects should remain separable when practical.
- When planning depends on multiple pose topics, guard against stale mixed inputs by using timestamps, sequence IDs, or clear validation logic when practical.

## Gazebo and simulator conventions

- Gazebo support must remain simulator-only.
- Prefer headless-compatible launch paths for automated validation.
- Do not require Gazebo GUI for smoke tests.
- Do not require GPU acceleration.
- If a task introduces simulated contact or force signals, document exactly whether they are physical simulation outputs, approximations, or scripted abstractions.

## `ros2_control` conventions

- Use `ros2_control` only for simulated controllers unless explicitly requested.
- Keep controller names, joint names, and action names configurable where practical.
- Document controller startup assumptions in launch files or package README files.
- Validation scripts should fail clearly when controllers are unavailable, inactive, or not connected.

## Topic and interface conventions

- Prefer configurable topic names through ROS2 parameters or launch arguments.
- Avoid unnecessary absolute topic names when namespace support is useful.
- If absolute topic names are intentionally used for simple demos, document that choice.
- Prefer typed ROS2 messages for structured status when practical.
- If string status messages are used, define and document a stable schema.
- Include enough status fields for validation scripts to distinguish:
  - planning failure;
  - execution failure;
  - timeout;
  - object attachment failure;
  - final pose validation failure;
  - successful completion.

## File placement conventions

- C++ ROS2 nodes: `src/<package>/src/*.cpp`
- Python ROS2 nodes: `src/<package>/<python_module>/*.py`
- Launch files: `src/<package>/launch/*.launch.py`
- Config files: `src/<package>/config/*.yaml`
- URDF/Xacro/SDF assets: package-local `urdf/`, `xacro/`, `models/`, or `worlds/`
- Package READMEs: `src/<package>/README.md`
- Project docs: `docs/*.md`
- Validation scripts: `scripts/check_*.sh` or `scripts/check_*.py`
- Benchmark helpers: `scripts/run_*benchmark*.sh` or `scripts/compare_*.py`

## C++ conventions

- Keep C++17 enabled.
- Prefer simple `rclcpp::Node` implementations.
- Declare parameters with clear defaults.
- Log startup configuration clearly.
- Log important runtime decisions such as update, skip, failure, retry, and validation-relevant events.
- Prefer explicit topic names, parameter names, and frame names over hidden behavior.
- Do not add unnecessary dependencies.
- Keep large behavior changes out of unrelated refactor PRs.

## Python conventions

- Use Python 3.
- Use `rclpy` for ROS2 validation nodes, subscribers, or lightweight orchestration nodes.
- Use clear non-zero exits for validation failures.
- Keep scripts deterministic and easy to run from `~/ros2_adaptive_assembly_ws`.
- Validation scripts should print clear PASS/FAIL messages.
- Avoid hard-coded Docker or `/workspaces/...` paths.
- Prefer `Path(__file__).resolve()` or repository-root-relative assumptions documented in the script.

## Launch conventions

- Launch files should be minimal and composable.
- Prefer package-local launch files for package-local nodes.
- Bringup launch files may include package launch files.
- Use screen output for development and validation launch files.
- Keep launch order explicit when startup order matters for readability.
- Expose important behavior as launch arguments.
- Clearly distinguish:
  - plan-only demos;
  - simulator execution demos;
  - Gazebo demos;
  - benchmark demos;
  - recovery or full-episode demos.

## Documentation policy

When behavior changes:

- Update root `README.md`.
- Update or add a focused file under `docs/` when the behavior is non-trivial.
- Keep roadmap entries current.
- Keep scope claims accurate:
  - simulator-only means simulator-only;
  - fake perception means fake perception;
  - kinematic attachment means kinematic attachment;
  - contact-lite validation means contact-lite validation;
  - force control should not be claimed unless implemented.
- Mention validation commands for new features.
- Explain whether existing node behavior changed.
- Remove stale Docker references when editing setup or workflow docs.
- Do not add new Docker instructions unless explicitly requested.

## Validation policy

Always run or explain why you could not run:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
git diff --check
```

For non-MoveIt pipeline changes, also validate relevant checks such as:

```bash
bash scripts/run_pipeline_validation.sh
```

For Panda / MoveIt2 planning changes, validate relevant checks such as:

```bash
bash scripts/check_panda_adapted_pose.sh
bash scripts/check_panda_adapted_pose_frame.sh
python3 scripts/check_panda_pose_adapter_orientation.py --expected-frame panda_link0
bash scripts/check_static_planning_scene_ready.sh
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
```

For Gazebo or simulator execution changes, validate with a headless simulator smoke test when practical. Use repository-root-relative commands only.

For recovery, episode, or benchmark changes, validate at least one deterministic success path and one expected failure or timeout path when practical.

For documentation-only changes, run at minimum:


## Final response format

After implementing a PR, summarize:

1. files added;
2. files modified;
3. files deleted;
4. whether existing behavior changed;
5. build result;
6. validation results;
7. known limitations;
8. whether the PR is safe to commit.

If validation could not be run, explain the exact reason and provide the commands the user should run locally.

## PR prompt style

PR prompts should stay focused on the current PR. Do not repeat all repository rules from this file.

A good PR prompt should include:

- PR goal;
- files to add, modify, or delete;
- required ROS2 topics, parameters, message types, launch files, and validation scripts;
- behavior that must not change;
- validation commands specific to the PR;
- final summary requirements only if they differ from this file.

Avoid duplicating:

- global environment details;
- general project boundaries;
- full package history;
- unrelated previous PR descriptions;
- generic validation commands already listed here.
