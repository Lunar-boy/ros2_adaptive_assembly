# AGENTS.md

This file defines the operating rules for coding agents working in this repository.

## Mission

The repository has one primary product path:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py
```

The goal is a simulator-only, physics-based Panda pick-and-place task in which the arm:

1. observes the dynamic Gazebo `target_object`;
2. plans and executes `pre_grasp -> grasp -> lift -> pre_place -> place -> retreat`;
3. closes the simulated gripper and verifies the grasp;
4. transports the object to the socket fixture;
5. opens the gripper;
6. retreats; and
7. leaves the object stably inside the socket.

The project is complete only when the final condition in step 7 is verified from Gazebo state. Planning success, controller success, gripper success, grasp verification, lift verification, or `/physical_pick_place_execution_success=true` alone is not whole-task success.

## Current truthful status

The repository already contains:

- the physical Gazebo workcell and dynamic target object;
- Gazebo target-pose observation and adaptation to `/target_pose`;
- six-stage MoveIt planning for `assembly_tcp`;
- simulator-only `ros2_control` arm execution;
- simulated gripper command execution;
- contact-aware close handling;
- grasp and lift/slip verification;
- preflight, status, logging, and bounded runtime checks.

The repository does not yet prove that the released object remains inside the socket. Treat final socket-placement verification as the highest-priority missing capability.

## Hard scope boundaries

Unless a task explicitly changes the scope:

- Keep the project simulator-only.
- Do not add real robot drivers or claim real-hardware support.
- Do not add camera hardware, image perception, marker detection, or visual servoing.
- Do not add Isaac Sim, Docker, devcontainers, or container-only workflows.
- Do not use a kinematic attach/detach shortcut to claim physical grasp or placement success.
- Do not enable `fake_object_pose_node` in the full physical launch.
- Do not replace Gazebo-observed object state with commanded or planned poses.
- Do not broaden the root README with legacy demos or unrelated research directions.
- Do not delete legacy files until a dependency search proves they are unused by the full physical path and relevant tests still pass.

“Physical” in this repository means physics-based Gazebo simulation, not a real robot.

## Primary launch dependency chain

The supported entry point is:

```text
adaptive_assembly_full_physical_pick_place_demo.launch.py
  -> adaptive_assembly_panda_gazebo.launch.py
  -> gazebo_target_pose_adapter.launch.py
  -> adaptive_assembly_physical_pick_place_execution.launch.py
       -> adaptive_assembly_panda_sequence_planning_reachable.launch.py
       -> gripper_action_bridge_node
       -> gazebo_grasp_contact_status_node
       -> grasp_verifier_node
       -> physical_grasp_preflight_node
       -> physical_pick_place_executor_node
```

Agents must inspect this chain before modifying launch arguments, topic names, frames, time settings, controllers, or task parameters.

## Non-negotiable runtime contracts

Preserve these contracts unless the task explicitly requires a coordinated migration:

### Simulation and control

- Gazebo is the only controller-manager provider for the full physical launch.
- `use_standard_panda_demo` remains `false` in the full physical path.
- `use_sim_time` remains `true` throughout the full physical path.
- The arm action remains `/panda_arm_controller/follow_joint_trajectory` unless all producers, consumers, tests, and docs are updated together.
- The target object remains dynamic in `adaptive_assembly_physical_workcell.sdf`.

### Perception and frames

- Gazebo `/model/target_object/pose` is the physical target source.
- The bridge and observer publish the authoritative object pose used by the task.
- `/target_pose` must not be synthesized before a valid Gazebo observation exists.
- `target_pose_output_frame_id` is only a frame-label override; do not describe it as a TF transform.
- Physical stage targets use `assembly_tcp`.
- The six `/panda_*_pose` topics represent desired `assembly_tcp` poses in `panda_link0`.

### Planning and execution

- The default stage order is:

  ```text
  pre_grasp, grasp, lift, pre_place, place, retreat
  ```

- Gripper close occurs after `grasp`.
- Grasp verification occurs after close.
- Lift/slip verification occurs after `lift`.
- Gripper open occurs after `place`.
- Retreat occurs after release.
- A controller result is not a geometric task-success result.

### Contact and grasp semantics

- `contact_limited_success` is valid only under the existing fresh, settled, expected-object, bilateral-contact contract.
- It permits the sequence to continue; it does not prove whole-task success.
- Stale, unilateral, unrelated-object, rejected, canceled, timed-out, or otherwise invalid close results remain failures.

### Geometry parity

- `adaptive_assembly_physical_workcell.sdf` is the source of truth for the physical table, target support, target object, and socket geometry.
- `physical_workcell_planning_scene.yaml` must remain geometrically consistent with the SDF where MoveIt collision geometry overlaps the world.
- Changes to target size, socket size, support pose, socket pose, TCP, or place pose require coordinated updates to configuration, checks, and documentation.

## Final insertion success contract

A future final verifier must not infer success from the planned place pose or executor completion. It must use fresh Gazebo-observed state after object release and arm retreat.

At minimum, final task success must require:

- the six-stage executor completed successfully;
- the gripper open command succeeded after `place`;
- the `retreat` stage completed;
- a fresh, finite Gazebo object pose is available;
- the object lies within a configurable socket acceptance region;
- the object orientation is within a configurable acceptance tolerance when orientation matters;
- the object remains within tolerance for a configurable settle interval;
- the result is evaluated after release, not while the fingers are still carrying the object; and
- terminal success and failure include explicit machine-readable reasons.

Prefer retained outputs such as:

- `/physical_pick_place_task_status` (`std_msgs/msg/String`);
- `/physical_pick_place_task_success` (`std_msgs/msg/Bool`); and
- an optional final pose-error or settle-duration diagnostic.

The exact interface may be changed in a focused PR, but there must be one unambiguous whole-task success signal. Do not repurpose `/physical_pick_place_execution_success` without a migration plan because it currently describes executor completion.

## Relevant repository areas

Focus development on these paths:

```text
src/adaptive_assembly_bringup/
  launch/adaptive_assembly_full_physical_pick_place_demo.launch.py
  launch/adaptive_assembly_physical_pick_place_execution.launch.py
  config/adaptive_assembly_physical_pick_place_params.yaml
  config/physical_workcell_planning_scene.yaml

src/adaptive_assembly_sim/
  launch/adaptive_assembly_panda_gazebo.launch.py
  launch/gazebo_target_pose_adapter.launch.py
  worlds/adaptive_assembly_physical_workcell.sdf

src/adaptive_assembly_task/
src/adaptive_assembly_planning/
src/adaptive_assembly_manipulation/
src/adaptive_assembly_execution/

scripts/check_physical_*.py
scripts/check_full_physical_pick_place_*.py
scripts/run_full_physical_pick_place_with_logs.sh

docs/physical_pick_place_execution.md
docs/gazebo_contact_grasp_verification.md
docs/run_logging.md
```

Legacy demos may remain temporarily as dependencies or regression fixtures, but new work should not expand them.

## Workspace conventions

Use two explicit roots:

```text
WS_ROOT=<colcon workspace>
REPO_ROOT=$WS_ROOT/src/ros2_adaptive_assembly
```

Run `colcon` commands from `WS_ROOT`. Run repository scripts from `REPO_ROOT` after sourcing the workspace.

Do not modify generated directories:

- `build/`
- `install/`
- `log/`
- `runs/`
- Python cache and test cache directories

## Build

```bash
cd "$WS_ROOT"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Use package-selective builds while iterating, but run the affected dependency set before declaring a change complete.

## Required validation discipline

Every behavioral PR must include the narrowest relevant static tests and at least one runtime success or expected-failure path.

Baseline static and package checks:

```bash
cd "$WS_ROOT"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon test --packages-select \
  adaptive_assembly_bringup \
  adaptive_assembly_execution \
  adaptive_assembly_manipulation \
  adaptive_assembly_planning \
  adaptive_assembly_sim \
  adaptive_assembly_task \
  --event-handlers console_direct+
colcon test-result --verbose
```

Repository-level physical-path checks:

```bash
cd "$REPO_ROOT"
python3 scripts/check_physical_pick_place_launch_static.py
python3 scripts/check_physical_grasp_preflight_static.py
python3 scripts/check_physical_planning_scene_parity.py
python3 scripts/check_full_physical_pick_place_arm_motion.py
python3 scripts/check_full_physical_pick_place_tcp_contract.py
```

The bounded arm-motion and TCP checks do not prove final placement or insertion. Never report them as end-to-end completion.

For full manual runs with preserved logs:

```bash
cd "$REPO_ROOT"
bash scripts/run_full_physical_pick_place_with_logs.sh
```

If a check cannot run because ROS, Gazebo, MoveIt, or display/runtime dependencies are unavailable, state the exact blocker and provide the exact command that remains to be run.

## Coding rules

- Keep changes small and centered on the supported physical path.
- Preserve deterministic status schemas and explicit failure reasons.
- Prefer typed messages for new structured interfaces; if strings are retained, document their stable key/value schema.
- Keep timeouts and tolerances configurable through ROS parameters or launch arguments.
- Do not hide sleeps in callbacks; use timers, actions, or explicit state-machine transitions.
- Keep headless Gazebo validation possible.
- Do not require a GPU.
- Add tests for success, timeout, stale data, wrong source, and geometric boundary conditions when introducing a verifier.
- Update launch wiring, package dependencies, tests, root README, and focused docs together when behavior changes.

## Documentation rules

The root `README.md` must describe only the supported full physical pick-and-place path, its truthful current status, build/run instructions, validation, and known limitations.

Do not claim:

- successful insertion before a Gazebo-state final verifier exists;
- real perception;
- force control or tactile control;
- real-hardware execution;
- physics-accurate behavior when a component is only logical or kinematic.

## Definition of done for agent work

Before reporting completion, summarize:

1. files added, modified, and deleted;
2. runtime behavior changed;
3. interfaces or parameters changed;
4. build result;
5. tests and runtime checks run;
6. evidence for success;
7. known limitations; and
8. whether the change advances the final criterion: the released object remains stably inside the socket after retreat.

A change is not complete merely because code compiles or the executor publishes success.
