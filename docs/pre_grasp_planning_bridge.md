# Plan-only pre-grasp planning bridge

PR6 added a minimal MoveIt2 bridge that asks MoveIt2 to plan a Panda arm
trajectory. PR7 adds a Panda-specific adapter in front of that bridge so the
normal planning launch uses `/panda_pre_grasp_pose` instead of the task-level
`/pre_grasp_pose`.

```text
fake_object_pose_node
     │
     ▼
   /target_pose
     │
     ▼
assembly_task_node
     │
     ▼
/pre_grasp_pose
     │
     ▼
panda_pre_grasp_pose_adapter_node
     │
     ▼
/panda_pre_grasp_pose
     │
     ▼
pre_grasp_planning_node
     │
     ├── MoveIt2 plan request to panda_arm
     ├── /pre_grasp_plan_success
     ├── /pre_grasp_planning_status
     └── /pre_grasp_planning_duration_ms
```

When launched through `pre_grasp_planning.launch.py`, the planning bridge now
subscribes to `/panda_pre_grasp_pose`. The C++ node still supports a configurable
`input_topic` parameter and defaults to `/pre_grasp_pose` if launched manually
without parameters.

When launched through `panda_pre_grasp_pose_adapter.launch.py`,
`/panda_pre_grasp_pose` defaults to frame `panda_link0`. This reduces frame
mismatch risk with the standard Panda MoveIt2 demo.

When launched through the Panda planning demo, the planning bridge now also runs
alongside static PlanningScene collision objects from
`static_planning_scene_node` and the dynamic target collision object from
`dynamic_target_scene_node`.

PR10 adds planning diagnostics. `/pre_grasp_plan_success` is still available for
compatibility, but it is no longer the only status signal. The bridge also
publishes:

- `/pre_grasp_planning_status`: a human-readable key-value event string
- `/pre_grasp_planning_duration_ms`: wall-clock planning duration in milliseconds

The status topic distinguishes `failure` from `skipped_small_motion`, which
means a small target update did not trigger a new MoveIt2 planning request.

The bridge is intentionally plan-only. Gazebo, ros2_control integration for this
project, real robot hardware, and PlanningScene collision objects are not added
by the bridge, and trajectory execution is still disabled.

## Install dependencies

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

## Validate

```bash
bash scripts/check_planning_bridge_available.sh
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
ros2 topic echo /pre_grasp_plan_success
ros2 topic echo /pre_grasp_planning_status
ros2 topic echo /pre_grasp_planning_duration_ms
```

Expected behavior:

- `/target_pose`, `/pre_grasp_pose`, and `/assembly_pose` are still published
- `/panda_pre_grasp_pose` is published by the Panda pose adapter
- The Panda MoveIt2 demo starts
- `pre_grasp_planning_node` attempts planning when `/panda_pre_grasp_pose`
  changes enough
- `/pre_grasp_plan_success` publishes `true` or `false`
- `/pre_grasp_planning_status` publishes `success`, `failure`, or
  `skipped_small_motion`
- `/pre_grasp_planning_duration_ms` publishes the latest planning attempt
  duration
- No execution occurs

## Next PR

A future PR can compare planning benchmarks with and without dynamic target
scene updates or add object removal/reset utilities.
