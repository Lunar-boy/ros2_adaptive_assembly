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
     └── /pre_grasp_plan_success
```

When launched through `pre_grasp_planning.launch.py`, the planning bridge now
subscribes to `/panda_pre_grasp_pose`. The C++ node still supports a configurable
`input_topic` parameter and defaults to `/pre_grasp_pose` if launched manually
without parameters.

The bridge is intentionally plan-only. Gazebo, ros2_control integration for this
project, real robot hardware, and PlanningScene collision objects are not added
yet.

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
ros2 topic echo /pre_grasp_plan_success
```

Expected behavior:

- `/target_pose`, `/pre_grasp_pose`, and `/assembly_pose` are still published
- `/panda_pre_grasp_pose` is published by the Panda pose adapter
- The Panda MoveIt2 demo starts
- `pre_grasp_planning_node` attempts planning when `/panda_pre_grasp_pose`
  changes enough
- `/pre_grasp_plan_success` publishes `true` or `false`
- No execution occurs

## Next PR

A future PR can add PlanningScene collision objects or a more robust
pose-to-planning adapter.
