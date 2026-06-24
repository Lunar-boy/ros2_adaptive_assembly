# Plan-only pre-grasp planning bridge

PR6 adds a minimal MoveIt2 bridge that subscribes to `/pre_grasp_pose` and asks
MoveIt2 to plan a Panda arm trajectory to that pose. The node reports whether
planning succeeded, but it does not execute trajectories.

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
pre_grasp_planning_node
     │
     ├── MoveIt2 plan request to panda_arm
     └── /pre_grasp_plan_success
```

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
- The Panda MoveIt2 demo starts
- `pre_grasp_planning_node` attempts planning when `/pre_grasp_pose` changes
  enough
- `/pre_grasp_plan_success` publishes `true` or `false`
- No execution occurs

## Next PR

A future PR can add PlanningScene collision objects or a more robust
pose-to-planning adapter.
