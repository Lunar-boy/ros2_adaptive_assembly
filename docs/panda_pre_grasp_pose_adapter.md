# Panda pre-grasp pose adapter

`/pre_grasp_pose` is a task-level pose derived from the target object. Its
orientation is copied from the object, which is useful for task abstraction but
is not always a suitable Panda end-effector orientation for MoveIt2 IK and
planning.

PR7 adds `panda_pre_grasp_pose_adapter_node`, which converts the task-level pose
into a robot-specific `/panda_pre_grasp_pose`. This keeps the task layer generic
while giving the Panda planning bridge a more explicit robot-aware target.

```text
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
     ▼
MoveIt2 plan-only request to panda_arm
```

The default fixed quaternion is:

- `fixed_qx: 1.0`
- `fixed_qy: 0.0`
- `fixed_qz: 0.0`
- `fixed_qw: 0.0`

This is a simple configurable starting point. Later PRs can refine the adapter
with better approach directions, grasp frames, collision-aware offsets, or
PlanningScene context.

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
bash scripts/check_panda_adapted_pose.sh
python3 scripts/check_panda_pose_adapter_orientation.py
ros2 topic echo /panda_pre_grasp_pose
ros2 topic echo /pre_grasp_plan_success
```

The launch remains plan-only:

- trajectories are not executed
- Gazebo is not used
- PlanningScene collision objects are not included yet
- real hardware is not used
