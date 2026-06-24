# Panda pre-grasp pose adapter

`/pre_grasp_pose` is a task-level pose derived from the target object. It comes
from the fake perception/task pipeline in frame `world`, and its orientation is
copied from the object. That is useful for task abstraction, but object
orientation is not always a suitable Panda end-effector orientation for MoveIt2
IK and planning.

`/panda_pre_grasp_pose` is robot-specific. PR7 introduced the adapter and PR8
makes it frame-aware: the default launch now publishes the adapted pose in frame
`panda_link0`, which is appropriate for the standard Panda MoveIt2 demo.

The numeric position is currently copied from the task pose, with optional
configured offsets. The frame and orientation are adapted for the Panda demo.

```text
/pre_grasp_pose
     │ frame: world
     ▼
panda_pre_grasp_pose_adapter_node
     │ output_frame_id: panda_link0
     ▼
/panda_pre_grasp_pose
     │ frame: panda_link0
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
bash scripts/check_panda_adapted_pose_frame.sh
python3 scripts/check_panda_pose_adapter_orientation.py --expected-frame panda_link0
ros2 topic echo --once /panda_pre_grasp_pose
```

The launch remains plan-only:

- trajectories are not executed
- Gazebo is not used
- ros2_control is not used by this project
- PlanningScene collision objects are not included yet
- real hardware is not used

## Next PR

A future PR can add simple PlanningScene collision objects after the planning
frame is stable.
