# adaptive_assembly_bringup

Launch entry points for the current adaptive assembly pipeline.

This package connects:

- `adaptive_assembly_perception`, which publishes `/target_pose` and broadcasts
  TF `world -> target_object`
- `adaptive_assembly_task`, which subscribes to `/target_pose` and publishes
  `/pre_grasp_pose` and `/assembly_pose`

This is intentionally the non-MoveIt pipeline launch. It does not start Gazebo,
RViz, a robot model, ros2_control, or MoveIt2.

The optional
`adaptive_assembly_panda_ros2_control_execution.launch.py` profile is separate
from the non-MoveIt pipeline. It composes known-reachable sequence planning with
a simulator-only execution bridge. It can connect to an already running Panda
`FollowJointTrajectory` controller. It adds no Gazebo launch or controller
configuration; the included existing MoveIt demo may provide its own mock
ros2_control stack.

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py
```

## Validate

In another terminal:

```bash
source ~/ros2_adaptive_assembly_ws/install/setup.bash
ros2 topic echo --once /target_pose
ros2 topic echo --once /pre_grasp_pose
ros2 topic echo --once /assembly_pose
ros2 run tf2_ros tf2_echo world target_object
```

The expected pose offsets are:

- `pre_grasp_pose.z = target_pose.z + 0.20`
- `assembly_pose.z = target_pose.z + 0.05`

## Tests

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select adaptive_assembly_bringup --event-handlers console_direct+
colcon test-result --verbose
```
