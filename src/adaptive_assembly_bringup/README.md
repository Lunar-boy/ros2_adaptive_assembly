# adaptive_assembly_bringup

Launch entry points for the current adaptive assembly pipeline.

This package connects:

- `adaptive_assembly_perception`, which publishes `/target_pose` and broadcasts
  TF `world -> target_object`
- `adaptive_assembly_task`, which subscribes to `/target_pose` and publishes
  `/pre_grasp_pose` and `/assembly_pose`

This is intentionally the non-MoveIt pipeline launch. It does not start Gazebo,
RViz, a robot model, ros2_control, or MoveIt2.

The optional Gazebo workcell wrapper is separate:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_workcell_demo.launch.py
```

It starts the primitive-geometry workcell from `adaptive_assembly_sim` and, by
default, the same non-MoveIt pipeline. Use `launch_pipeline:=false` for the
workcell alone. The Gazebo target is static and is not synchronized with
`/target_pose`. This wrapper does not spawn a robot, start ros2_control, include
the sequence executor, or execute trajectories.

The optional
`adaptive_assembly_panda_ros2_control_execution.launch.py` profile is separate
from the non-MoveIt pipeline. It composes known-reachable sequence planning with
a simulator-only execution bridge. It can connect to an already running Panda
`FollowJointTrajectory` controller. It adds no Gazebo launch or controller
configuration; the separate PR34 workcell wrapper also does not provide a
controller. The included existing MoveIt demo may provide its own mock
ros2_control stack.

The PR35 local success fixture is:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_ros2_control_success_demo.launch.py
```

It starts a deterministic simulated action server and verifies both planned
stages. The optional
`adaptive_assembly_gazebo_ros2_control_success_demo.launch.py` wrapper adds the
visual-only workcell; it does not physically move a Panda in Gazebo. Neither
launch supports real hardware, grippers, attach/detach, force control, or
contact-rich insertion.

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
