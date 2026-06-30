# Adaptive assembly execution

This package provides two optional consumers for the exported Panda assembly
sequence trajectories:

- `dry_run_sequence_executor_node` validates and simulates the `pre_grasp` and
  `assembly` stages using ROS 2 messages only.
- `ros2_control_sequence_executor_node` validates the same stages and can send
  them in order to a simulated `FollowJointTrajectory` controller.

The ros2_control node is an execution bridge for an externally running
simulator/controller stack. It does not launch Gazebo, define ros2_control
hardware, or support real hardware. See
[`docs/dry_run_execution.md`](../../docs/dry_run_execution.md) and
[`docs/gazebo_ros2_control_execution.md`](../../docs/gazebo_ros2_control_execution.md)
for launch and validation instructions.
