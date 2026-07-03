# Adaptive assembly execution

This package provides two optional consumers for the exported Panda assembly
sequence trajectories:

- `dry_run_sequence_executor_node` validates and simulates the `pre_grasp` and
  `assembly` stages using ROS 2 messages only.
- `ros2_control_sequence_executor_node` validates the same stages and can send
  them in order to a simulated `FollowJointTrajectory` controller.
- `simulated_follow_joint_trajectory_server_node` provides deterministic
  success, rejection, failure, and timeout modes for local action-level checks.
- `wait_for_gazebo_controller_ready_node` publishes retained readiness only
  after both controllers, the action server, and valid Panda joint states exist.

The ros2_control node can use the local fixture or an externally running
simulator/controller stack. It does not define ros2_control hardware, move a
Gazebo robot, or support real hardware. See
[`docs/dry_run_execution.md`](../../docs/dry_run_execution.md) and
[`docs/gazebo_ros2_control_execution.md`](../../docs/gazebo_ros2_control_execution.md)
for launch and validation instructions.

Initial execution can optionally be gated by retained target-sync status with
`require_target_sync_success:=true`. The executor then waits until all
trajectory and joint-state prerequisites are ready and
`/gazebo_target_sync_status` reports `event=success`. This gate is disabled by
default and does not affect transitions after the pre-grasp goal is sent.
