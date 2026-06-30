# Adaptive assembly execution

This package provides a dry-run consumer for the exported Panda assembly
sequence trajectories. It validates and simulates the `pre_grasp` and
`assembly` stages using ROS 2 messages only.

The package does not command a robot, execute a MoveIt trajectory, connect to a
controller, or provide Gazebo or hardware integration. See
[`docs/dry_run_execution.md`](../../docs/dry_run_execution.md) for launch and
validation instructions.
