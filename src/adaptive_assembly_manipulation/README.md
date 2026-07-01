# adaptive_assembly_manipulation

Simulator-only logical gripper and object-grasp state for the PR35 execution
fixture. The node maps pre-grasp success to close/attach and aggregate execution
success to open/detach. Failures retain the last grasp state by default.

```bash
ros2 launch adaptive_assembly_manipulation logical_grasp_lifecycle.launch.py
```

Published retained topics are `/gripper_command`, `/gripper_command_status`,
`/object_grasp_state`, `/object_grasp_attached`, and
`/logical_grasp_lifecycle_status`.

This package does not control a physical gripper, send gripper action goals,
attach Gazebo objects, alter contact physics, or support real hardware.
