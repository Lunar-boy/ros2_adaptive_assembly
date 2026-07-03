# adaptive_assembly_manipulation

Simulator-only logical gripper lifecycle and Gazebo kinematic object attachment.
The lifecycle maps pre-grasp success to close/attach and aggregate execution
success to open/detach. The Gazebo layer follows `world -> panda_hand` plus a
configurable fixed local hand-to-object offset while attached and leaves the
object at its last world pose when detached. Zero offset preserves the original
behavior.

```bash
ros2 launch adaptive_assembly_manipulation logical_grasp_lifecycle.launch.py
ros2 launch adaptive_assembly_manipulation gazebo_attach_detach.launch.py
```

Published retained topics are `/gripper_command`, `/gripper_command_status`,
`/object_grasp_state`, `/object_grasp_attached`, and
`/logical_grasp_lifecycle_status`.

Gazebo attachment uses repeated set-pose calls and is not physics-accurate. The
offset is a tunable visual-correctness aid for simplified hand geometry, not a
physical grasp. This package does not control a physical gripper, alter contact
physics, or support real hardware.
