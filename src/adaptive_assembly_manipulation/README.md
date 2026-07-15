# adaptive_assembly_manipulation

Simulator-only logical gripper lifecycle, gripper controller action bridge, and
Gazebo kinematic object attachment.
The lifecycle maps pre-grasp success to close/attach and aggregate execution
success to open/detach. The Gazebo layer follows `world -> panda_hand` plus a
configurable fixed local hand-to-object offset while attached and leaves the
object at its last world pose when detached. Zero offset preserves the original
behavior.

```bash
ros2 launch adaptive_assembly_manipulation logical_grasp_lifecycle.launch.py
ros2 launch adaptive_assembly_manipulation gazebo_attach_detach.launch.py
ros2 launch adaptive_assembly_manipulation gripper_action_bridge.launch.py
```

Published retained topics are `/gripper_command`, `/gripper_command_status`,
`/object_grasp_state`, `/object_grasp_attached`, and
`/logical_grasp_lifecycle_status`.

Gazebo attachment uses repeated set-pose calls and is not physics-accurate. The
offset is a tunable visual-correctness aid for simplified hand geometry, not a
physical grasp.

`gripper_action_bridge_node` consumes logical open/close strings from
`/gripper_command` and sends two-finger `FollowJointTrajectory` goals to
`/panda_gripper_controller/follow_joint_trajectory`. The simulator contract is
the ordered list `panda_finger_joint1`, `panda_finger_joint2`; every open or
close point contains two equal positions. Incomplete, duplicate, reordered, or
unexpected joint lists are rejected at startup. Its retained outputs are:

- `/physical_gripper_command_status` (`std_msgs/msg/String`)
- `/physical_gripper_command_success` (`std_msgs/msg/Bool`)
- `/physical_gripper_closed` (`std_msgs/msg/Bool`)

The full physical Gazebo profile enables contact-aware close classification.
An accepted goal-tolerance abort becomes `contact_limited_success` only after
fresh, settled, bilateral contact on the exact configured target model.
Reusable profiles keep this disabled by default, and opening remains strict.
Close contact does not verify lift, slip, retention, or whole-task success.
The package does not alter contact physics or support real hardware. See
[`docs/gripper_action_bridge.md`](../../docs/gripper_action_bridge.md) for its
parameters, manual demo, and validation commands.
