# Gripper abstraction and logical grasp lifecycle

PR36 adds a simulator-only state layer over the PR35 action success fixture. It
turns execution events into manipulation semantics without controlling a
gripper or changing simulation physics.

## Run

```bash
source install/setup.bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_logical_grasp_demo.launch.py
```

The standalone observer is also available as:

```bash
ros2 launch adaptive_assembly_manipulation \
  logical_grasp_lifecycle.launch.py
```

## Lifecycle

1. Startup publishes open and detached state.
2. `event=success;stage=pre_grasp` closes the logical gripper and attaches the
   object to `attach_parent_frame`.
3. Aggregate `event=success` opens the logical gripper and releases the object
   to `release_parent_frame`.
4. Aggregate `event=failure` or `event=timeout` publishes failure state and
   retains the last grasp state. Set `detach_on_failure:=true` to logically
   release instead.

All commands and attachments are state messages only. The node sends no action
goals, does not actuate Panda fingers, does not attach an SDF entity, and does
not implement contact, force control, real hardware, recovery, or retries.

## Interfaces

Inputs:

- `/assembly_ros2_control_execution_stage_status` (`std_msgs/msg/String`)
- `/assembly_ros2_control_execution_status` (`std_msgs/msg/String`)

Retained outputs:

- `/gripper_command` (`std_msgs/msg/String`)
- `/gripper_command_status` (`std_msgs/msg/String`)
- `/object_grasp_state` (`std_msgs/msg/String`)
- `/object_grasp_attached` (`std_msgs/msg/Bool`)
- `/logical_grasp_lifecycle_status` (`std_msgs/msg/String`)

All topic names are parameters. Object and frame parameters are `object_id`,
`gripper_id`, `attach_parent_frame`, and `release_parent_frame`.
`simulated_only` must remain `true`; the node rejects `false`.

## Validate

```bash
bash scripts/check_logical_grasp_lifecycle_available.sh
python3 scripts/check_logical_grasp_success_path.py
python3 scripts/check_logical_grasp_failure_path.py
```

The Python checks use isolated topics and synthetic status messages. They verify
retained startup state, attach/release transitions, and failure retention.
