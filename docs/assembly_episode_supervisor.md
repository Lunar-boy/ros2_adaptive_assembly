# Assembly Episode Supervisor

## Scope

`assembly_episode_supervisor_node` is a passive, simulator-only status
aggregator. It observes an already-running assembly pipeline and publishes one
retained terminal episode result. It does not call services, orchestrate nodes,
publish target poses, send trajectories, or trigger recovery.

The status fields follow
[`assembly_episode_status_schema.md`](assembly_episode_status_schema.md).

## Subscribed topics

| Topic | Type |
|---|---|
| `/assembly_sequence_planning_status` | `std_msgs/String` |
| `/assembly_sequence_stage_status` | `std_msgs/String` |
| `/assembly_ros2_control_execution_status` | `std_msgs/String` |
| `/assembly_ros2_control_execution_success` | `std_msgs/Bool` |
| `/assembly_ros2_control_execution_duration_ms` | `std_msgs/Float64` |
| `/assembly_ros2_control_execution_stage_status` | `std_msgs/String` |
| `/logical_grasp_lifecycle_status` | `std_msgs/String` |
| `/object_grasp_attached` | `std_msgs/Bool` |
| `/gazebo_attach_detach_status` | `std_msgs/String` |
| `/gazebo_object_attached` | `std_msgs/Bool` |
| `/gazebo_attach_pose_error_mm` | `std_msgs/Float64` |
| `/assembly_insertion_status` | `std_msgs/String` |
| `/assembly_insertion_success` | `std_msgs/Bool` |
| `/assembly_insertion_error_mm` | `std_msgs/Float64` |
| `/assembly_insertion_error_deg` | `std_msgs/Float64` |

## Published topics

The retained outputs are `/assembly_episode_status` (`String`),
`/assembly_episode_success` (`Bool`), `/assembly_episode_duration_ms`
(`Float64`), and `/assembly_episode_failure_reason` (`String`). Stage changes
are published as a normal, non-retained `String` stream on
`/assembly_episode_stage_status`.

## Terminal rules

Success requires every enabled requirement. By default this means successful
planning; successful pre-grasp and assembly execution stages plus both
execution terminal-success signals; logical-grasp release after attachment;
successful Gazebo attachment; and successful insertion evaluation.

A required planning, execution, grasp/attachment, or insertion subsystem's
deterministic terminal non-success produces `event=failure` with
`planning_failed`, `execution_failed`, `grasp_attach_failed`, or
`insertion_failed`. If no terminal result is reached within
`episode_timeout_sec`, the result is `event=timeout` with
`failure_reason=episode_timeout`. A process publishes at most one terminal
result.

## Run and validate

```bash
ros2 launch adaptive_assembly_episode assembly_episode_supervisor.launch.py
bash scripts/check_assembly_episode_supervisor_available.sh
python3 scripts/check_assembly_episode_supervisor_success_path.py
python3 scripts/check_assembly_episode_supervisor_failure_path.py
python3 scripts/check_assembly_episode_supervisor_timeout_path.py
```

The path checks start isolated supervisor processes and inject synthetic
upstream messages, so they do not require the full simulation.

## Limitations

- Simulator-only; `simulated_only:=false` is rejected.
- No runtime orchestration.
- No Gazebo achieved/final object-pose observer.
- No benchmark recorder.
- No visual servoing.
- No force control or contact-rich insertion.
- No real-hardware support.
