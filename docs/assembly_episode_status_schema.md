# Assembly Episode Status Schema

## Purpose and scope

This document defines the stable topic and semicolon-delimited `key=value` status contract for a complete assembly episode. This is a schema-only contract: no episode supervisor, Gazebo pose observer, benchmark recorder, or runtime orchestration is implemented by this change.

## Episode topics

| Topic | Type | Durability and purpose |
|---|---|---|
| `/assembly_episode_status` | `std_msgs/msg/String` | Retained terminal aggregate status |
| `/assembly_episode_success` | `std_msgs/msg/Bool` | Retained aggregate success |
| `/assembly_episode_duration_ms` | `std_msgs/msg/Float64` | Retained aggregate duration |
| `/assembly_episode_stage_status` | `std_msgs/msg/String` | Non-retained stage transition/status events |
| `/assembly_episode_failure_reason` | `std_msgs/msg/String` | Retained reason string for non-success terminal outcomes |

Here, retained means a future publisher should use transient-local durability so late subscribers can receive the latest value. Stage events are non-retained and should use volatile durability.

## Status string format

Status strings contain semicolon-delimited `key=value` fields. Every episode status string requires:

- `event`
- `mode=assembly_episode`
- `stage`
- `simulated_only=true`
- `real_hardware=false`

Terminal `event` values are:

- `success`
- `failure`
- `timeout`
- `skipped`

Known `stage` values are:

- `init`
- `wait_target_pose`
- `wait_planning`
- `wait_execution_start`
- `wait_pre_grasp_success`
- `wait_grasp_attached`
- `wait_assembly_execution`
- `wait_release`
- `wait_insertion_evaluation`
- `terminal`

Stage status events may use descriptive non-terminal event values such as `ready`, `waiting`, or `stage_transition`. Aggregate terminal statuses must use one of the terminal event values and `stage=terminal`.

## Recommended fields

Boolean fields, when present, use only `true` or `false`:

- `target_pose_available`
- `planning_success`
- `pre_grasp_success`
- `assembly_success`
- `execution_success`
- `logical_grasp_attached`
- `logical_grasp_released`
- `gazebo_attach_success`
- `insertion_success`
- `episode_success`

Numeric fields, when present, must be parseable as floating-point values:

- `duration_ms`
- `insertion_error_mm`
- `insertion_error_deg`
- `gazebo_attach_pose_error_mm`

Optional context fields are:

- `episode_id`
- `trial_id`
- `profile`
- `seed`
- `failure_reason`
- `recovery_action`

## Aggregate success rule

`episode_success` is `true` only when all of the following are `true`: `planning_success`, `pre_grasp_success`, `assembly_success`, `execution_success`, `logical_grasp_released`, `gazebo_attach_success`, and `insertion_success`.

`logical_grasp_attached` is useful stage evidence but is not an independent terminal-success input because a successful episode ends with the logical grasp released.

## Failure reasons

Failure reasons use lower snake case. Examples include `planning_failed`, `execution_failed`, `grasp_attach_failed`, `release_failed`, `insertion_failed`, and `timeout_wait_planning`.

Every non-success terminal status (`failure`, `timeout`, or `skipped`) includes a non-empty `failure_reason`. The same reason is published on `/assembly_episode_failure_reason`.

## Examples

Ready/stage status:

```text
event=ready;mode=assembly_episode;stage=init;simulated_only=true;real_hardware=false;episode_id=episode_001
```

Success terminal status:

```text
event=success;mode=assembly_episode;stage=terminal;simulated_only=true;real_hardware=false;episode_id=episode_001;target_pose_available=true;planning_success=true;pre_grasp_success=true;assembly_success=true;execution_success=true;logical_grasp_attached=true;logical_grasp_released=true;gazebo_attach_success=true;insertion_success=true;episode_success=true;duration_ms=8421.5;insertion_error_mm=1.2;insertion_error_deg=0.8;gazebo_attach_pose_error_mm=0.6
```

Failure terminal status:

```text
event=failure;mode=assembly_episode;stage=terminal;simulated_only=true;real_hardware=false;episode_id=episode_002;planning_success=true;pre_grasp_success=true;assembly_success=true;execution_success=false;episode_success=false;duration_ms=4100.0;failure_reason=execution_failed
```

Timeout terminal status:

```text
event=timeout;mode=assembly_episode;stage=terminal;simulated_only=true;real_hardware=false;episode_id=episode_003;planning_success=false;episode_success=false;duration_ms=30000.0;failure_reason=timeout_wait_planning
```

## Limitations

This contract describes a pipeline that is:

- simulator-only;
- based on a logical gripper only;
- based on kinematic Gazebo attachment only;
- limited to final-pose geometric insertion evaluation only;
- not connected to a real camera;
- without visual servoing;
- without force control;
- without contact-rich insertion;
- without real hardware execution.

