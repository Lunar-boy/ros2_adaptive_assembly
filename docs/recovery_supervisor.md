# Closed-loop recovery supervisor

The `adaptive_assembly_recovery` package observes planning, dry-run execution,
and PlanningScene diagnostics. It classifies terminal outcomes and publishes a
deterministic recovery action. This is a supervision abstraction only: it does
not invoke MoveIt planning, retry a plan, discard a trajectory in another node,
execute motion, or command a robot.

## State machine

The exposed states are:

- `IDLE`
- `WAIT_FOR_PLAN`
- `PLAN_SUCCEEDED`
- `EXECUTION_SUCCEEDED`
- `RECOVER_PLANNING_FAILURE`
- `RECOVER_EXECUTION_FAILURE`
- `RECOVER_SCENE_FAILURE`
- `RECOVERY_ACTION_PUBLISHED`
- `RECOVERY_EXHAUSTED`

The supervisor begins in `IDLE`, immediately waits in `WAIT_FOR_PLAN`, records
planning success as `PLAN_SUCCEEDED`, and records a successful dry run as
`EXECUTION_SUCCEEDED`. Failure classifications publish a recovery state followed
by `RECOVERY_ACTION_PUBLISHED`. Once `max_recovery_attempts` is reached, the
next distinct failure produces `RECOVERY_EXHAUSTED`. Duplicate copies of the
same failure are ignored until progress is observed.

## Interfaces

Subscriptions (`std_msgs/msg/String`):

- `/assembly_sequence_planning_status`
- `/assembly_execution_status`
- `/dynamic_target_scene_status`
- `/planning_scene_audit_status`

Publications:

- `/assembly_recovery_status` (`std_msgs/msg/String`)
- `/assembly_recovery_action` (`std_msgs/msg/String`)
- `/assembly_recovery_success` (`std_msgs/msg/Bool`)

All recovery status messages are semicolon-delimited key/value fields and
include `real_execution=false`. Result publishers use transient-local
durability so late subscribers can inspect the current outcome.

## Classification and actions

| Input | Recovery action | Reason |
| --- | --- | --- |
| Pre-grasp planning failure | `reset_scene_and_retry` | `pre_grasp_planning_failed` |
| Assembly planning failure | `clear_dynamic_target_and_retry` | `assembly_planning_failed` |
| Dry-run execution failure | `discard_trajectories_and_replan` | `dry_run_execution_failed` |
| Dynamic scene or audit failure | `reset_planning_scene` | `scene_inconsistent` |

For example:

```text
event=recovery_action;state=RECOVERY_ACTION_PUBLISHED;action=clear_dynamic_target_and_retry;reason=assembly_planning_failed;attempt=1;service_calls=false;real_execution=false
```

An audit is treated as inconsistent only when its status is parseable, reports
`all_present=false`, and lists a non-`none` `missing` value.

## Parameters

| Parameter | Default |
| --- | --- |
| `planning_status_topic` | `/assembly_sequence_planning_status` |
| `execution_status_topic` | `/assembly_execution_status` |
| `dynamic_scene_status_topic` | `/dynamic_target_scene_status` |
| `planning_scene_audit_status_topic` | `/planning_scene_audit_status` |
| `recovery_status_topic` | `/assembly_recovery_status` |
| `recovery_action_topic` | `/assembly_recovery_action` |
| `recovery_success_topic` | `/assembly_recovery_success` |
| `clear_dynamic_scene_service` | `/clear_dynamic_target_scene` |
| `clear_static_scene_service` | `/clear_static_planning_scene` |
| `reapply_static_scene_service` | `/reapply_static_planning_scene` |
| `enable_service_calls` | `false` |
| `max_recovery_attempts` | `1` |
| `publish_heartbeat` | `true` |

With service calls disabled, the supervisor only publishes its decision. With
them enabled, scene-reset actions call the existing `std_srvs/srv/Trigger`
services asynchronously. `reset_scene_and_retry` and `reset_planning_scene`
call dynamic clear, static clear, then static reapply. The assembly-specific
action calls only dynamic clear. Each result is published as
`event=service_call`; unavailable services and two-second response timeouts are
reported without blocking the supervisor.

The service side effects reset PlanningScene objects only. They do not cause a
new planning request; future orchestration must consume the recovery action to
perform a real retry.

## Run and validate

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_recovery_supervisor_demo.launch.py
```

The demo uses the known-reachable dry-run sequence and keeps service calls
disabled. Its scene audit is disabled because that profile intentionally omits
the dynamic target collision object.

In another sourced shell:

```bash
bash scripts/check_recovery_supervisor_topics.sh
python3 scripts/check_recovery_supervisor_success_path.py
python3 scripts/check_recovery_supervisor_failure_injection.py
```

The failure injector publishes a retained synthetic assembly-stage planning
failure and verifies `action=clear_dynamic_target_and_retry`. No real execution
interface is introduced by the launch or validators.
