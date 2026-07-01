# Recovery orchestration and simulated retry loop

PR37 connects retained recovery supervisor actions to bounded, simulator-only
retry requests. The orchestrator does not plan or execute trajectories. It
resets existing logical PlanningScene state where required and asks fake
perception to publish one fresh target pose, which restarts the existing
pose-driven planning pipeline.

## Launch

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_recovery_orchestration_demo.launch.py
```

The full demo composes the existing recovery supervisor demo and the new
orchestrator. Fake perception in that pipeline exposes
`/publish_target_pose_once`. The supervisor's own optional reset calls remain
disabled so each reset is owned by the orchestrator.

The isolated validation scripts launch only fake perception or the
orchestrator with fixture services; they do not require MoveIt2 or Gazebo.

## Action mapping

| Recovery action | Ordered orchestration |
| --- | --- |
| `clear_dynamic_target_and_retry` | clear dynamic scene, publish target pose |
| `reset_scene_and_retry` | clear dynamic scene, clear static scene, reapply static scene, publish target pose |
| `reset_planning_scene` | clear dynamic scene, clear static scene, reapply static scene, publish target pose |
| `discard_trajectories_and_replan` | publish discard/replan status, publish target pose; no trajectory topic mutation |

Only semicolon-delimited messages with `event=recovery_action` and a recognized
`action` are processed. `max_retry_attempts` bounds accepted actions. Further
duplicate actions publish `event=retry_exhausted` and do not call services.

## Interfaces

Subscribed topic:

- `/assembly_recovery_action` (`std_msgs/msg/String`)

Retained published topics:

- `/recovery_orchestration_status` (`std_msgs/msg/String`)
- `/recovery_retry_requested` (`std_msgs/msg/Bool`)

Fake perception service:

- `/publish_target_pose_once` (`std_srvs/srv/Trigger`)

The terminal status uses deterministic fields including `event`, `action`,
`attempt`, `services_ok`, `target_pose_triggered`, `simulated_only=true`, and
`real_hardware=false`.

## Parameters

| Parameter | Default |
| --- | --- |
| `recovery_action_topic` | `/assembly_recovery_action` |
| `orchestration_status_topic` | `/recovery_orchestration_status` |
| `retry_requested_topic` | `/recovery_retry_requested` |
| `clear_dynamic_scene_service` | `/clear_dynamic_target_scene` |
| `clear_static_scene_service` | `/clear_static_planning_scene` |
| `reapply_static_scene_service` | `/reapply_static_planning_scene` |
| `publish_target_pose_service` | `/publish_target_pose_once` |
| `max_retry_attempts` | `1` |
| `service_timeout_sec` | `2.0` |
| `enable_service_calls` | `true` |
| `simulated_only` | `true` |

`simulated_only:=false` is rejected at startup.

## Validation

```bash
bash scripts/check_recovery_orchestrator_available.sh
python3 scripts/check_fake_pose_trigger_service.py
python3 scripts/check_recovery_orchestrator_retry_path.py
python3 scripts/check_recovery_orchestrator_exhausted_path.py
```

## Limitations

- no Gazebo object attachment;
- no real gripper actuation;
- no contact-rich insertion;
- no real hardware support;
- retry means simulated scene reset plus fake-perception target republish.
