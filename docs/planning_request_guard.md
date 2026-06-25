# Planning request guard

PR23 adds an optional safety guard to `pre_grasp_planning_node`. The guard
rejects invalid or unsafe pre-grasp requests before the node calls MoveIt2.

The default remains unchanged:

- `enable_request_guard: false`
- existing planning behavior is preserved
- requests still flow through the existing replan-distance and plan-only logic

## Guard parameters

- `enable_request_guard`, default `false`
- `required_frame_id`, default `""`
- `workspace_min_x`, default `-10.0`
- `workspace_max_x`, default `10.0`
- `workspace_min_y`, default `-10.0`
- `workspace_max_y`, default `10.0`
- `workspace_min_z`, default `-10.0`
- `workspace_max_z`, default `10.0`
- `min_quaternion_norm`, default `1e-6`

## Rejection conditions

When enabled, the guard rejects a request before MoveIt2 planning if:

- the pose frame is empty
- `required_frame_id` is set and the pose frame differs
- x/y/z is not finite
- x/y/z is outside the configured workspace bounds
- quaternion norm is below `min_quaternion_norm`

Rejected requests publish:

- `/pre_grasp_plan_success`: `false`
- `/pre_grasp_planning_status`: `event=guard_rejected`
- `/pre_grasp_planning_duration_ms`: `0.0`

MoveIt2 planning is not called for rejected requests.

## Status fields

Planning status messages now include:

- `guard_enabled`
- `guard_passed`
- `guard_reason`

For accepted requests, `guard_reason=none`. For rejected requests,
`guard_reason` identifies the rejection cause, such as `frame_mismatch` or
`outside_workspace`.

## Guarded benchmark launch

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_guarded.launch.py
```

This launch uses the baseline seeded benchmark profile and enables:

- `required_frame_id: panda_link0`
- x workspace: `0.20` to `0.80`
- y workspace: `-0.50` to `0.50`
- z workspace: `0.10` to `0.60`

## Validate

```bash
bash scripts/check_guarded_benchmark_launch.sh
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
bash scripts/check_planning_request_guard_status.sh
```

Benchmark recording still uses the existing CSV tools:

```bash
MAX_EVENTS=5 TIMEOUT_SEC=60 OUTPUT=/tmp/guarded.csv bash scripts/run_seeded_planning_benchmark.sh
python3 scripts/compare_planning_benchmark_csvs.py --input guarded=/tmp/guarded.csv --output-markdown /tmp/guarded_report.md
```

## Scope

This PR adds a request validation layer only. It does not execute trajectories,
add Gazebo, add ros2_control integration for this project, require real
hardware, or change the default planning launch behavior.
