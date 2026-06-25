# MoveIt2 planner settings

PR20 makes key MoveIt2 planner settings explicit in the plan-only pre-grasp
planning bridge. These settings are useful for benchmark profiles because CSV
results can now record which planner configuration produced each diagnostic
event.

`pre_grasp_planning_node` parameters:

| parameter | default | purpose |
| --- | --- | --- |
| `planner_id` | `""` | Optional MoveIt2 planner ID. Empty uses the MoveIt2 default. |
| `num_planning_attempts` | `1` | Number of planning attempts requested from MoveIt2. Clamped to at least `1`. |
| `max_velocity_scaling_factor` | `1.0` | Velocity scaling factor in `(0.0, 1.0]`. Invalid values clamp to `1.0`. |
| `max_acceleration_scaling_factor` | `1.0` | Acceleration scaling factor in `(0.0, 1.0]`. Invalid values clamp to `1.0`. |

The node applies these settings before planning:

- `setPlannerId(...)` when `planner_id` is non-empty
- `setNumPlanningAttempts(...)`
- `setMaxVelocityScalingFactor(...)`
- `setMaxAccelerationScalingFactor(...)`

The values are included in `/pre_grasp_planning_status`, recorded by
`record_planning_diagnostics_csv.py`, and included as metadata in Markdown
benchmark reports when available.

## Run with default planner settings

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

The default launch remains plan-only and uses the default values above.
Benchmark-specific launch profiles can set different values in
`adaptive_assembly_planning/launch/pre_grasp_planning.launch.py` or include the
node with custom parameters.

## Planner-settings benchmark profiles

PR21 adds deterministic launch profiles for comparing default attempts,
increased planning attempts, and slower velocity/acceleration scaling. See
[planner_settings_benchmark_profiles.md](planner_settings_benchmark_profiles.md).

## Validate

With the Panda planning demo already running:

```bash
bash scripts/check_planning_diagnostics.sh
python3 scripts/check_planning_status_format.py
bash scripts/check_planner_parameter_status.sh
```

This PR remains plan-only:

- trajectories are not executed
- no Gazebo
- no ros2_control integration for this project
- no real hardware
