# Planner-settings benchmark profiles

PR21 adds deterministic benchmark launch profiles that vary MoveIt2 planner
settings while keeping the adaptive assembly pipeline plan-only.

The profiles all use the baseline seeded benchmark parameters from
`adaptive_assembly_benchmark_params.yaml`.

## Profiles

- `attempts1`
  - launch:
    `adaptive_assembly_panda_planning_benchmark_attempts1.launch.py`
  - `num_planning_attempts: 1`
  - velocity/acceleration scaling: `1.0`

- `attempts5`
  - launch:
    `adaptive_assembly_panda_planning_benchmark_attempts5.launch.py`
  - `num_planning_attempts: 5`
  - velocity/acceleration scaling: `1.0`

- `slow_scaling`
  - launch:
    `adaptive_assembly_panda_planning_benchmark_slow_scaling.launch.py`
  - `num_planning_attempts: 1`
  - `max_velocity_scaling_factor: 0.25`
  - `max_acceleration_scaling_factor: 0.25`

## Validate

```bash
bash scripts/check_planner_benchmark_profiles.sh
```

## Record CSVs

Run one launch at a time, then record its CSV from another terminal.

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts1.launch.py
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/attempts1.csv bash scripts/run_seeded_planning_benchmark.sh
```

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts5.launch.py
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/attempts5.csv bash scripts/run_seeded_planning_benchmark.sh
```

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_slow_scaling.launch.py
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/slow_scaling.csv bash scripts/run_seeded_planning_benchmark.sh
```

For a manual checklist:

```bash
bash scripts/run_planner_settings_benchmark_hint.sh
```

## Compare and export a report

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input attempts1=benchmark_results/attempts1.csv \
  --input attempts5=benchmark_results/attempts5.csv \
  --input slow_scaling=benchmark_results/slow_scaling.csv \
  --output-markdown benchmark_results/planner_settings_report.md
```

The generated CSVs and Markdown reports are ignored if written under
`benchmark_results/`.

This remains benchmark/reporting infrastructure only:

- trajectories are not executed
- no Gazebo
- no ros2_control integration for this project
- no real hardware
