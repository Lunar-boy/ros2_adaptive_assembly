# Benchmark profile suite

PR13 adds multiple deterministic benchmark profiles for comparing plan-only
MoveIt2 behavior across target-pose distributions.

The profiles are:

- baseline: `adaptive_assembly_benchmark_params.yaml`, seed `42`, moderate x/y
  target range
- narrow: `adaptive_assembly_benchmark_narrow_params.yaml`, seed `101`,
  narrower x/y target distribution
- wide: `adaptive_assembly_benchmark_wide_params.yaml`, seed `202`, wider x/y
  target distribution
- fixed-yaw: `adaptive_assembly_benchmark_fixed_yaw_params.yaml`, seed `303`,
  yaw fixed to `0.0`

```text
benchmark profile YAML
     │
     ▼
benchmark launch profile
     │
     ▼
deterministic fake perception
     │
     ▼
planning diagnostics
     │
     ▼
CSV recorder
     │
     ▼
compare_planning_benchmark_csvs.py
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Validate the profile suite

```bash
bash scripts/check_benchmark_profile_suite.sh
bash scripts/check_benchmark_profile_params.sh
```

## Launch profiles

Run one profile at a time:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_narrow.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_wide.launch.py
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_fixed_yaw.launch.py
```

## Record CSV output

With one benchmark launch already running:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/baseline.csv bash scripts/run_seeded_planning_benchmark.sh
```

Repeat with a different output file for each profile.

PR14 adds a dynamic target PlanningScene object to the Panda planning demo.
Future comparisons can use the same profile suite to measure planning
diagnostics with that dynamic target object enabled.

## Compare CSV files

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input baseline=benchmark_results/baseline.csv \
  --input narrow=benchmark_results/narrow.csv \
  --input wide=benchmark_results/wide.csv \
  --input fixed_yaw=benchmark_results/fixed_yaw.csv
```

For a manual step-by-step checklist:

```bash
bash scripts/run_benchmark_profile_suite_hint.sh
```

Each benchmark launch is deterministic, and the benchmark scripts assume a
launch is already running. This PR remains plan-only:

- trajectories are not executed
- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no dynamic PlanningScene updates
