# Planning benchmark recording

PR11 adds lightweight benchmark recording on top of the PR10 planning
diagnostics. The scripts record planning events only; they do not execute
trajectories.

Key topics:

- `/pre_grasp_plan_success`
- `/pre_grasp_planning_status`
- `/pre_grasp_planning_duration_ms`

Benchmark recording matters because it separates success, failure, and skipped
events, measures planning duration, and provides a repeatable baseline for
future comparisons when dynamic PlanningScene updates or new planners are
added.

```text
/panda_pre_grasp_pose
     │
     ▼
pre_grasp_planning_node
     │
     ├── /pre_grasp_plan_success
     ├── /pre_grasp_planning_status
     └── /pre_grasp_planning_duration_ms
               │
               ▼
record_planning_diagnostics_csv.py
               │
               ▼
planning_diagnostics.csv
               │
               ▼
summarize_planning_diagnostics_csv.py
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the Panda planning demo

Start the demo first:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

## Run a benchmark

In another terminal:

```bash
bash scripts/check_planning_benchmark_tools.sh
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/planning_diagnostics.csv bash scripts/run_planning_benchmark.sh
```

## Manual recording and summary

```bash
python3 scripts/record_planning_diagnostics_csv.py --output benchmark_results/planning_diagnostics.csv --max-events 20 --timeout-sec 120
python3 scripts/summarize_planning_diagnostics_csv.py --input benchmark_results/planning_diagnostics.csv
```

The benchmark scripts assume the Panda planning demo is already running.

## Reproducible seeded benchmark profile

PR12 adds a deterministic launch profile for more repeatable benchmark CSV
results. It uses seeded fake perception parameters and the same PR11 recording
tools. See
[reproducible_benchmark_profile.md](reproducible_benchmark_profile.md).

## Comparing multiple benchmark CSV files

PR13 adds deterministic baseline, narrow, wide, and fixed-yaw benchmark
profiles plus a CSV comparison helper. This makes it possible to compare
success/failure/skipped counts and duration statistics across repeatable
profile runs.

See [benchmark_profile_suite.md](benchmark_profile_suite.md).

## Markdown report export

PR19 adds Markdown report export for CSV comparisons:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input baseline=benchmark_results/baseline.csv \
  --input narrow=benchmark_results/narrow.csv \
  --output-markdown benchmark_results/benchmark_report.md
```

See [benchmark_report_export.md](benchmark_report_export.md).

This benchmark tooling is deliberately minimal:

- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no trajectory execution
- no dynamic PlanningScene updates
