#!/usr/bin/env bash

set -euo pipefail

cat <<'EOF'
Benchmark profile suite workflow
================================

Run one launch at a time. Stop each launch before starting the next profile.
These commands record planning diagnostics only; trajectories are not executed.

1. Baseline profile
   Terminal 1:
     ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py
   Terminal 2:
     MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/baseline.csv bash scripts/run_seeded_planning_benchmark.sh

2. Narrow profile
   Terminal 1:
     ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_narrow.launch.py
   Terminal 2:
     MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/narrow.csv bash scripts/run_seeded_planning_benchmark.sh

3. Wide profile
   Terminal 1:
     ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_wide.launch.py
   Terminal 2:
     MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/wide.csv bash scripts/run_seeded_planning_benchmark.sh

4. Fixed-yaw profile
   Terminal 1:
     ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_fixed_yaw.launch.py
   Terminal 2:
     MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/fixed_yaw.csv bash scripts/run_seeded_planning_benchmark.sh

5. Compare recorded CSVs
   python3 scripts/compare_planning_benchmark_csvs.py \
     --input baseline=benchmark_results/baseline.csv \
     --input narrow=benchmark_results/narrow.csv \
     --input wide=benchmark_results/wide.csv \
     --input fixed_yaw=benchmark_results/fixed_yaw.csv
EOF
