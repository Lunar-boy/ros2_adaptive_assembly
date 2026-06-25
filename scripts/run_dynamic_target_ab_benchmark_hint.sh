#!/usr/bin/env bash

set -euo pipefail

cat <<'EOF'
Dynamic target A/B benchmark workflow
=====================================

Run one launch at a time. Stop each launch before starting the next profile.
These commands record planning diagnostics only; trajectories are not executed.

1. Start the benchmark without the dynamic target collision object:
   ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_no_dynamic_target.launch.py

2. In another terminal, record:
   MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/no_dynamic_target.csv bash scripts/run_seeded_planning_benchmark.sh

3. Stop the no-dynamic launch.

4. Start the benchmark with the dynamic target collision object:
   ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_with_dynamic_target.launch.py

5. In another terminal, record:
   MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/with_dynamic_target.csv bash scripts/run_seeded_planning_benchmark.sh

6. Compare:
   python3 scripts/compare_planning_benchmark_csvs.py --input no_dynamic=benchmark_results/no_dynamic_target.csv --input with_dynamic=benchmark_results/with_dynamic_target.csv
EOF
