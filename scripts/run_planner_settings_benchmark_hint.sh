#!/usr/bin/env bash

set -euo pipefail

cat <<'EOF'
Planner-settings benchmark profile workflow
===========================================

Run one launch at a time. Stop each launch before starting the next one.
The benchmarks are plan-only; trajectories are not executed.

1. Launch attempts1:
   ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts1.launch.py

2. Record attempts1:
   MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/attempts1.csv bash scripts/run_seeded_planning_benchmark.sh

3. Stop the attempts1 launch.

4. Launch attempts5:
   ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_attempts5.launch.py

5. Record attempts5:
   MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/attempts5.csv bash scripts/run_seeded_planning_benchmark.sh

6. Stop the attempts5 launch.

7. Launch slow_scaling:
   ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_slow_scaling.launch.py

8. Record slow_scaling:
   MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/slow_scaling.csv bash scripts/run_seeded_planning_benchmark.sh

9. Stop the slow_scaling launch.

10. Compare and export Markdown:
   python3 scripts/compare_planning_benchmark_csvs.py \
     --input attempts1=benchmark_results/attempts1.csv \
     --input attempts5=benchmark_results/attempts5.csv \
     --input slow_scaling=benchmark_results/slow_scaling.csv \
     --output-markdown benchmark_results/planner_settings_report.md
EOF
