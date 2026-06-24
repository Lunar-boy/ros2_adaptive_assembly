#!/usr/bin/env bash

set -euo pipefail

MAX_EVENTS="${MAX_EVENTS:-20}"
TIMEOUT_SEC="${TIMEOUT_SEC:-120}"
OUTPUT="${OUTPUT:-benchmark_results/seeded_planning_diagnostics.csv}"

echo "This helper records a seeded planning benchmark."
echo "Start the seeded Panda planning benchmark launch first:"
echo "  ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py"
echo

MAX_EVENTS="${MAX_EVENTS}" \
TIMEOUT_SEC="${TIMEOUT_SEC}" \
OUTPUT="${OUTPUT}" \
  bash scripts/run_planning_benchmark.sh
