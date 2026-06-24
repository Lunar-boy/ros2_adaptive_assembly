#!/usr/bin/env bash

set -euo pipefail

MAX_EVENTS="${MAX_EVENTS:-20}"
TIMEOUT_SEC="${TIMEOUT_SEC:-120}"
OUTPUT="${OUTPUT:-benchmark_results/planning_diagnostics.csv}"

echo "Recording planning benchmark..."
echo "  MAX_EVENTS=${MAX_EVENTS}"
echo "  TIMEOUT_SEC=${TIMEOUT_SEC}"
echo "  OUTPUT=${OUTPUT}"

if ! python3 scripts/record_planning_diagnostics_csv.py \
  --output "${OUTPUT}" \
  --max-events "${MAX_EVENTS}" \
  --timeout-sec "${TIMEOUT_SEC}"; then
  echo "FAIL: no planning benchmark events were recorded"
  echo "Start the Panda planning demo first:"
  echo "  ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py"
  exit 1
fi

python3 scripts/summarize_planning_diagnostics_csv.py --input "${OUTPUT}"
