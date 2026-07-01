#!/usr/bin/env bash

set -euo pipefail

OUTPUT="${OUTPUT:-benchmark_results/contact_lite_insertion.csv}"
REPORT="${REPORT:-benchmark_results/contact_lite_insertion_summary.md}"
MAX_TRIALS="${MAX_TRIALS:-20}"
TIMEOUT_SEC="${TIMEOUT_SEC:-120}"

echo "Recording contact-lite insertion benchmark"
echo "  output CSV: ${OUTPUT}"
echo "  output report: ${REPORT}"
echo "  max trials: ${MAX_TRIALS}"
echo "  timeout: ${TIMEOUT_SEC}s"
echo
echo "Start in another terminal if it is not already running:"
echo "  ros2 launch adaptive_assembly_bringup \\"
echo "    adaptive_assembly_contact_lite_insertion_benchmark.launch.py"
echo

python3 scripts/record_contact_lite_insertion_csv.py \
  --output "${OUTPUT}" \
  --max-trials "${MAX_TRIALS}" \
  --timeout-sec "${TIMEOUT_SEC}"

python3 scripts/summarize_contact_lite_insertion_csv.py \
  --input "${OUTPUT}" \
  --output-markdown "${REPORT}"
