#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_pipeline_hint() {
  echo "Start the pipeline first with:"
  echo "  ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py"
}

echo "Running pipeline topic checks..."
if ! "${SCRIPT_DIR}/check_pipeline_topics.sh"; then
  print_pipeline_hint
  exit 1
fi

echo "Running pipeline offset checks..."
if ! python3 "${SCRIPT_DIR}/check_pipeline_offsets.py"; then
  print_pipeline_hint
  exit 1
fi

echo "PASS: pipeline validation completed successfully"
