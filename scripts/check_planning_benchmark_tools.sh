#!/usr/bin/env bash

set -euo pipefail

required_scripts=(
  "scripts/record_planning_diagnostics_csv.py"
  "scripts/summarize_planning_diagnostics_csv.py"
  "scripts/run_planning_benchmark.sh"
)

for script_path in "${required_scripts[@]}"; do
  if [[ ! -f "${script_path}" ]]; then
    echo "FAIL: missing ${script_path}"
    exit 1
  fi

  if [[ ! -x "${script_path}" ]]; then
    echo "FAIL: ${script_path} is not executable"
    exit 1
  fi

  echo "PASS: ${script_path} exists and is executable"
done

echo "PASS: planning benchmark tools are available"
