#!/usr/bin/env bash

set -euo pipefail

params_file="src/adaptive_assembly_bringup/config/adaptive_assembly_benchmark_params.yaml"

if [[ ! -f "${params_file}" ]]; then
  echo "FAIL: missing ${params_file}"
  exit 1
fi

required_patterns=(
  "random_seed: 42"
  "publish_period_sec: 2.0"
  "frame_id: world"
  "target_frame_id: target_object"
)

for pattern in "${required_patterns[@]}"; do
  if ! grep -q "${pattern}" "${params_file}"; then
    echo "FAIL: ${params_file} does not contain '${pattern}'"
    exit 1
  fi

  echo "PASS: ${params_file} contains '${pattern}'"
done

echo "PASS: seeded fake perception benchmark parameters are configured"
