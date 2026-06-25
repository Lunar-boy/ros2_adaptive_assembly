#!/usr/bin/env bash

set -euo pipefail

if ! package_prefix="$(ros2 pkg prefix adaptive_assembly_bringup 2>/dev/null)"; then
  echo "FAIL: adaptive_assembly_bringup is not discoverable"
  exit 1
fi

echo "PASS: adaptive_assembly_bringup is available at ${package_prefix}"

required_files=(
  "share/adaptive_assembly_bringup/config/adaptive_assembly_benchmark_params.yaml"
  "share/adaptive_assembly_bringup/config/adaptive_assembly_benchmark_narrow_params.yaml"
  "share/adaptive_assembly_bringup/config/adaptive_assembly_benchmark_wide_params.yaml"
  "share/adaptive_assembly_bringup/config/adaptive_assembly_benchmark_fixed_yaw_params.yaml"
  "share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_planning_benchmark.launch.py"
  "share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_planning_benchmark_narrow.launch.py"
  "share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_planning_benchmark_wide.launch.py"
  "share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_planning_benchmark_fixed_yaw.launch.py"
)

for relative_path in "${required_files[@]}"; do
  full_path="${package_prefix}/${relative_path}"
  if [[ ! -f "${full_path}" ]]; then
    echo "FAIL: missing installed file ${full_path}"
    exit 1
  fi

  echo "PASS: found ${full_path}"
done

echo "PASS: benchmark profile suite is installed"
