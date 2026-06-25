#!/usr/bin/env bash

set -euo pipefail

if ! package_prefix="$(ros2 pkg prefix adaptive_assembly_bringup 2>/dev/null)"; then
  echo "FAIL: adaptive_assembly_bringup is not discoverable"
  exit 1
fi

echo "PASS: adaptive_assembly_bringup is available at ${package_prefix}"

launch_dir="${package_prefix}/share/adaptive_assembly_bringup/launch"
required_launches=(
  "adaptive_assembly_panda_planning_benchmark_no_dynamic_target.launch.py"
  "adaptive_assembly_panda_planning_benchmark_with_dynamic_target.launch.py"
)

for launch_file in "${required_launches[@]}"; do
  full_path="${launch_dir}/${launch_file}"
  if [[ ! -f "${full_path}" ]]; then
    echo "FAIL: missing installed launch file ${full_path}"
    exit 1
  fi

  echo "PASS: found ${full_path}"
done

echo "PASS: dynamic target A/B benchmark launch files are installed"
