#!/usr/bin/env bash

set -euo pipefail

if ! prefix="$(ros2 pkg prefix adaptive_assembly_bringup 2>/dev/null)"; then
  echo "FAIL: adaptive_assembly_bringup is not discoverable"
  exit 1
fi

launch_dir="${prefix}/share/adaptive_assembly_bringup/launch"
launch_files=(
  "adaptive_assembly_panda_planning_benchmark_attempts1.launch.py"
  "adaptive_assembly_panda_planning_benchmark_attempts5.launch.py"
  "adaptive_assembly_panda_planning_benchmark_slow_scaling.launch.py"
)

for launch_file in "${launch_files[@]}"; do
  path="${launch_dir}/${launch_file}"
  if [[ ! -f "${path}" ]]; then
    echo "FAIL: missing installed launch file: ${path}"
    exit 1
  fi
  echo "PASS: installed launch file exists: ${path}"
done

echo "PASS: planner-settings benchmark profiles are installed"
