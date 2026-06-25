#!/usr/bin/env bash

set -euo pipefail

if ! package_prefix="$(ros2 pkg prefix adaptive_assembly_planning 2>/dev/null)"; then
  echo "FAIL: adaptive_assembly_planning is not discoverable"
  exit 1
fi

echo "PASS: adaptive_assembly_planning is available at ${package_prefix}"

launch_file="${package_prefix}/share/adaptive_assembly_planning/launch/dynamic_target_scene.launch.py"
executable="${package_prefix}/lib/adaptive_assembly_planning/dynamic_target_scene_node"

if [[ ! -f "${launch_file}" ]]; then
  echo "FAIL: missing installed launch file ${launch_file}"
  exit 1
fi
echo "PASS: found ${launch_file}"

if [[ ! -x "${executable}" ]]; then
  echo "FAIL: missing executable ${executable}"
  exit 1
fi
echo "PASS: found executable ${executable}"

echo "PASS: dynamic target scene node and launch file are installed"
