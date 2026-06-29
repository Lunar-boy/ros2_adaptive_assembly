#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $1"
  exit 1
}

pass() {
  echo "PASS: $1"
}

if ! planning_prefix="$(ros2 pkg prefix adaptive_assembly_planning 2>/dev/null)"; then
  fail "adaptive_assembly_planning is not discoverable; build and source the workspace"
fi
if ! bringup_prefix="$(ros2 pkg prefix adaptive_assembly_bringup 2>/dev/null)"; then
  fail "adaptive_assembly_bringup is not discoverable; build and source the workspace"
fi

planning_files=(
  "${planning_prefix}/lib/adaptive_assembly_planning/panda_assembly_pose_adapter_node"
  "${planning_prefix}/lib/adaptive_assembly_planning/assembly_sequence_planning_node"
  "${planning_prefix}/share/adaptive_assembly_planning/launch/panda_assembly_pose_adapter.launch.py"
  "${planning_prefix}/share/adaptive_assembly_planning/launch/assembly_sequence_planning.launch.py"
)
bringup_file="${bringup_prefix}/share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_sequence_planning_demo.launch.py"

for path in "${planning_files[@]}"; do
  [[ -e "${path}" ]] || fail "missing installed file: ${path}"
  pass "installed file exists: ${path}"
done

[[ -f "${bringup_file}" ]] || fail "missing installed launch file: ${bringup_file}"
pass "installed launch file exists: ${bringup_file}"

echo "PASS: assembly sequence planner executables and launch files are available"
