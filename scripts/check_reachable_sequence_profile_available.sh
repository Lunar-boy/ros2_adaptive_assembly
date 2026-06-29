#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $1"
  exit 1
}

pass() {
  echo "PASS: $1"
}

if ! prefix="$(ros2 pkg prefix adaptive_assembly_bringup 2>/dev/null)"; then
  fail "adaptive_assembly_bringup is not discoverable; build and source the workspace"
fi

launch_path="${prefix}/share/adaptive_assembly_bringup/launch/adaptive_assembly_panda_sequence_planning_reachable.launch.py"
params_path="${prefix}/share/adaptive_assembly_bringup/config/adaptive_assembly_sequence_reachable_params.yaml"

[[ -f "${launch_path}" ]] || fail "missing installed launch file: ${launch_path}"
pass "installed reachable sequence launch exists: ${launch_path}"

[[ -f "${params_path}" ]] || fail "missing installed parameter file: ${params_path}"
pass "installed reachable sequence parameters exist: ${params_path}"

echo "PASS: known-reachable sequence profile is available"
