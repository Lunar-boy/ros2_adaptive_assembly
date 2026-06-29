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

launch_name="adaptive_assembly_panda_sequence_planning_fixed_start.launch.py"
installed_launch="${prefix}/share/adaptive_assembly_bringup/launch/${launch_name}"
source_launch="src/adaptive_assembly_bringup/launch/${launch_name}"

[[ -f "${installed_launch}" ]] || fail "missing installed launch file: ${installed_launch}"
pass "installed fixed-start launch exists: ${installed_launch}"

[[ -f "${source_launch}" ]] || fail "missing source launch file: ${source_launch}"
grep -q "'start_state_mode': 'fixed'" "${source_launch}" || \
  fail "fixed-start launch does not pass start_state_mode=fixed"
pass "fixed-start launch passes start_state_mode=fixed"

echo "PASS: fixed-start assembly sequence launch is available"
