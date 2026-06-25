#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="adaptive_assembly_planning"

fail() {
  echo "FAIL: $*"
  exit 1
}

pass() {
  echo "PASS: $*"
}

prefix="$(ros2 pkg prefix "${PACKAGE_NAME}")" || fail "${PACKAGE_NAME} is not discoverable"

launch_file="${prefix}/share/${PACKAGE_NAME}/launch/planning_scene_audit.launch.py"
executable="${prefix}/lib/${PACKAGE_NAME}/planning_scene_audit_node"

[[ -f "${launch_file}" ]] || fail "installed launch file not found: ${launch_file}"
pass "installed launch file exists: ${launch_file}"

[[ -x "${executable}" ]] || fail "installed executable not found or not executable: ${executable}"
pass "installed executable exists: ${executable}"

pass "PlanningScene audit node is available"
