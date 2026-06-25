#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="adaptive_assembly_bringup"
LAUNCH_FILE="adaptive_assembly_panda_planning_benchmark_guarded.launch.py"
SOURCE_LAUNCH="src/${PACKAGE_NAME}/launch/${LAUNCH_FILE}"

fail() {
  echo "FAIL: $*"
  exit 1
}

pass() {
  echo "PASS: $*"
}

prefix="$(ros2 pkg prefix "${PACKAGE_NAME}")" || fail "${PACKAGE_NAME} is not discoverable"
installed_launch="${prefix}/share/${PACKAGE_NAME}/launch/${LAUNCH_FILE}"

[[ -f "${installed_launch}" ]] || fail "installed launch file not found: ${installed_launch}"
pass "installed guarded benchmark launch exists: ${installed_launch}"

[[ -f "${SOURCE_LAUNCH}" ]] || fail "source launch file not found: ${SOURCE_LAUNCH}"

for key in \
  enable_request_guard \
  required_frame_id \
  workspace_min_x \
  workspace_max_x \
  workspace_min_y \
  workspace_max_y \
  workspace_min_z \
  workspace_max_z; do
  if grep -q "${key}" "${SOURCE_LAUNCH}"; then
    pass "source launch contains ${key}"
  else
    fail "source launch is missing ${key}"
  fi
done

pass "guarded benchmark launch contains request guard settings"
