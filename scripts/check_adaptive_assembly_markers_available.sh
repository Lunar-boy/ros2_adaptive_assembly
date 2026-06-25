#!/usr/bin/env bash
set -euo pipefail

PACKAGE="adaptive_assembly_planning"
EXECUTABLE="adaptive_assembly_marker_node"
LAUNCH_FILE="adaptive_assembly_markers.launch.py"

fail() {
  echo "FAIL: $1"
  exit 1
}

pass() {
  echo "PASS: $1"
}

if ! PREFIX="$(ros2 pkg prefix "${PACKAGE}" 2>/dev/null)"; then
  fail "ROS2 package '${PACKAGE}' is not discoverable. Build and source the workspace first."
fi
pass "Found package '${PACKAGE}' at ${PREFIX}"

LAUNCH_PATH="${PREFIX}/share/${PACKAGE}/launch/${LAUNCH_FILE}"
EXECUTABLE_PATH="${PREFIX}/lib/${PACKAGE}/${EXECUTABLE}"

[[ -f "${LAUNCH_PATH}" ]] || fail "Missing installed launch file: ${LAUNCH_PATH}"
pass "Installed launch file exists: ${LAUNCH_PATH}"

[[ -x "${EXECUTABLE_PATH}" ]] || fail "Missing executable: ${EXECUTABLE_PATH}"
pass "Installed executable exists: ${EXECUTABLE_PATH}"

echo "PASS: Adaptive assembly marker launch and executable are available."
