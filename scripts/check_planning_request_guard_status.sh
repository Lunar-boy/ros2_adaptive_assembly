#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $*"
  exit 1
}

pass() {
  echo "PASS: $*"
}

status="$(ros2 topic echo --once --full-length /pre_grasp_planning_status 2>/dev/null)" ||
  fail "could not read one /pre_grasp_planning_status message"

for field in guard_enabled= guard_passed= guard_reason=; do
  if grep -q "${field}" <<< "${status}"; then
    pass "/pre_grasp_planning_status contains ${field}"
  else
    echo "${status}"
    fail "/pre_grasp_planning_status is missing ${field}"
  fi
done

pass "planning request guard metadata is present"
