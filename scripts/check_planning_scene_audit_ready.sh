#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $*"
  exit 1
}

pass() {
  echo "PASS: $*"
}

topic="/planning_scene_audit_ready"
expected_type="std_msgs/msg/Bool"

topic_info="$(ros2 topic info "${topic}" 2>/dev/null)" ||
  fail "${topic} does not exist"

if grep -q "Type: ${expected_type}" <<< "${topic_info}"; then
  pass "${topic} exists with type ${expected_type}"
else
  echo "${topic_info}"
  fail "${topic} does not have type ${expected_type}"
fi

message="$(ros2 topic echo --once "${topic}" 2>/dev/null)" ||
  fail "could not read one ${topic} message"

if grep -q "data: true" <<< "${message}"; then
  pass "${topic} reports data: true"
else
  echo "${message}"
  fail "${topic} did not report data: true"
fi
