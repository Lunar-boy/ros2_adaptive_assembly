#!/usr/bin/env bash

set -euo pipefail

topic="/dynamic_target_scene_ready"
expected_type="std_msgs/msg/Bool"

if ! ros2 topic list | grep -qx "${topic}"; then
  echo "FAIL: ${topic} does not exist"
  exit 1
fi

actual_type="$(ros2 topic info "${topic}" | awk -F': ' '/^Type:/ {print $2}')"
if [[ "${actual_type}" != "${expected_type}" ]]; then
  echo "FAIL: ${topic} has type '${actual_type}', expected '${expected_type}'"
  exit 1
fi

message="$(ros2 topic echo --once "${topic}")"
if ! grep -q "data: true" <<< "${message}"; then
  echo "FAIL: ${topic} did not report data: true"
  echo "${message}"
  exit 1
fi

echo "PASS: ${topic} exists with type ${expected_type} and reports data: true"
