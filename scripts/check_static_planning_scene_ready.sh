#!/usr/bin/env bash

set -euo pipefail

topic_name="/planning_scene_objects_ready"
expected_type="std_msgs/msg/Bool"

if ! topic_info="$(ros2 topic info "${topic_name}" 2>/dev/null)"; then
  echo "FAIL: ${topic_name} does not exist or ROS 2 is not available"
  exit 1
fi

if ! grep -q "^Type: ${expected_type}$" <<< "${topic_info}"; then
  echo "FAIL: ${topic_name} has unexpected type"
  echo "      expected: ${expected_type}"
  echo "${topic_info}" | sed 's/^/      /'
  exit 1
fi

if ! message="$(ros2 topic echo --once "${topic_name}" 2>/dev/null)"; then
  echo "FAIL: could not read one message from ${topic_name}"
  exit 1
fi

if ! grep -q "data: true" <<< "${message}"; then
  echo "FAIL: ${topic_name} did not report data: true"
  echo "${message}" | sed 's/^/      /'
  exit 1
fi

echo "PASS: ${topic_name} exists with type ${expected_type} and reports data: true"
