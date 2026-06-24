#!/usr/bin/env bash

set -euo pipefail

topic_name="/panda_pre_grasp_pose"
expected_type="geometry_msgs/msg/PoseStamped"

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

echo "PASS: ${topic_name} exists with type ${expected_type}"
