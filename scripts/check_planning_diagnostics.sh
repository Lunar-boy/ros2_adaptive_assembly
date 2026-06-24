#!/usr/bin/env bash

set -euo pipefail

check_topic_type() {
  local topic_name="$1"
  local expected_type="$2"

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
}

check_topic_type "/pre_grasp_plan_success" "std_msgs/msg/Bool"
check_topic_type "/pre_grasp_planning_status" "std_msgs/msg/String"
check_topic_type "/pre_grasp_planning_duration_ms" "std_msgs/msg/Float64"

echo "PASS: all planning diagnostics topics are available"
