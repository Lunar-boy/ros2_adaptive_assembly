#!/usr/bin/env bash

set -euo pipefail

check_topic() {
  local topic_name="$1"
  local expected_type="$2"
  local topic_info

  if ! topic_info="$(ros2 topic info "${topic_name}" 2>/dev/null)"; then
    echo "FAIL: ${topic_name} does not exist or ROS 2 is not available"
    return 1
  fi

  if ! grep -q "^Type: ${expected_type}$" <<< "${topic_info}"; then
    echo "FAIL: ${topic_name} has unexpected type"
    echo "      expected: ${expected_type}"
    echo "${topic_info}" | sed 's/^/      /'
    return 1
  fi

  echo "PASS: ${topic_name} exists with type ${expected_type}"
}

main() {
  local failures=0

  check_topic "/target_pose" "geometry_msgs/msg/PoseStamped" || failures=$((failures + 1))
  check_topic "/pre_grasp_pose" "geometry_msgs/msg/PoseStamped" || failures=$((failures + 1))
  check_topic "/assembly_pose" "geometry_msgs/msg/PoseStamped" || failures=$((failures + 1))

  if ((failures > 0)); then
    echo "FAIL: ${failures} topic check(s) failed"
    exit 1
  fi

  echo "PASS: all pipeline topic checks passed"
}

main "$@"
