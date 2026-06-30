#!/usr/bin/env bash
set -euo pipefail

check_topic_type() {
  local topic_name="$1"
  local expected_type="$2"
  local topic_info

  if ! topic_info="$(ros2 topic info "${topic_name}" 2>/dev/null)"; then
    echo "FAIL: ${topic_name} does not exist or ROS 2 is not available"
    exit 1
  fi
  if ! grep -q "^Type: ${expected_type}$" <<< "${topic_info}"; then
    echo "FAIL: ${topic_name} has unexpected type; expected ${expected_type}"
    echo "${topic_info}" | sed 's/^/      /'
    exit 1
  fi
  echo "PASS: ${topic_name} exists with type ${expected_type}"
}

check_topic_type "/assembly_sequence_planning_status" "std_msgs/msg/String"
check_topic_type "/assembly_execution_status" "std_msgs/msg/String"
check_topic_type "/dynamic_target_scene_status" "std_msgs/msg/String"
check_topic_type "/planning_scene_audit_status" "std_msgs/msg/String"
check_topic_type "/assembly_recovery_status" "std_msgs/msg/String"
check_topic_type "/assembly_recovery_action" "std_msgs/msg/String"
check_topic_type "/assembly_recovery_success" "std_msgs/msg/Bool"

echo "PASS: all recovery supervisor topics are available"
