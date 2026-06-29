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

check_topic_type "/panda_assembly_pose" "geometry_msgs/msg/PoseStamped"
check_topic_type "/panda_assembly_pose_adapter_status" "std_msgs/msg/String"
check_topic_type "/assembly_sequence_plan_success" "std_msgs/msg/Bool"
check_topic_type "/assembly_sequence_planning_status" "std_msgs/msg/String"
check_topic_type "/assembly_sequence_planning_duration_ms" "std_msgs/msg/Float64"

echo "PASS: all Panda assembly sequence topics are available"
