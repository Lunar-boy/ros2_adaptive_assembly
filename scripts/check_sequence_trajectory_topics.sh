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

check_topic_type "/pre_grasp_trajectory" "moveit_msgs/msg/RobotTrajectory"
check_topic_type "/assembly_trajectory" "moveit_msgs/msg/RobotTrajectory"
check_topic_type "/assembly_sequence_trajectory_status" "std_msgs/msg/String"

echo "PASS: all assembly sequence trajectory export topics are available"
