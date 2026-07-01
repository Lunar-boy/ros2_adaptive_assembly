#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-30}"
ACTION_NAME="${ACTION_NAME:-/panda_arm_controller/follow_joint_trajectory}"

fail() {
  echo "FAIL: $*"
  exit 1
}

wait_for_topic_type() {
  local topic_name="$1"
  local expected_type="$2"
  local deadline=$((SECONDS + TIMEOUT_SEC))
  local topic_info

  while (( SECONDS < deadline )); do
    if topic_info="$(ros2 topic info "${topic_name}" 2>/dev/null)" &&
      grep -q "^Type: ${expected_type}$" <<< "${topic_info}"; then
      echo "PASS: ${topic_name} exists with type ${expected_type}"
      return 0
    fi
    sleep 1
  done
  fail "${topic_name} with type ${expected_type} was not observed"
}

wait_for_action() {
  local deadline=$((SECONDS + TIMEOUT_SEC))
  local action_output

  while (( SECONDS < deadline )); do
    if action_output="$(ros2 action list -t 2>/dev/null)" &&
      grep -Fq "${ACTION_NAME} [control_msgs/action/FollowJointTrajectory]" \
        <<< "${action_output}"; then
      echo "PASS: ${ACTION_NAME} action is available"
      return 0
    fi
    sleep 1
  done
  fail "${ACTION_NAME} FollowJointTrajectory action was not observed"
}

command -v ros2 >/dev/null 2>&1 ||
  fail "ros2 command is unavailable; source ROS 2 and the workspace"

wait_for_topic_type "/joint_states" "sensor_msgs/msg/JointState"
wait_for_topic_type "/pre_grasp_trajectory" "moveit_msgs/msg/RobotTrajectory"
wait_for_topic_type "/assembly_trajectory" "moveit_msgs/msg/RobotTrajectory"
wait_for_topic_type \
  "/assembly_ros2_control_execution_status" "std_msgs/msg/String"
wait_for_topic_type \
  "/assembly_ros2_control_execution_success" "std_msgs/msg/Bool"
wait_for_topic_type \
  "/assembly_ros2_control_execution_duration_ms" "std_msgs/msg/Float64"
wait_for_topic_type \
  "/assembly_ros2_control_execution_stage_status" "std_msgs/msg/String"
wait_for_action

echo "PASS: full Gazebo execution topics and controller action are available"
