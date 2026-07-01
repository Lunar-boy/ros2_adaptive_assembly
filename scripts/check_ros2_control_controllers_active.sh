#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-30}"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"

fail() {
  echo "FAIL: $*"
  exit 1
}

diagnostics() {
  echo "Diagnostics:"
  echo "  ros2 service list | grep controller_manager"
  timeout 5s ros2 service list 2>/dev/null |
    grep controller_manager | sed 's/^/      /' || true
  echo '  ros2 node list | grep -E "controller|panda|gz"'
  timeout 5s ros2 node list 2>/dev/null |
    grep -E 'controller|panda|gz' | sed 's/^/      /' || true
  echo "  ros2 action list | grep follow_joint_trajectory"
  timeout 5s ros2 action list 2>/dev/null |
    grep follow_joint_trajectory | sed 's/^/      /' || true
}

command -v ros2 >/dev/null 2>&1 ||
  fail "ros2 command is unavailable; source ROS 2 and the workspace"

deadline=$((SECONDS + TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if controller_output="$(
    timeout 5s ros2 control list_controllers \
      --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null
  )"; then
    if grep -Eq '^joint_state_broadcaster[[:space:]]+' <<< "${controller_output}" &&
      grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "${controller_output}" &&
      grep -Eq '^panda_arm_controller[[:space:]]+' <<< "${controller_output}" &&
      grep -Eq '^panda_arm_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' <<< "${controller_output}"; then
      echo "PASS: joint_state_broadcaster and panda_arm_controller are active"
      exit 0
    fi
  fi
  sleep 1
done

echo "FAIL: expected controllers were not active within ${TIMEOUT_SEC}s"
timeout 5s ros2 control list_controllers \
  --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null |
  sed 's/^/      /' || true
echo "Configured controller manager: ${CONTROLLER_MANAGER}"
echo "Verify gz_ros2_control loaded its YAML and that the manager namespace matches."
diagnostics
exit 1
