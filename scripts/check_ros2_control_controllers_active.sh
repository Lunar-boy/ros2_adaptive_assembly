#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-30}"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"

fail() {
  echo "FAIL: $*"
  exit 1
}

command -v ros2 >/dev/null 2>&1 ||
  fail "ros2 command is unavailable; source ROS 2 and the workspace"

deadline=$((SECONDS + TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if controller_output="$(
    ros2 control list_controllers \
      --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null
  )"; then
    if grep -Eq '^joint_state_broadcaster[[:space:]]+' <<< "${controller_output}" &&
      grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active[[:space:]]' <<< "${controller_output}" &&
      grep -Eq '^panda_arm_controller[[:space:]]+' <<< "${controller_output}" &&
      grep -Eq '^panda_arm_controller[[:space:]].*[[:space:]]active[[:space:]]' <<< "${controller_output}"; then
      echo "PASS: joint_state_broadcaster and panda_arm_controller are active"
      exit 0
    fi
  fi
  sleep 1
done

echo "FAIL: expected controllers were not active within ${TIMEOUT_SEC}s"
ros2 control list_controllers \
  --controller-manager "${CONTROLLER_MANAGER}" 2>/dev/null |
  sed 's/^/      /' || true
exit 1
