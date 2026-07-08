#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-30}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/joint_states}"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

diagnostics() {
  echo "Diagnostics:"
  echo "  ros2 topic list | grep joint_states"
  timeout 5s ros2 topic list 2>/dev/null |
    grep joint_states | sed 's/^/      /' || true
  echo '  ros2 node list | grep -E "controller|panda|gz|gazebo"'
  timeout 5s ros2 node list 2>/dev/null |
    grep -E 'controller|panda|gz|gazebo' | sed 's/^/      /' || true
}

command -v ros2 >/dev/null 2>&1 ||
  fail "ros2 is unavailable; source ROS 2 Jazzy and the workspace"

deadline=$((SECONDS + TIMEOUT_SEC))
joint_state=''
while ((SECONDS < deadline)); do
  if joint_state="$(timeout 5s ros2 topic echo "${JOINT_STATE_TOPIC}" --once 2>/dev/null)" &&
    grep -Eq '^[[:space:]]*-[[:space:]]*panda_finger_joint1([[:space:]]|$)' <<< "${joint_state}" &&
    grep -Eq '^[[:space:]]*-[[:space:]]*panda_finger_joint2([[:space:]]|$)' <<< "${joint_state}"; then
    echo "PASS: ${JOINT_STATE_TOPIC} contains panda_finger_joint1 and panda_finger_joint2"
    exit 0
  fi
  sleep 1
done

echo "FAIL: ${JOINT_STATE_TOPIC} did not contain Panda finger joints within ${TIMEOUT_SEC}s"
echo "Start the simulator-only Panda Gazebo launch first:"
echo "  ros2 launch adaptive_assembly_sim adaptive_assembly_panda_gazebo.launch.py"
if [[ -n "${joint_state}" ]]; then
  echo "Last ${JOINT_STATE_TOPIC} sample:"
  echo "${joint_state}" | sed 's/^/      /'
fi
diagnostics
exit 1
