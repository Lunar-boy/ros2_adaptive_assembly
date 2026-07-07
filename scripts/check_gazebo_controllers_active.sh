#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-60}"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

command -v ros2 >/dev/null 2>&1 ||
  fail "ros2 is unavailable; source ROS 2 Jazzy and the workspace"

deadline=$((SECONDS + TIMEOUT_SEC))
controller_output=''
while ((SECONDS < deadline)); do
  if controller_output="$(
    timeout 5s ros2 control list_controllers \
      -c "${CONTROLLER_MANAGER}" 2>/dev/null
  )" &&
    grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' \
      <<< "${controller_output}" &&
    grep -Eq '^panda_arm_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
      <<< "${controller_output}"; then
    break
  fi
  sleep 1
done

grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' \
  <<< "${controller_output}" ||
  fail "joint_state_broadcaster was not active within ${TIMEOUT_SEC}s"
grep -Eq '^panda_arm_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
  <<< "${controller_output}" ||
  fail "panda_arm_controller was not active within ${TIMEOUT_SEC}s"

echo "${controller_output}"

joint_state="$({ timeout 10s ros2 topic echo /joint_states --once; } 2>/dev/null)" ||
  fail "no /joint_states message received within 10s"
stamp_sec="$(awk '/^[[:space:]]*sec:/{print $2; exit}' <<< "${joint_state}")"
stamp_nanosec="$(awk '/^[[:space:]]*nanosec:/{print $2; exit}' <<< "${joint_state}")"
if [[ ! "${stamp_sec}" =~ ^[0-9]+$ || ! "${stamp_nanosec}" =~ ^[0-9]+$ ]]; then
  fail "/joint_states did not contain a valid timestamp"
fi
if ((stamp_sec == 0 && stamp_nanosec == 0)); then
  fail "/joint_states timestamp was zero"
fi

echo "PASS: both Panda controllers are active and /joint_states has a non-zero timestamp"
