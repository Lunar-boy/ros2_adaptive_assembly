#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="adaptive_assembly_planning"
SOURCE_LAUNCH="src/${PACKAGE_NAME}/launch/panda_pre_grasp_pose_adapter.launch.py"

fail() {
  echo "FAIL: $*"
  exit 1
}

pass() {
  echo "PASS: $*"
}

prefix="$(ros2 pkg prefix "${PACKAGE_NAME}")" || fail "${PACKAGE_NAME} is not discoverable"
installed_launch="${prefix}/share/${PACKAGE_NAME}/launch/panda_pre_grasp_pose_adapter.launch.py"

[[ -f "${installed_launch}" ]] || fail "installed launch file not found: ${installed_launch}"
pass "installed launch file exists: ${installed_launch}"

[[ -f "${SOURCE_LAUNCH}" ]] || fail "source launch file not found: ${SOURCE_LAUNCH}"

for key in use_tf_transform target_frame_id tf_lookup_timeout_sec status_topic; do
  if grep -q "${key}" "${SOURCE_LAUNCH}"; then
    pass "source launch contains ${key}"
  else
    fail "source launch is missing ${key}"
  fi
done

pass "Panda pose adapter TF2 parameters are present"
