#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 is unavailable; source ROS 2 Jazzy and the workspace" >&2
  exit 1
fi

if ! output="$(timeout 15s ros2 control list_controllers 2>&1)"; then
  echo "FAIL: could not list controllers within 15 seconds" >&2
  echo "$output" >&2
  exit 1
fi

if ! grep -Eq '^panda_gripper_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' <<<"$output"; then
  echo "FAIL: panda_gripper_controller is not active" >&2
  echo "$output" >&2
  exit 1
fi

echo "PASS: panda_gripper_controller is active"
