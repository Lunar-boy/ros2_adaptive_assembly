#!/usr/bin/env bash

set -euo pipefail

topic="/pre_grasp_planning_status"
required_fields=(
  "planner_id="
  "num_planning_attempts="
  "max_velocity_scaling_factor="
  "max_acceleration_scaling_factor="
)

if ! message="$(ros2 topic echo --once --full-length "${topic}" 2>/dev/null)"; then
  echo "FAIL: could not read one message from ${topic}"
  echo "Start the Panda planning demo first."
  exit 1
fi

for field in "${required_fields[@]}"; do
  if ! grep -q "${field}" <<< "${message}"; then
    echo "FAIL: ${topic} message is missing '${field}'"
    echo "${message}" | sed 's/^/      /'
    exit 1
  fi
done

echo "PASS: ${topic} includes planner parameter fields"
