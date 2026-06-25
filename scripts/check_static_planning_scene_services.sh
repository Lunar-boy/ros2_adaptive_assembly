#!/usr/bin/env bash

set -euo pipefail

expected_type="std_srvs/srv/Trigger"
services=(
  "/clear_static_planning_scene"
  "/reapply_static_planning_scene"
)

for service in "${services[@]}"; do
  if ! actual_type="$(ros2 service type "${service}" 2>/dev/null)"; then
    echo "FAIL: ${service} does not exist"
    exit 1
  fi

  if [[ "${actual_type}" != "${expected_type}" ]]; then
    echo "FAIL: ${service} has type '${actual_type}', expected '${expected_type}'"
    exit 1
  fi

  echo "PASS: ${service} exists with type ${expected_type}"
done
