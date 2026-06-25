#!/usr/bin/env bash

set -euo pipefail

service="/clear_dynamic_target_scene"
expected_type="std_srvs/srv/Trigger"

if ! actual_type="$(ros2 service type "${service}" 2>/dev/null)"; then
  echo "FAIL: ${service} does not exist"
  exit 1
fi

if [[ "${actual_type}" != "${expected_type}" ]]; then
  echo "FAIL: ${service} has type '${actual_type}', expected '${expected_type}'"
  exit 1
fi

echo "PASS: ${service} exists with type ${expected_type}"
