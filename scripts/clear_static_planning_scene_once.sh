#!/usr/bin/env bash

set -euo pipefail

service="/clear_static_planning_scene"
service_type="std_srvs/srv/Trigger"
timeout_sec=10

echo "Waiting for ${service}..."
for _ in $(seq 1 "${timeout_sec}"); do
  if actual_type="$(ros2 service type "${service}" 2>/dev/null)" && [[ "${actual_type}" == "${service_type}" ]]; then
    response="$(ros2 service call "${service}" "${service_type}" "{}")"
    echo "${response}"

    if grep -Eiq "success[:=][[:space:]]*true" <<< "${response}"; then
      echo "PASS: ${service} returned success=true"
      exit 0
    fi

    echo "FAIL: ${service} did not return success=true"
    exit 1
  fi
  sleep 1
done

echo "FAIL: ${service} was not available within ${timeout_sec}s"
exit 1
