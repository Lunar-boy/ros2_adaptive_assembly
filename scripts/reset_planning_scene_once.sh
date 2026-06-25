#!/usr/bin/env bash

set -euo pipefail

trigger_type="std_srvs/srv/Trigger"

check_service() {
  local service="$1"
  local actual_type

  if ! actual_type="$(ros2 service type "${service}" 2>/dev/null)"; then
    echo "FAIL: ${service} does not exist"
    exit 1
  fi

  if [[ "${actual_type}" != "${trigger_type}" ]]; then
    echo "FAIL: ${service} has type '${actual_type}', expected '${trigger_type}'"
    exit 1
  fi

  echo "PASS: ${service} exists with type ${trigger_type}"
}

call_trigger_service() {
  local service="$1"
  local response

  echo "Calling ${service}..."
  if ! response="$(ros2 service call "${service}" "${trigger_type}" "{}")"; then
    echo "FAIL: ${service} call failed"
    exit 1
  fi

  echo "${response}"
  if ! grep -Eiq "success[:=][[:space:]]*true" <<< "${response}"; then
    echo "FAIL: ${service} did not return success=true"
    exit 1
  fi

  echo "PASS: ${service} returned success=true"
}

services=(
  "/clear_dynamic_target_scene"
  "/clear_static_planning_scene"
  "/reapply_static_planning_scene"
)

for service in "${services[@]}"; do
  check_service "${service}"
done

call_trigger_service "/clear_dynamic_target_scene"
call_trigger_service "/clear_static_planning_scene"
call_trigger_service "/reapply_static_planning_scene"

bash scripts/check_static_planning_scene_ready.sh

echo "PASS: PlanningScene reset workflow completed"
