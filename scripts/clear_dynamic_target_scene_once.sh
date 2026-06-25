#!/usr/bin/env bash

set -euo pipefail

service="/clear_dynamic_target_scene"
service_type="std_srvs/srv/Trigger"

echo "Waiting for ${service}..."
if ! timeout 10 bash -c "until ros2 service type '${service}' >/dev/null 2>&1; do sleep 0.5; done"; then
  echo "FAIL: timed out waiting for ${service}"
  exit 1
fi

response="$(ros2 service call "${service}" "${service_type}" "{}")"
echo "${response}"

if grep -Eiq "success[:=][[:space:]]*true" <<< "${response}"; then
  echo "PASS: dynamic target scene clear service succeeded"
  exit 0
fi

echo "FAIL: dynamic target scene clear service did not report success"
exit 1
