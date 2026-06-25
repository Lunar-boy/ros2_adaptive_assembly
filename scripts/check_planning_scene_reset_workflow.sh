#!/usr/bin/env bash

set -euo pipefail

required_scripts=(
  "scripts/reset_planning_scene_once.sh"
  "scripts/check_static_planning_scene_ready.sh"
  "scripts/check_static_planning_scene_services.sh"
  "scripts/check_dynamic_target_clear_service.sh"
)

for script in "${required_scripts[@]}"; do
  if [[ ! -f "${script}" ]]; then
    echo "FAIL: ${script} does not exist"
    exit 1
  fi

  if [[ ! -x "${script}" ]]; then
    echo "FAIL: ${script} is not executable"
    exit 1
  fi

  echo "PASS: ${script} exists and is executable"
done

echo "PASS: PlanningScene reset workflow scripts are available"
