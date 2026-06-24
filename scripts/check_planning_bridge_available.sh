#!/usr/bin/env bash

set -euo pipefail

check_package() {
  local package_name="$1"
  local prefix

  if prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
    echo "PASS: ${package_name} is available at ${prefix}"
    return 0
  fi

  echo "FAIL: ${package_name} is not discoverable"
  if [[ "${package_name}" == "moveit_resources_panda_moveit_config" ]]; then
    echo "      Install MoveIt2 and Panda demo resources with:"
    echo "      sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config"
  fi
  return 1
}

main() {
  local failures=0

  check_package "adaptive_assembly_planning" || failures=$((failures + 1))
  check_package "adaptive_assembly_bringup" || failures=$((failures + 1))
  check_package "adaptive_assembly_perception" || failures=$((failures + 1))
  check_package "adaptive_assembly_task" || failures=$((failures + 1))
  check_package "moveit_resources_panda_moveit_config" || failures=$((failures + 1))

  if ((failures > 0)); then
    echo "FAIL: ${failures} required package(s) are missing"
    exit 1
  fi

  echo "PASS: all planning bridge packages are available"
}

main "$@"
