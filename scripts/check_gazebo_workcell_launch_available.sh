#!/usr/bin/env bash

set -euo pipefail

fail() {
  echo "FAIL: $*"
  exit 1
}

for package_name in adaptive_assembly_sim adaptive_assembly_bringup ros_gz_sim; do
  if ! prefix="$(ros2 pkg prefix "${package_name}" 2>/dev/null)"; then
    if [[ "${package_name}" == "ros_gz_sim" ]]; then
      fail "ros_gz_sim is unavailable; install ros-jazzy-ros-gz-sim"
    fi
    fail "${package_name} is not discoverable; build and source the workspace"
  fi
  echo "PASS: ${package_name} is available at ${prefix}"
done

command -v gz >/dev/null 2>&1 ||
  fail "the Gazebo 'gz' executable is unavailable"

ros2 launch adaptive_assembly_sim \
  adaptive_assembly_workcell.launch.py --show-args >/dev/null ||
  fail "adaptive_assembly_sim workcell launch is not loadable"
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_workcell_demo.launch.py --show-args >/dev/null ||
  fail "bringup workcell wrapper launch is not loadable"

echo "PASS: Gazebo workcell launch files are installed and loadable"
