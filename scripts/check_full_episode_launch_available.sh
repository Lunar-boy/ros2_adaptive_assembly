#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

package_prefix="$(ros2 pkg prefix adaptive_assembly_bringup)"
launch_file="${package_prefix}/share/adaptive_assembly_bringup/launch/adaptive_assembly_full_episode_demo.launch.py"

if [[ ! -f "${launch_file}" ]]; then
  echo "FAIL: full episode launch file is not installed"
  exit 1
fi

ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_episode_demo.launch.py --show-args >/dev/null

echo "PASS: full episode launch is installed and discoverable"
