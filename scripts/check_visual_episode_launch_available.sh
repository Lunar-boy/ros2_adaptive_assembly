#!/usr/bin/env bash
set -euo pipefail

prefix="$(ros2 pkg prefix adaptive_assembly_bringup)"
launch_file="${prefix}/share/adaptive_assembly_bringup/launch/adaptive_assembly_full_episode_visual_demo.launch.py"
test -f "${launch_file}"
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_episode_visual_demo.launch.py --show-args >/dev/null
echo "PASS: visual episode launch is installed and loadable"
