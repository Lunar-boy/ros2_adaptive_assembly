#!/usr/bin/env bash
set -euo pipefail

manipulation_share="$(ros2 pkg prefix adaptive_assembly_manipulation)/share/adaptive_assembly_manipulation"
bringup_share="$(ros2 pkg prefix adaptive_assembly_bringup)/share/adaptive_assembly_bringup"

test -x "$(ros2 pkg prefix adaptive_assembly_manipulation)/lib/adaptive_assembly_manipulation/gazebo_attach_detach_node"
test -f "${manipulation_share}/launch/gazebo_attach_detach.launch.py"
test -f "${bringup_share}/launch/adaptive_assembly_gazebo_grasp_attach_demo.launch.py"
ros2 launch adaptive_assembly_manipulation gazebo_attach_detach.launch.py --show-args >/dev/null
ros2 launch adaptive_assembly_bringup adaptive_assembly_gazebo_grasp_attach_demo.launch.py --show-args >/dev/null
echo "PASS: Gazebo attach/detach executable and launch files are available"
