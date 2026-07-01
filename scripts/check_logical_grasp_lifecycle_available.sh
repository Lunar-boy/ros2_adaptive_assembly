#!/usr/bin/env bash
set -euo pipefail

manipulation_share="$(ros2 pkg prefix adaptive_assembly_manipulation)/share/adaptive_assembly_manipulation"
bringup_share="$(ros2 pkg prefix adaptive_assembly_bringup)/share/adaptive_assembly_bringup"

test -f "${manipulation_share}/launch/logical_grasp_lifecycle.launch.py"
test -f "${bringup_share}/launch/adaptive_assembly_logical_grasp_demo.launch.py"
ros2 pkg executables adaptive_assembly_manipulation | grep -q \
  'logical_grasp_lifecycle_node'

echo 'PASS: logical grasp package, node, and launch files are available'
