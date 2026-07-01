#!/usr/bin/env bash
set -euo pipefail

share="$(ros2 pkg prefix adaptive_assembly_bringup)/share/adaptive_assembly_bringup"
test -f "${share}/launch/adaptive_assembly_ros2_control_success_demo.launch.py"
test -f "${share}/launch/adaptive_assembly_gazebo_ros2_control_success_demo.launch.py"
ros2 pkg executables adaptive_assembly_execution | grep -q \
  'simulated_follow_joint_trajectory_server_node'
echo 'PASS: ros2_control success demo launch files and action server are available'
