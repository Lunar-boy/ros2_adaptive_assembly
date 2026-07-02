#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 pkg executables adaptive_assembly_perception | grep -q \
  'simulated_marker_pose_node'
ros2 launch adaptive_assembly_perception \
  simulated_vision_perception.launch.py --show-args >/dev/null
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_simulated_vision_demo.launch.py --show-args >/dev/null

echo 'PASS: simulated vision executable and launch files are available'
