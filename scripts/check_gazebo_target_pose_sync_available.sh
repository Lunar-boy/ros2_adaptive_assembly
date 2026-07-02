#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 pkg executables adaptive_assembly_sim | grep -q \
  'gazebo_target_pose_sync_node'
ros2 launch adaptive_assembly_sim gazebo_target_pose_sync.launch.py \
  --show-args >/dev/null
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_target_sync_demo.launch.py --show-args >/dev/null

echo 'PASS: Gazebo target pose sync executable and launch files are available'
