#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 pkg executables adaptive_assembly_sim | grep -q \
  'gazebo_entity_pose_observer_node'
ros2 launch adaptive_assembly_sim gazebo_entity_pose_observer.launch.py \
  --show-args >/dev/null

echo 'PASS: Gazebo entity pose observer executable and launch file are available'
