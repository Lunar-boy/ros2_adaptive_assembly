#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 pkg executables adaptive_assembly_perception | grep -q 'aruco_detector_node'
ros2 launch adaptive_assembly_perception \
  opencv_aruco_perception.launch.py --show-args >/dev/null
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_opencv_aruco_demo.launch.py --show-args >/dev/null

echo 'PASS: optional OpenCV ArUco executable and launch files are available'
