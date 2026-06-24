#!/usr/bin/env bash

set -euo pipefail

echo "===== /target_pose ====="
ros2 topic echo --once /target_pose

echo "===== /pre_grasp_pose ====="
ros2 topic echo --once /pre_grasp_pose

echo "===== /assembly_pose ====="
ros2 topic echo --once /assembly_pose
