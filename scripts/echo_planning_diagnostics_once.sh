#!/usr/bin/env bash

set -euo pipefail

echo "=== /pre_grasp_plan_success ==="
ros2 topic echo --once /pre_grasp_plan_success

echo "=== /pre_grasp_planning_status ==="
ros2 topic echo --once /pre_grasp_planning_status

echo "=== /pre_grasp_planning_duration_ms ==="
ros2 topic echo --once /pre_grasp_planning_duration_ms
