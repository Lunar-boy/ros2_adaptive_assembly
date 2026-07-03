#!/usr/bin/env bash
set -euo pipefail

topics=(
  /assembly_ros2_control_execution_status
  /assembly_ros2_control_execution_success
  /logical_grasp_lifecycle_status
  /gazebo_attach_detach_status
  /gazebo_target_sync_status
  /gazebo_target_object_pose
  /gazebo_target_object_pose_status
  /assembly_insertion_status
  /assembly_insertion_success
  /assembly_episode_status
  /assembly_episode_success
)

available_topics="$(ros2 topic list)"
failed=0
for topic in "${topics[@]}"; do
  if grep -Fxq "${topic}" <<< "${available_topics}"; then
    echo "PASS: ${topic} is available"
  else
    echo "FAIL: ${topic} is not available"
    failed=1
  fi
done

if (( failed != 0 )); then
  exit 1
fi
echo "PASS: all full episode topics are available"
