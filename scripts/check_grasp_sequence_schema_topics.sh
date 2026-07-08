#!/usr/bin/env bash
set -euo pipefail

cd "${HOME}/ros2_adaptive_assembly_ws"
if [[ ! -f install/setup.bash ]]; then
  echo "FAIL: install/setup.bash is missing. Run colcon build --symlink-install first."
  exit 1
fi
set +u
source install/setup.bash
set -u

expected=(
  /grasp_candidates /selected_grasp_pose /pre_grasp_pose /grasp_pose
  /lift_pose /object_place_pose /pre_place_pose /place_pose /retreat_pose
  /grasp_sequence_status
)
available="$(ros2 topic list)"
for topic in "${expected[@]}"; do
  if ! grep -Fxq "${topic}" <<<"${available}"; then
    echo "FAIL: expected topic is unavailable: ${topic}"
    exit 1
  fi
done
echo "PASS: explicit grasp sequence schema topics are available"
