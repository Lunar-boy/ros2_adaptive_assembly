#!/usr/bin/env bash
set -euo pipefail
WS=/workspaces/ros2_adaptive_assembly_ws/

cd "${WS}"
set +u  # 临时关闭“未定义变量报错”检查
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u  # 重新开启检查（如果脚本后续还需要的话）
LAUNCH_WAIT_SEC="${LAUNCH_WAIT_SEC:-12}"
VALIDATION_TIMEOUT_SEC="${VALIDATION_TIMEOUT_SEC:-90}"

ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_panda_sequence_planning_reachable.launch.py &
launch_pid=$!

cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep "${LAUNCH_WAIT_SEC}"

timeout "${VALIDATION_TIMEOUT_SEC}" \
  python3 src/ros2_adaptive_assembly/scripts/check_assembly_sequence_success_path.py
