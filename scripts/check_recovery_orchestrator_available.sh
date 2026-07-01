#!/usr/bin/env bash
set -euo pipefail

if ! ros2 pkg executables adaptive_assembly_recovery | \
  grep -q 'adaptive_assembly_recovery recovery_orchestrator_node'; then
  echo 'FAIL: recovery_orchestrator_node is not installed'
  exit 1
fi

if ! ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_recovery_orchestration_demo.launch.py --show-args \
  >/dev/null; then
  echo 'FAIL: recovery orchestration bringup launch is unavailable'
  exit 1
fi

echo 'PASS: recovery orchestrator executable and bringup launch are available'
