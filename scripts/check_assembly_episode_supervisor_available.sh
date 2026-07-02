#!/usr/bin/env bash
set -euo pipefail

if ! ros2 pkg executables adaptive_assembly_episode | \
  grep -q 'adaptive_assembly_episode assembly_episode_supervisor_node'; then
  echo 'FAIL: assembly_episode_supervisor_node is not installed'
  exit 1
fi

if ! ros2 launch adaptive_assembly_episode \
  assembly_episode_supervisor.launch.py --show-args >/dev/null; then
  echo 'FAIL: assembly episode supervisor launch is unavailable'
  exit 1
fi

echo 'PASS: assembly episode supervisor executable and launch are available'
