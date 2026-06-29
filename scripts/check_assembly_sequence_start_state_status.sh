#!/usr/bin/env bash
set -euo pipefail

if ! status_message="$(ros2 topic echo --once /assembly_sequence_planning_status 2>/dev/null)"; then
  echo "FAIL: could not read /assembly_sequence_planning_status"
  echo "Start the Panda assembly sequence demo before running this check."
  exit 1
fi

if ! grep -q "start_state_mode=" <<< "${status_message}"; then
  echo "FAIL: assembly sequence status does not contain start_state_mode"
  echo "${status_message}" | sed 's/^/      /'
  exit 1
fi

if ! grep -q "start_state_mode=\(current\|fixed\)" <<< "${status_message}"; then
  echo "FAIL: assembly sequence status has an invalid start_state_mode"
  echo "${status_message}" | sed 's/^/      /'
  exit 1
fi

echo "PASS: assembly sequence status contains a valid start_state_mode"
echo "${status_message}" | sed 's/^/      /'
