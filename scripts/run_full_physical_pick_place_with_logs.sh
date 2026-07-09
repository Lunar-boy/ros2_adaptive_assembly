#!/usr/bin/env bash
set -euo pipefail

RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-runs/$RUN_ID}
LAUNCH_PACKAGE=adaptive_assembly_bringup
LAUNCH_FILE=adaptive_assembly_full_physical_pick_place_demo.launch.py
LOG_PATH="$RUN_DIR/launch.log"
METADATA_PATH="$RUN_DIR/metadata.env"
START_TIMESTAMP=$(date -Iseconds)

mkdir -p "$RUN_DIR"

{
  printf 'RUN_ID=%q\n' "$RUN_ID"
  printf 'RUN_DIR=%q\n' "$RUN_DIR"
  printf 'LAUNCH_PACKAGE=%q\n' "$LAUNCH_PACKAGE"
  printf 'LAUNCH_FILE=%q\n' "$LAUNCH_FILE"
  printf 'CWD=%q\n' "$(pwd)"
  printf 'START_TIMESTAMP=%q\n' "$START_TIMESTAMP"
  printf 'LAUNCH_ARGS=%q\n' "$*"
} > "$METADATA_PATH"

echo "Starting full physical pick-place run."
echo "Run directory: $RUN_DIR"
echo "Launch log: $LOG_PATH"

set +e
ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE" "$@" 2>&1 | tee "$LOG_PATH"
launch_status=${PIPESTATUS[0]}
set -e

if [[ "$launch_status" -eq 0 ]]; then
  echo "Launch exited successfully. Log saved to $LOG_PATH"
else
  echo "Launch failed with exit code $launch_status. Log saved to $LOG_PATH"
fi

exit "$launch_status"
