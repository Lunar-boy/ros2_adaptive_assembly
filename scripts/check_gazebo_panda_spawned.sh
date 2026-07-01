#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${TIMEOUT_SEC:-30}"
ENTITY_NAME="${ENTITY_NAME:-panda}"

fail() {
  echo "FAIL: $*"
  exit 1
}

command -v gz >/dev/null 2>&1 ||
  fail "the Gazebo 'gz' executable is unavailable"

deadline=$((SECONDS + TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if model_output="$(gz model --list 2>/dev/null)" &&
    sed -E 's/^[[:space:]]*-[[:space:]]*//' <<< "${model_output}" |
      grep -Fxq "${ENTITY_NAME}"; then
    echo "PASS: Gazebo model '${ENTITY_NAME}' is spawned"
    exit 0
  fi
  sleep 1
done

echo "FAIL: Gazebo model '${ENTITY_NAME}' was not observed within ${TIMEOUT_SEC}s"
if model_output="$(gz model --list 2>/dev/null)"; then
  echo "${model_output}" | sed 's/^/      /'
fi
exit 1
