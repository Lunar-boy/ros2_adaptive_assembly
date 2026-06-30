#!/usr/bin/env bash

set -euo pipefail

fail() {
  echo "FAIL: $*"
  exit 1
}

package_prefix="$(ros2 pkg prefix adaptive_assembly_sim 2>/dev/null)" ||
  fail "adaptive_assembly_sim is not discoverable; build and source the workspace"
package_share="${package_prefix}/share/adaptive_assembly_sim"
world_file="${package_share}/worlds/adaptive_assembly_workcell.sdf"
launch_file="${package_share}/launch/adaptive_assembly_workcell.launch.py"

[[ -f "${package_share}/package.xml" ]] || fail "installed package.xml is missing"
[[ -f "${world_file}" ]] || fail "installed workcell world is missing"
[[ -f "${launch_file}" ]] || fail "installed workcell launch file is missing"

for model_name in work_table target_support target_object assembly_socket_fixture; do
  grep -q "<model name=\"${model_name}\">" "${world_file}" ||
    fail "world is missing model '${model_name}'"
done

echo "PASS: adaptive_assembly_sim is discoverable at ${package_prefix}"
echo "PASS: installed workcell world and launch assets are present"
echo "PASS: required primitive workcell models are declared"
