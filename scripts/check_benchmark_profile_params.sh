#!/usr/bin/env bash

set -euo pipefail

config_dir="src/adaptive_assembly_bringup/config"

check_contains() {
  local file_path="$1"
  local pattern="$2"

  if [[ ! -f "${file_path}" ]]; then
    echo "FAIL: missing ${file_path}"
    exit 1
  fi

  if ! grep -q "${pattern}" "${file_path}"; then
    echo "FAIL: ${file_path} does not contain '${pattern}'"
    exit 1
  fi

  echo "PASS: ${file_path} contains '${pattern}'"
}

check_contains "${config_dir}/adaptive_assembly_benchmark_params.yaml" "random_seed: 42"
check_contains "${config_dir}/adaptive_assembly_benchmark_narrow_params.yaml" "random_seed: 101"
check_contains "${config_dir}/adaptive_assembly_benchmark_wide_params.yaml" "random_seed: 202"
check_contains "${config_dir}/adaptive_assembly_benchmark_fixed_yaw_params.yaml" "random_seed: 303"
check_contains "${config_dir}/adaptive_assembly_benchmark_fixed_yaw_params.yaml" "yaw_min: 0.0"
check_contains "${config_dir}/adaptive_assembly_benchmark_fixed_yaw_params.yaml" "yaw_max: 0.0"

echo "PASS: benchmark profile deterministic parameters are configured"
