#!/usr/bin/env bash

set -euo pipefail

tmp_dir="$(mktemp -d /tmp/planning_report_export.XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

csv_a="${tmp_dir}/a.csv"
csv_b="${tmp_dir}/b.csv"
report="${tmp_dir}/report.md"

cat > "${csv_a}" <<'CSV'
event,duration_ms
success,10.0
skipped_small_motion,0.0
CSV

cat > "${csv_b}" <<'CSV'
event,duration_ms
failure,20.0
success,30.0
CSV

python3 scripts/compare_planning_benchmark_csvs.py \
  --input "a=${csv_a}" \
  --input "b=${csv_b}" \
  --output-markdown "${report}"

if [[ ! -f "${report}" ]]; then
  echo "FAIL: Markdown report was not created"
  exit 1
fi

if ! grep -q "Planning Benchmark Comparison" "${report}"; then
  echo "FAIL: report is missing title"
  exit 1
fi

if ! grep -q "| profile |" "${report}"; then
  echo "FAIL: report is missing metrics table"
  exit 1
fi

if ! grep -q "skipped_small_motion" "${report}"; then
  echo "FAIL: report is missing skipped_small_motion note"
  exit 1
fi

echo "PASS: Markdown benchmark report export works"
