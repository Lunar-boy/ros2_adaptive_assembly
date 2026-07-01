#!/usr/bin/env bash

set -euo pipefail

tmp_dir="$(mktemp -d /tmp/contact_lite_insertion_report.XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

csv_path="${tmp_dir}/insertion.csv"
report_path="${tmp_dir}/summary.md"

{
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    'trial_id' \
    'timestamp_sec' \
    'success' \
    'position_error_mm' \
    'orientation_error_deg' \
    'position_tolerance_mm' \
    'orientation_tolerance_deg' \
    'execution_required' \
    'execution_success' \
    'achieved_pose_source' \
    'status'
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    '1' '1.0' 'true' '1.0' '2.0' '5.0' '5.0' \
    'false' 'unavailable' 'planned_pose' 'event=insertion_evaluated'
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    '2' '2.0' 'false' '7.0' '8.0' '5.0' '5.0' \
    'false' 'unavailable' 'planned_pose' 'event=insertion_evaluated'
} > "${csv_path}"

python3 scripts/summarize_contact_lite_insertion_csv.py \
  --input "${csv_path}" \
  --output-markdown "${report_path}"

if [[ ! -f "${report_path}" ]]; then
  echo "FAIL: Markdown insertion summary was not created"
  exit 1
fi

if ! grep -q "Contact-lite Insertion Benchmark Summary" "${report_path}"; then
  echo "FAIL: report is missing title"
  exit 1
fi

if ! grep -q "success rate: 0.500" "${report_path}"; then
  echo "FAIL: report is missing expected success rate"
  exit 1
fi

if ! grep -q "contact-lite geometric benchmark only" "${report_path}"; then
  echo "FAIL: report is missing contact-lite limitation"
  exit 1
fi

echo "PASS: contact-lite insertion Markdown report export works"
