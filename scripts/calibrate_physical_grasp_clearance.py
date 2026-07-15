#!/usr/bin/env python3
"""Run and record the canonical Panda grasp-height collision sweep."""

import argparse
import json
from pathlib import Path
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]


def parse_fields(text):
    """Parse one semicolon-delimited repository status line."""
    return dict(
        fragment.split('=', 1)
        for fragment in text.split(';')
        if '=' in fragment
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--timeout-sec', type=float, default=60.0)
    args = parser.parse_args()
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_dir = args.run_dir or ROOT / 'runs' / f'grasp_clearance_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        [
            'ros2', 'launch', 'adaptive_assembly_planning',
            'grasp_clearance_calibration.launch.py',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=args.timeout_sec,
    )
    log_text = completed.stdout + completed.stderr
    (run_dir / 'calibration.log').write_text(log_text, encoding='utf-8')
    candidates = []
    selected = None
    for line in log_text.splitlines():
        if 'event=candidate;' in line:
            fields = parse_fields(line[line.index('event=candidate;'):])
            candidates.append(fields)
        if 'event=selected;' in line:
            selected = parse_fields(line[line.index('event=selected;'):])
    valid_offsets = sorted(
        float(item['offset']) for item in candidates
        if item.get('valid') == 'true'
    )
    reason = ''
    if completed.returncode != 0:
        reason = f'calibration_process_failed:{completed.returncode}'
    elif len(candidates) != 26:
        reason = f'candidate_count_invalid:{len(candidates)}'
    elif not selected or not valid_offsets:
        reason = 'selection_missing'
    elif abs(float(selected['grasp_height_offset']) - valid_offsets[0]) > 1.0e-12:
        reason = 'selection_not_smallest_valid_candidate'
    result = {
        'passed': not reason,
        'reason': reason or 'success',
        'selected_grasp_height_offset': (
            float(selected['grasp_height_offset']) if selected else None
        ),
        'candidates': candidates,
    }
    (run_dir / 'result.json').write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    if reason:
        print(f'FAIL: {reason}; artifacts={run_dir}')
        return 1
    selected_candidate = next(
        item for item in candidates
        if float(item['offset']) == result['selected_grasp_height_offset']
    )
    print(
        'PASS: selected grasp_height_offset='
        f'{result["selected_grasp_height_offset"]:.3f}; '
        'minimum_disallowed_clearance='
        f'{float(selected_candidate["minimum_disallowed_clearance"]):.6f}; '
        f'nearest_disallowed_link={selected_candidate["nearest_disallowed_link"]}; '
        f'artifacts={run_dir}'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
