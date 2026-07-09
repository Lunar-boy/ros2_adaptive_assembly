#!/usr/bin/env python3
"""Static checks for the manual full physical pick-place run logger."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/run_full_physical_pick_place_with_logs.sh'
DOC = ROOT / 'docs/run_logging.md'


def main() -> int:
    failures = []

    if not SCRIPT.exists():
        failures.append('manual run logging script is missing')
        script_text = ''
    else:
        script_text = SCRIPT.read_text(encoding='utf-8')

    script_requirements = [
        ('set -euo pipefail', 'script must enable strict bash mode'),
        ('RUN_ID', 'script must use RUN_ID'),
        ('RUN_DIR', 'script must use RUN_DIR'),
        ('metadata.env', 'script must write metadata.env'),
        ('launch.log', 'script must write launch.log'),
        (
            'ros2 launch "$LAUNCH_PACKAGE" "$LAUNCH_FILE"',
            'script must call ros2 launch with the configured package/file',
        ),
        ('adaptive_assembly_bringup', 'script must launch adaptive_assembly_bringup'),
        (
            'adaptive_assembly_full_physical_pick_place_demo.launch.py',
            'script must launch the full physical pick-place demo',
        ),
        ('tee', 'script must mirror launch output with tee'),
        ('PIPESTATUS', 'script must preserve the ros2 launch exit code'),
    ]
    for token, message in script_requirements:
        if token not in script_text:
            failures.append(message)

    if not DOC.exists():
        failures.append('run logging documentation is missing')
        doc_text = ''
    else:
        doc_text = DOC.read_text(encoding='utf-8')

    doc_requirements = [
        'bash scripts/run_full_physical_pick_place_with_logs.sh',
        'RUN_ID',
        'RUN_DIR',
        'metadata.env',
        'launch.log',
        'rosbag',
    ]
    for token in doc_requirements:
        if token not in doc_text:
            failures.append(f'documentation missing basic usage token: {token}')

    if failures:
        print('FAIL manual run logging static check')
        for failure in failures:
            print(f'- {failure}')
        return 1

    print('PASS manual run logging static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
