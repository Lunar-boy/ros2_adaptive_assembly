#!/usr/bin/env python3
"""Verify PR66 status schema fields are represented in source and docs."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FILES = [
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'physical_pick_place_executor_node.py',
    ROOT / 'docs/physical_pick_place_execution.md',
]


def main() -> int:
    text = '\n'.join(path.read_text(encoding='utf-8') for path in FILES)
    required_fields = [
        'event',
        'mode',
        'stage',
        'stage_index',
        'stage_count',
        'stage_sequence',
        'action',
        'command',
        'duration_ms',
        'execution',
        'simulated_execution_only',
        'real_hardware',
    ]
    failures = [
        f'missing status field: {field}'
        for field in required_fields
        if field not in text
    ]
    if 'mode=physical_pick_place' not in text:
        failures.append('mode=physical_pick_place is missing')
    if 'real_hardware=false' not in text:
        failures.append('real_hardware=false is missing')

    if failures:
        print('FAIL physical pick-place status schema check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place status schema check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
