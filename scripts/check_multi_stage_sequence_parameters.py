#!/usr/bin/env python3
"""Validate PR65 multi-stage planner parameters and documentation."""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(path: Path, snippets: list[str]) -> list[str]:
    """Return missing snippets for one repository file."""
    text = path.read_text(encoding='utf-8')
    return [snippet for snippet in snippets if snippet not in text]


def main() -> int:
    """Check launch, source, and documentation contracts."""
    checks = {
        ROOT / (
            'src/adaptive_assembly_planning/src/'
            'assembly_sequence_planning_node.cpp'
        ): [
            'struct StageConfig', 'stage_names', 'stage_names_csv',
            'require_place_sequence', '"/panda_" + name + "_pose"',
            '"/" + name + "_trajectory"',
        ],
        ROOT / (
            'src/adaptive_assembly_planning/launch/'
            'assembly_sequence_planning.launch.py'
        ): [
            "'stage_names'", "default_value='pre_grasp,assembly'",
            "'lift_topic': '/panda_lift_pose'",
            "default_value='/lift_trajectory'",
        ],
        ROOT / 'docs/assembly_sequence_planner.md': [
            'pre_grasp -> grasp -> lift -> pre_place -> place -> retreat',
            'require_place_sequence', 'deprecated', 'PR66',
        ],
    }
    failures = []
    for path, snippets in checks.items():
        failures.extend(
            f'{path.relative_to(ROOT)}: {item}'
            for item in require(path, snippets)
        )
    if failures:
        print('FAIL: missing multi-stage planner contracts:')
        for failure in failures:
            print(f'  {failure}')
        return 1
    print('PASS: multi-stage sequence parameters and profiles are documented')
    return 0


if __name__ == '__main__':
    sys.exit(main())
