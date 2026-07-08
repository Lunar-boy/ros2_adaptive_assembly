#!/usr/bin/env python3
"""Statically validate PR65 sequence status schema fields."""

import sys
from pathlib import Path


def main() -> int:
    """Require indexed stage and aggregate sequence fields."""
    root = Path(__file__).resolve().parents[1]
    source_path = root / (
        'src/adaptive_assembly_planning/src/'
        'assembly_sequence_planning_node.cpp'
    )
    source = source_path.read_text(encoding='utf-8')
    required = ('requested_stage_count=', 'stage_index=', 'stage_sequence=')
    missing = [field for field in required if field not in source]
    if missing:
        print(f'FAIL: missing status fields: {", ".join(missing)}')
        return 1
    print(
        'PASS: aggregate, stage, and trajectory schema fields exist'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
