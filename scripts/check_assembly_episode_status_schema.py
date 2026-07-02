#!/usr/bin/env python3
"""Validate representative assembly episode status schema strings."""

import sys
from typing import Dict


REQUIRED_FIELDS = {
    'event', 'mode', 'stage', 'simulated_only', 'real_hardware'
}
TERMINAL_EVENTS = {'success', 'failure', 'timeout', 'skipped'}
KNOWN_STAGES = {
    'init',
    'wait_target_pose',
    'wait_planning',
    'wait_execution_start',
    'wait_pre_grasp_success',
    'wait_grasp_attached',
    'wait_assembly_execution',
    'wait_release',
    'wait_insertion_evaluation',
    'terminal',
}
BOOLEAN_FIELDS = {
    'target_pose_available',
    'planning_success',
    'pre_grasp_success',
    'assembly_success',
    'execution_success',
    'logical_grasp_attached',
    'logical_grasp_released',
    'gazebo_attach_success',
    'insertion_success',
    'episode_success',
    'simulated_only',
    'real_hardware',
}
NUMERIC_FIELDS = {
    'duration_ms',
    'insertion_error_mm',
    'insertion_error_deg',
    'gazebo_attach_pose_error_mm',
}

EXAMPLES = {
    'ready': (
        'event=ready;mode=assembly_episode;stage=init;simulated_only=true;'
        'real_hardware=false;episode_id=episode_001'
    ),
    'success': (
        'event=success;mode=assembly_episode;stage=terminal;'
        'simulated_only=true;real_hardware=false;episode_id=episode_001;'
        'target_pose_available=true;planning_success=true;'
        'pre_grasp_success=true;assembly_success=true;execution_success=true;'
        'logical_grasp_attached=true;logical_grasp_released=true;'
        'gazebo_attach_success=true;insertion_success=true;'
        'episode_success=true;duration_ms=8421.5;insertion_error_mm=1.2;'
        'insertion_error_deg=0.8;gazebo_attach_pose_error_mm=0.6'
    ),
    'failure': (
        'event=failure;mode=assembly_episode;stage=terminal;'
        'simulated_only=true;real_hardware=false;episode_id=episode_002;'
        'planning_success=true;pre_grasp_success=true;assembly_success=true;'
        'execution_success=false;episode_success=false;duration_ms=4100.0;'
        'failure_reason=execution_failed'
    ),
    'timeout': (
        'event=timeout;mode=assembly_episode;stage=terminal;'
        'simulated_only=true;real_hardware=false;episode_id=episode_003;'
        'planning_success=false;episode_success=false;duration_ms=30000.0;'
        'failure_reason=timeout_wait_planning'
    ),
}


def parse_status(status: str) -> Dict[str, str]:
    """Parse a semicolon-delimited key=value string."""
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if not item or '=' not in item:
            raise ValueError(f'invalid field {item!r}')
        key, value = item.split('=', 1)
        if not key or not value:
            raise ValueError(f'invalid field {item!r}')
        if key in fields:
            raise ValueError(f'duplicate field {key!r}')
        fields[key] = value
    return fields


def validate_status(name: str, status: str) -> None:
    """Validate one embedded example against the episode schema."""
    fields = parse_status(status)
    missing = REQUIRED_FIELDS - fields.keys()
    if missing:
        raise ValueError(f'missing required fields {sorted(missing)}')
    if fields['mode'] != 'assembly_episode':
        raise ValueError('mode must be assembly_episode')
    if fields['simulated_only'] != 'true':
        raise ValueError('simulated_only must be true')
    if fields['real_hardware'] != 'false':
        raise ValueError('real_hardware must be false')
    if fields['stage'] not in KNOWN_STAGES:
        raise ValueError(f"unknown stage {fields['stage']!r}")

    for field in BOOLEAN_FIELDS & fields.keys():
        if fields[field] not in {'true', 'false'}:
            raise ValueError(f'{field} must be true or false')
    for field in NUMERIC_FIELDS & fields.keys():
        try:
            float(fields[field])
        except ValueError as error:
            raise ValueError(f'{field} must be numeric') from error

    event = fields['event']
    if fields['stage'] == 'terminal' and event not in TERMINAL_EVENTS:
        raise ValueError(f'unknown terminal event {event!r}')
    if event in TERMINAL_EVENTS and fields['stage'] != 'terminal':
        raise ValueError('terminal events require stage=terminal')
    if name == 'success' and fields.get('episode_success') != 'true':
        raise ValueError('success example requires episode_success=true')
    if event in TERMINAL_EVENTS - {'success'} and not fields.get('failure_reason'):
        raise ValueError('non-success terminal example requires failure_reason')


def main() -> int:
    """Validate all embedded examples and report a concise result."""
    for name, status in EXAMPLES.items():
        try:
            validate_status(name, status)
        except ValueError as error:
            print(f'FAIL: {name} assembly episode status: {error}')
            return 1
    print(f'PASS: {len(EXAMPLES)} assembly episode status examples match the schema')
    return 0


if __name__ == '__main__':
    sys.exit(main())

