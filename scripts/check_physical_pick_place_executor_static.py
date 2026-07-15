#!/usr/bin/env python3
"""Check the physical executor's plan-lock wiring without launching ROS."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution'
    / 'physical_pick_place_executor_node.py'
)
LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)


def main() -> int:
    failures = []
    source = SOURCE.read_text(encoding='utf-8')
    launch = LAUNCH.read_text(encoding='utf-8')
    required_source = (
        "self.declare_parameter('require_plan_lock', False)",
        "'/assembly_sequence_plan_lock_status'",
        'DurabilityPolicy.VOLATILE',
        "event == 'planning_started'",
        'self._trajectories.clear()',
        "event != 'locked'",
        "fields.get('stage_sequence') != self._stage_sequence",
        'self._execution_trajectories = dict(self._trajectories)',
        'self._trajectory_set_frozen = True',
        'trajectory_update_after_plan_lock',
        'locked_plan_incomplete',
        'plan_lock_timeout',
    )
    for token in required_source:
        if token not in source:
            failures.append(f'missing executor token: {token}')
    for token in (
        "'require_plan_lock': 'true'",
        "'plan_lock_timeout_sec': '10.0'",
        "'plan_lock_status_topic': '/assembly_sequence_plan_lock_status'",
        "'open_gripper_before_first_arm_stage': 'true'",
    ):
        if token not in launch:
            failures.append(f'missing physical launch token: {token}')
    if failures:
        print('FAIL physical executor plan-lock static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical executor plan-lock static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
