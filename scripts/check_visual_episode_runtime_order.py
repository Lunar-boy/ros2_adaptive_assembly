#!/usr/bin/env python3
"""Validate visual episode controller gating and supervisor terminal logic."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def require(path: Path, snippets: tuple[str, ...], failures: list[str]) -> None:
    """Append missing static contract concepts."""
    text = path.read_text(encoding='utf-8')
    failures.extend(
        f'{path.relative_to(ROOT)} missing {snippet}'
        for snippet in snippets if snippet not in text
    )


def main() -> int:
    """Check launch ordering, retained target sync, and pending supervision."""
    failures: list[str] = []
    launch = ROOT / 'src/adaptive_assembly_bringup/launch/' \
        'adaptive_assembly_full_episode_visual_demo.launch.py'
    require(launch, (
        'gazebo_controller_ready_status',
        'require_gazebo_controller_ready',
        'wait_for_gazebo_controller_ready_node',
        'OnProcessExit',
        'start_after_readiness',
        "'launch_simulation': 'false'",
        'gazebo_target_pose_sync.launch.py',
        "'require_target_sync_success': 'true'",
        "'require_grasp_trajectory': 'true'",
        "'require_place_sequence': 'true'",
        'contact_lite_insertion_evaluator_node',
        'assembly_episode_supervisor.launch.py',
    ), failures)
    supervisor = ROOT / 'src/adaptive_assembly_episode/' \
        'adaptive_assembly_episode/assembly_episode_supervisor_node.py'
    require(supervisor, (
        'self._execution_terminal_received = False',
        'elif event in self._TERMINAL_FAILURE_EVENTS:',
        "self._fail_if_required('execution', 'execution_failed')",
        "stage = 'wait_execution_terminal'",
        "self._publish_terminal('timeout', 'episode_timeout')",
    ), failures)
    supervisor_text = supervisor.read_text(encoding='utf-8')
    callback = supervisor_text.split(
        'def _execution_success_cb', 1
    )[1].split('def _execution_duration_cb', 1)[0]
    if '_fail_if_required' in callback or '_publish_terminal' in callback:
        failures.append('execution Bool callback still publishes failure')
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS: visual downstream startup is controller-ready gated')
    return 0


if __name__ == '__main__':
    sys.exit(main())
