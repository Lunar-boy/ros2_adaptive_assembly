#!/usr/bin/env python3
"""Statically validate the visual single-trial grasp-stage wiring."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def require(path: str, snippets: tuple[str, ...]) -> None:
    """Require every snippet in one repository file."""
    text = (ROOT / path).read_text(encoding='utf-8')
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        raise AssertionError(f'{path} missing: {missing}')


def main() -> int:
    """Check config, task output, adapter, trajectory, and attach wiring."""
    try:
        require(
            'src/adaptive_assembly_bringup/config/'
            'adaptive_assembly_visual_single_trial_params.yaml',
            ('z: 0.10', 'grasp_height_offset: 0.05'),
        )
        require(
            'src/adaptive_assembly_task/adaptive_assembly_task/'
            'assembly_task_node.py',
            ("grasp_pose_topic', '/grasp_pose'", '_grasp_publisher.publish'),
        )
        require(
            'src/adaptive_assembly_planning/launch/'
            'panda_grasp_pose_adapter.launch.py',
            ("'input_topic': '/grasp_pose'", "'output_topic': '/panda_grasp_pose'"),
        )
        require(
            'src/adaptive_assembly_bringup/launch/'
            'adaptive_assembly_full_episode_visual_demo.launch.py',
            ("'require_grasp_trajectory': 'true'", "'attach_stage': 'grasp'"),
        )
        require(
            'src/adaptive_assembly_execution/adaptive_assembly_execution/'
            'ros2_control_sequence_executor_node.py',
            ("'grasp_trajectory_topic', '/grasp_trajectory'", "stage == 'grasp'"),
        )
    except (AssertionError, OSError) as error:
        print(f'FAIL: {error}')
        return 1
    print('PASS: visual single-trial grasp-stage wiring is present')
    return 0


if __name__ == '__main__':
    sys.exit(main())
