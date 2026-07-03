#!/usr/bin/env python3
"""Validate the simulator-only Gazebo controller readiness implementation."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    """Check installation, dependencies, readiness inputs, and status QoS."""
    node_path = (
        ROOT / 'src/adaptive_assembly_execution/adaptive_assembly_execution/'
        'wait_for_gazebo_controller_ready_node.py'
    )
    setup_path = ROOT / 'src/adaptive_assembly_execution/setup.py'
    package_path = ROOT / 'src/adaptive_assembly_execution/package.xml'
    failures = []
    if not node_path.is_file():
        failures.append(f'missing node: {node_path.relative_to(ROOT)}')
    else:
        text = node_path.read_text(encoding='utf-8')
        required = (
            'controller_manager_msgs.srv import ListControllers',
            'joint_state_broadcaster',
            'panda_arm_controller',
            '/panda_arm_controller/follow_joint_trajectory',
            '/joint_states',
            '/gazebo_controller_ready_status',
            'ReliabilityPolicy.RELIABLE',
            'DurabilityPolicy.TRANSIENT_LOCAL',
            'simulated_only:=false is not supported',
            'math.isfinite',
            "self._publish('failure', 'timeout')",
        )
        failures.extend(
            f'readiness node missing concept: {item}'
            for item in required if item not in text
        )
    setup = setup_path.read_text(encoding='utf-8')
    if 'wait_for_gazebo_controller_ready_node:main' not in setup:
        failures.append('console script is not installed')
    package = package_path.read_text(encoding='utf-8')
    if '<depend>controller_manager_msgs</depend>' not in package:
        failures.append('controller_manager_msgs dependency is missing')
    if failures:
        print('FAIL: ' + '; '.join(failures))
        return 1
    print('PASS: Gazebo controller readiness gate contract is present')
    return 0


if __name__ == '__main__':
    sys.exit(main())
