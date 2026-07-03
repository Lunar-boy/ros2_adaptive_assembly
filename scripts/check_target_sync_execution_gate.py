#!/usr/bin/env python3
"""Statically validate target-sync execution-gate launch contracts."""

import sys
from pathlib import Path


root = Path(__file__).resolve().parents[1]
launch_dir = root / 'src/adaptive_assembly_bringup/launch'
executor = (
    root / 'src/adaptive_assembly_execution/adaptive_assembly_execution/'
    'ros2_control_sequence_executor_node.py'
).read_text()
generic = (
    launch_dir / 'adaptive_assembly_panda_ros2_control_execution.launch.py'
).read_text()
visual = (
    launch_dir / 'adaptive_assembly_full_episode_visual_demo.launch.py'
).read_text()

required_executor = (
    "'require_target_sync_success', False",
    "'target_sync_status_topic', '/gazebo_target_sync_status'",
    "'target_sync_timeout_sec', 10.0",
    'if self._target_sync_required and not self._target_sync_succeeded:',
    "self._publish_target_sync_terminal('target_sync_timeout')",
)
required_generic = (
    "'require_target_sync_success', default_value='false'",
    "'target_sync_status_topic'",
    "'target_sync_timeout_sec'",
)
required_visual = (
    "'require_target_sync_success': 'true'",
    "'target_sync_status_topic': '/gazebo_target_sync_status'",
    "'target_sync_timeout_sec': '10.0'",
)

missing = [item for item in required_executor if item not in executor]
missing += [item for item in required_generic if item not in generic]
missing += [item for item in required_visual if item not in visual]
if missing:
    print(f"FAIL: missing target-sync gate entries: {', '.join(missing)}")
    sys.exit(1)

print('PASS: visual execution requires target sync; generic default is false')
