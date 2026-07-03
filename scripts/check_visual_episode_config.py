#!/usr/bin/env python3
"""Statically validate the visual launch/profile contract."""

from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
launch = (root / 'src/adaptive_assembly_bringup/launch/'
          'adaptive_assembly_full_episode_visual_demo.launch.py').read_text()
config = (root / 'src/adaptive_assembly_bringup/config/'
          'adaptive_assembly_visual_single_trial_params.yaml').read_text()
required_launch = (
    'adaptive_assembly_visual_single_trial_params.yaml',
    'gazebo_target_pose_sync.launch.py',
    "'target_pose_topic': '/object_place_pose'",
    "'achieved_pose_topic': '/gazebo_target_object_pose'",
    "'achieved_pose_source': 'gazebo_entity_pose_observer'",
    "'require_execution_success': True",
    "'require_target_sync_success': 'true'",
    "'require_place_sequence': 'true'",
    "'attach_stage': 'grasp'",
    "'release_stage': 'place'",
    "'target_sync_status_topic': '/gazebo_target_sync_status'",
    "'target_sync_timeout_sec': '10.0'",
)
required_config = (
    'assembly_pose_mode: fixed_socket', 'socket_x: 0.62',
    'socket_y: -0.18', 'socket_z: 0.10',
    'pre_place_height_offset: 0.20',
    'place_height_offset: 0.00',
    'retreat_height_offset: 0.20',
)
missing = [item for item in required_launch if item not in launch]
missing += [item for item in required_config if item not in config]
if "'target_pose_topic': '/panda_assembly_pose'" in launch:
    missing.append('visual evaluator must not target /panda_assembly_pose')
if missing:
    print(f"FAIL: missing visual contract entries: {', '.join(missing)}")
    sys.exit(1)
planner_launch = (root / 'src/adaptive_assembly_planning/launch/'
                  'assembly_sequence_planning.launch.py').read_text()
executor = (root / 'src/adaptive_assembly_execution/adaptive_assembly_execution/'
            'ros2_control_sequence_executor_node.py').read_text()
lifecycle = (root / 'src/adaptive_assembly_manipulation/'
             'adaptive_assembly_manipulation/logical_grasp_lifecycle_node.py').read_text()
for stage in ('pre_place', 'place', 'retreat'):
    if f"'{stage}_trajectory_topic'" not in planner_launch:
        missing.append(f'planner launch missing {stage} trajectory')
ordered_stage_calls = all(
    f"self._send_stage('{stage}'" in executor
    for stage in ('pre_place', 'place', 'retreat')
)
if not ordered_stage_calls:
    missing.append('executor missing ordered place stages')
if "fields.get('stage') == self._release_stage" not in lifecycle:
    missing.append('lifecycle missing stage-triggered release')
if missing:
    print(f"FAIL: missing visual contract entries: {', '.join(missing)}")
    sys.exit(1)
print('PASS: visual episode uses five-stage fixed-socket place semantics')
