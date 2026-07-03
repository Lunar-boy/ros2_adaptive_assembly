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
    "'target_pose_topic': '/panda_assembly_pose'",
    "'achieved_pose_topic': '/gazebo_target_object_pose'",
    "'achieved_pose_source': 'gazebo_entity_pose_observer'",
    "'require_execution_success': True",
)
required_config = (
    'assembly_pose_mode: fixed_socket', 'socket_x: 0.62',
    'socket_y: -0.18', 'socket_z: 0.10',
)
missing = [item for item in required_launch if item not in launch]
missing += [item for item in required_config if item not in config]
if missing:
    print(f"FAIL: missing visual contract entries: {', '.join(missing)}")
    sys.exit(1)
print('PASS: visual episode uses fixed socket profile and Gazebo pose semantics')
