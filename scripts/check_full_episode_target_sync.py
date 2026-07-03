#!/usr/bin/env python3
"""Statically validate target synchronization in the full episode launch."""

from pathlib import Path
import sys


launch_file = (
    Path(__file__).resolve().parents[1]
    / 'src/adaptive_assembly_bringup/launch/'
    / 'adaptive_assembly_full_episode_demo.launch.py'
)
launch = launch_file.read_text()
required = (
    'gazebo_target_pose_sync.launch.py',
    "'target_pose_topic': '/target_pose'",
    "'target_entity_name': config['target_entity_name']",
    "'world_frame': config['world_frame']",
    "'enable_service_calls': config['enable_service_calls']",
    "'simulated_only': config['simulated_only']",
    "'control_owner_topic': '/target_object_control_owner'",
    "'status_topic': '/gazebo_target_sync_status'",
)
missing = [entry for entry in required if entry not in launch]
if missing:
    print(f"FAIL: missing full episode target sync entries: {', '.join(missing)}")
    sys.exit(1)

print('PASS: full episode includes ownership-gated Gazebo target sync')
