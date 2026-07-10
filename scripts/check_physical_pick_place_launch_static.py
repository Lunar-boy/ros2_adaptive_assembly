#!/usr/bin/env python3
"""Static checks for the PR66 bringup launch file."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXECUTION_LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)
FULL_LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
)
PANDA_DEMO_LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_panda_demo.launch.py'
)
REACHABLE_LAUNCH = (
    ROOT / 'src/adaptive_assembly_bringup/launch'
    / 'adaptive_assembly_panda_sequence_planning_reachable.launch.py'
)
AUDIT_LAUNCH = (
    ROOT / 'src/adaptive_assembly_planning/launch'
    / 'planning_scene_audit.launch.py'
)


def main() -> int:
    failures = []
    if not EXECUTION_LAUNCH.exists():
        failures.append('physical pick-place launch file is missing')
        text = ''
    else:
        text = EXECUTION_LAUNCH.read_text(encoding='utf-8')
    full_text = FULL_LAUNCH.read_text(encoding='utf-8') if FULL_LAUNCH.exists() else ''
    panda_demo_text = (
        PANDA_DEMO_LAUNCH.read_text(encoding='utf-8')
        if PANDA_DEMO_LAUNCH.exists() else ''
    )
    reachable_text = (
        REACHABLE_LAUNCH.read_text(encoding='utf-8')
        if REACHABLE_LAUNCH.exists() else ''
    )
    audit_text = AUDIT_LAUNCH.read_text(encoding='utf-8') if AUDIT_LAUNCH.exists() else ''

    required = [
        'physical_pick_place_executor_node',
        'gripper_action_bridge_node',
        'stage_names',
        'lift_trajectory_topic',
        'send_gripper_commands',
        'simulated_execution_only',
        'launch_reachable_sequence',
        'launch_gripper_bridge',
        'launch_physical_grasp_preflight',
        'require_physical_grasp_preflight',
        'physical_grasp_preflight_timeout_sec',
        '/physical_grasp_preflight_status',
        '/world/adaptive_assembly_physical_workcell/pose/info',
        'adaptive_assembly_physical_workcell.sdf',
        'use_standard_panda_demo',
        "'use_standard_panda_demo': 'false'",
        "'require_target_entity_exact_match': 'false'",
        "'launch_fake_object_pose_node': launch_fake_object_pose_node",
        "default_value='false'",
        'gazebo_target_pose_adapter.launch.py',
        "'input_pose_topic': '/gazebo_target_object_pose'",
        "'output_pose_topic': '/target_pose'",
        'target_reference_z_offset',
        'UnlessCondition(launch_fake_object_pose_node)',
        "'require_model_name_match': _typed_value(",
        "'require_target_entity_exact_match', bool",
    ]
    for token in required:
        if token not in text and token not in full_text:
            failures.append(f'missing launch token: {token}')

    required_panda_demo_tokens = [
        "default_value='true'",
        'moveit_resources_panda_moveit_config',
        'moveit_ros_move_group',
        'UnlessCondition(use_standard_panda_demo)',
        'panda_gripper_controller',
    ]
    for token in required_panda_demo_tokens:
        if token not in panda_demo_text:
            failures.append(f'missing no-fake planning token: {token}')

    required_audit_tokens = [
        'expected_object_ids',
        'planning_scene_audit_expected_object_ids',
        'work_table,target_support',
    ]
    combined_audit_text = '\n'.join([reachable_text, audit_text])
    for token in required_audit_tokens:
        if token not in combined_audit_text:
            failures.append(f'missing planning scene audit token: {token}')

    forbidden = [
        'enable_real_hardware',
        'real_hardware:=true',
        'hardware_driver',
        'camera',
        "'require_model_name_match': True",
    ]
    for token in forbidden:
        if token in text or token in full_text:
            failures.append(f'forbidden launch token present: {token}')

    if failures:
        print('FAIL physical pick-place launch static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place launch static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
