#!/usr/bin/env python3
"""Statically check the supported physical pick-place launch composition."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP_LAUNCH_DIR = ROOT / 'src/adaptive_assembly_bringup/launch'
EXECUTION_LAUNCH = (
    BRINGUP_LAUNCH_DIR
    / 'adaptive_assembly_physical_pick_place_execution.launch.py'
)
FULL_LAUNCH = (
    BRINGUP_LAUNCH_DIR
    / 'adaptive_assembly_full_physical_pick_place_demo.launch.py'
)
PHYSICAL_PLANNING_LAUNCH = (
    BRINGUP_LAUNCH_DIR / 'adaptive_assembly_physical_planning.launch.py'
)
AUDIT_LAUNCH = (
    ROOT / 'src/adaptive_assembly_planning/launch'
    / 'planning_scene_audit.launch.py'
)
PLANNER_SOURCE = (
    ROOT / 'src/adaptive_assembly_planning/src'
    / 'assembly_sequence_planning_node.cpp'
)
TARGET_SCENE_SOURCE = (
    ROOT / 'src/adaptive_assembly_planning/src'
    / 'physical_target_planning_scene_node.cpp'
)
TARGET_SCENE_CONTRACT = (
    ROOT / 'src/adaptive_assembly_planning/include'
    / 'adaptive_assembly_planning/target_scene_contract.hpp'
)
PHYSICAL_PROFILE = (
    ROOT / 'src/adaptive_assembly_bringup/config'
    / 'adaptive_assembly_physical_pick_place_params.yaml'
)
CLEARANCE_SOURCE = (
    ROOT / 'src/adaptive_assembly_planning/src'
    / 'grasp_clearance_validation.cpp'
)


def _load(path: Path, label: str, failures: list[str]) -> str:
    if not path.is_file():
        failures.append(f'missing {label}: {path}')
        return ''
    return path.read_text(encoding='utf-8')


def _require(text: str, tokens: tuple[str, ...], scope: str, failures):
    for token in tokens:
        if token not in text:
            failures.append(f'missing {scope} token: {token}')


def main() -> int:
    failures = []
    execution_text = _load(EXECUTION_LAUNCH, 'execution launch', failures)
    full_text = _load(FULL_LAUNCH, 'full demo launch', failures)
    planning_text = _load(
        PHYSICAL_PLANNING_LAUNCH, 'physical planning launch', failures
    )
    audit_text = _load(AUDIT_LAUNCH, 'planning scene audit launch', failures)
    planner_text = _load(PLANNER_SOURCE, 'sequence planner source', failures)
    target_scene_text = _load(
        TARGET_SCENE_SOURCE, 'physical target scene source', failures
    )
    target_contract_text = _load(
        TARGET_SCENE_CONTRACT, 'physical target scene contract', failures
    )
    clearance_text = _load(
        CLEARANCE_SOURCE, 'grasp clearance validator', failures
    )

    _require(full_text, (
        'adaptive_assembly_panda_gazebo.launch.py',
        'gazebo_target_pose_adapter.launch.py',
        'adaptive_assembly_physical_planning.launch.py',
        'adaptive_assembly_physical_pick_place_execution.launch.py',
        'adaptive_assembly_physical_workcell.sdf',
        "default_value='false'",
        'UnlessCondition(launch_fake_object_pose_node)',
        "'input_pose_topic': '/gazebo_target_object_pose'",
        "'output_pose_topic': '/target_pose'",
        "'target_object_gazebo_pose_topic': '/model/target_object/pose'",
        "'object_pose_topic': '/gazebo_target_object_pose'",
        "'end_effector_link': end_effector_link",
    ), 'full-demo orchestration', failures)

    _require(planning_text, (
        'assembly_task_node',
        'move_group',
        'static_planning_scene_node',
        'planning_scene_audit_node',
        'panda_pre_grasp_pose_adapter_node',
        'assembly_sequence_planning_node',
        'pre_grasp,grasp,lift,pre_place,place,retreat',
        'assembly_tcp',
        'position_tolerance',
        'orientation_tolerance',
        'physical_workcell_planning_scene.yaml',
        'physical_target_planning_scene_node',
        "'linear_stage_names_csv': 'grasp'",
        "'linear_planning_pipeline_id': (",
        "'pilz_industrial_motion_planner'",
        "'linear_planner_id': 'LIN'",
        "'lock_after_successful_sequence': True",
        "'require_dynamic_target_scene_ready': True",
        "'require_grasp_clearance_validation': True",
        "'grasp_min_disallowed_clearance': 0.005",
        "'grasp_allowed_contact_links_csv': (",
        "'target_object'",
    ), 'physical-planning', failures)

    _require(execution_text, (
        'physical_pick_place_executor_node',
        'gripper_action_bridge_node',
        'gazebo_grasp_contact_status_node',
        'grasp_verifier_node',
        'physical_grasp_preflight_node',
        'gazebo_entity_pose_observer_node',
        'stage_names',
        'lift_trajectory_topic',
        'send_gripper_commands',
        'open_gripper_before_first_arm_stage',
        'simulated_execution_only',
        'launch_gripper_bridge',
        'launch_physical_grasp_preflight',
        'require_physical_grasp_preflight',
        'physical_grasp_preflight_timeout_sec',
        '/physical_grasp_preflight_status',
        '/world/adaptive_assembly_physical_workcell/pose/info',
        "'require_target_entity_exact_match': 'false'",
        "'send_arm_goals': 'true'",
        "'require_model_name_match': _typed_value(",
        "'require_target_entity_exact_match', bool",
        "'output_pose_topic': LaunchConfiguration('object_pose_topic')",
        "'require_plan_lock': 'true'",
        "'plan_lock_status_topic': '/assembly_sequence_plan_lock_status'",
    ), 'physical-execution', failures)

    _require('\n'.join((planning_text, audit_text)), (
        'expected_object_ids',
        'planning_scene_audit_expected_object_ids',
        'work_table,target_support',
        '/planning_scene_audit_status',
        '/planning_scene_audit_ready',
        'target_object',
    ), 'PlanningScene/audit', failures)

    _require(planner_text, (
        'setPlanningPipelineId(profile.planning_pipeline_id)',
        'setPlannerId(profile.planner_id)',
        'linear_planning_failed',
        'validate_linear_path(',
        'std::vector<Candidate> candidates',
        'State::LOCKED',
        'planning_started',
        'update_ignored',
        'evaluate_grasp_clearance(',
    ), 'sequence planner lock/LIN contract', failures)

    _require(clearance_text, (
        'distanceRobot(',
        'checkCollision(',
        'grasp_disallowed_collision',
        'grasp_clearance_below_minimum',
        'panda_finger_joint1',
        'panda_finger_joint2',
    ), 'grasp clearance validation', failures)

    if PHYSICAL_PROFILE.is_file():
        profile = yaml.safe_load(PHYSICAL_PROFILE.read_text(encoding='utf-8'))
        task = profile['assembly_task_node']['ros__parameters']
        clearance = profile[
            'assembly_sequence_planning_node'
        ]['ros__parameters']
        offset = float(task['grasp_height_offset'])
        if not 0.005 <= offset <= 0.030:
            failures.append('physical grasp offset is outside calibration range')
        if abs(task['pre_grasp_height_offset'] - offset - 0.20) > 1e-12:
            failures.append('physical approach distance is not 0.20 m')
        if clearance.get('require_grasp_clearance_validation') is not True:
            failures.append('physical grasp clearance validation is disabled')
        if float(clearance.get('grasp_min_disallowed_clearance', 0.0)) < 0.005:
            failures.append('physical minimum disallowed clearance is below 5 mm')
        if clearance.get('grasp_allowed_contact_links_csv') != (
            'panda_leftfinger,panda_rightfinger'
        ):
            failures.append('physical clearance allowlist is not finger-only')

    _require('\n'.join((target_scene_text, target_contract_text)), (
        'SolidPrimitive::CYLINDER',
        'CYLINDER_HEIGHT',
        'CYLINDER_RADIUS',
        'panda_leftfinger,panda_rightfinger',
        'set_acm_pair(acm, object_id, name, false)',
        'set_acm_pair(acm, object_id, link, true)',
        'scene.robot_state.is_diff = true',
        'locked_ = true',
    ), 'dynamic target PlanningScene', failures)

    combined_text = '\n'.join((full_text, planning_text, execution_text))
    for token in (
        'enable_real_hardware',
        'real_hardware:=true',
        'hardware_driver',
        'camera',
        "'require_model_name_match': True",
    ):
        if token in combined_text:
            failures.append(f'forbidden physical-path token present: {token}')

    legacy_tokens = (
        'adaptive_assembly_panda_sequence_planning_reachable.launch.py',
        'adaptive_assembly_panda_sequence_planning_demo.launch.py',
        'adaptive_assembly_panda_planning_demo.launch.py',
        'adaptive_assembly_panda_demo.launch.py',
        'adaptive_assembly_pipeline.launch.py',
        'launch_reachable_sequence',
        'use_standard_panda_demo',
    )
    for token in legacy_tokens:
        if token in combined_text:
            failures.append(f'legacy planning token present: {token}')

    pose_vector_bridge = '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    if pose_vector_bridge in execution_text:
        failures.append('physical SceneBroadcaster Pose_V bridge is present')
    pose_bridge = "'@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'"
    if execution_text.count(pose_bridge) != 1:
        failures.append('expected exactly one dedicated target Pose bridge')
    output_pose_source = (
        "'output_pose_topic': LaunchConfiguration('object_pose_topic')"
    )
    if execution_text.count(output_pose_source) != 1:
        failures.append(
            'expected exactly one /gazebo_target_object_pose observer source'
        )

    if failures:
        print('FAIL physical pick-place launch static check')
        for failure in failures:
            print(f'- {failure}')
        return 1
    print('PASS physical pick-place launch static check')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
