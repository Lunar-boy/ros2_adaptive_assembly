#!/usr/bin/env python3
"""Boundedly prove two-phase payload transitions in the actual PlanningScene."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents, RobotTrajectory
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


ROOT = Path(__file__).resolve().parents[1]
JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))
TRANSPORT_STAGES = ('lift', 'pre_place', 'place')


def fields(text):
    """Parse one semicolon-delimited status message."""
    return {
        key.strip(): value.strip()
        for part in text.split(';') if '=' in part
        for key, value in [part.split('=', 1)]
    }


class PayloadTransitionChecker(Node):
    """Correlate workflow events with independently queried scene state."""

    def __init__(self, controlled_verification=False):
        super().__init__('full_physical_payload_transition_checker')
        self.controlled_verification = controlled_verification
        self.failure = ''
        self.grasp_plan_id = None
        self.transport_plan_id = None
        self.grasp_locked = False
        self.grasp_verified = False
        self.attached_status = False
        self.transport_locked = False
        self.lift_transport_id = False
        self.open_succeeded = False
        self.detached_status = False
        self.retreat_after_detach = False
        self.scene_checks = set()
        self.pending_scene_checks = []
        self.scene_future = None
        self.scene_check_name = ''
        self.attachment_signature = None
        self.relative_pose = None
        self.latest_joints = None
        self.lift_first = None
        self.transport_start_delta = None
        self.status_log = []

        for phase, topic in (
            ('grasp', '/grasp_plan_lock_status'),
            ('transport', '/transport_plan_lock_status'),
        ):
            self.create_subscription(
                String, topic,
                lambda msg, name=phase: self.lock(name, msg), 20,
            )
        self.create_subscription(
            String, '/grasp_verification_status', self.verification, 20
        )
        self.create_subscription(
            String, '/payload_attachment_status', self.payload, 20
        )
        self.create_subscription(
            String, '/physical_pick_place_stage_status', self.stage, 50
        )
        self.create_subscription(
            JointState, '/joint_states', self.joints, 50
        )
        self.create_subscription(
            RobotTrajectory, '/lift_trajectory', self.lift_trajectory, 10
        )
        self.scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grasp_result_publisher = self.create_publisher(
            Bool, '/grasp_verified', retained
        )
        self.lift_result_publisher = self.create_publisher(
            Bool, '/lift_verified', retained
        )

    def lock(self, phase, message):
        data = fields(message.data)
        self.status_log.append(message.data)
        if data.get('event') != 'locked':
            return
        try:
            plan_id = int(data['plan_id'])
        except (KeyError, ValueError):
            self.failure = f'{phase}_plan_id_invalid'
            return
        if phase == 'grasp':
            self.grasp_locked = data.get('stage_sequence') == 'pre_grasp,grasp'
            self.grasp_plan_id = plan_id
        else:
            if not self.attached_status:
                self.failure = 'transport_locked_before_attachment'
                return
            self.transport_locked = (
                data.get('stage_sequence') == 'lift,pre_place,place,retreat'
            )
            self.transport_plan_id = plan_id
            if self.transport_plan_id == self.grasp_plan_id:
                self.failure = 'plan_ids_not_distinct'

    def verification(self, message):
        data = fields(message.data)
        self.status_log.append(message.data)
        if (
            data.get('event') == 'success'
            and data.get('verification') == 'grasp'
        ):
            self.grasp_verified = True

    def payload(self, message):
        data = fields(message.data)
        self.status_log.append(message.data)
        if data.get('event') == 'failure':
            self.failure = 'payload_' + data.get('operation', 'unknown') + ':' + data.get(
                'reason', 'failure'
            )
            return
        if data.get('event') != 'success':
            return
        if data.get('operation') == 'attach':
            if not self.grasp_verified:
                self.failure = 'attachment_before_grasp_verification'
                return
            self.attached_status = True
            self.pending_scene_checks.append(('attached', True))
        elif data.get('operation') == 'detach':
            if not self.open_succeeded:
                self.failure = 'detach_before_open'
                return
            self.detached_status = True
            self.pending_scene_checks.append(('detached', False))

    def stage(self, message):
        data = fields(message.data)
        self.status_log.append(message.data)
        if data.get('event') == 'failure':
            self.failure = 'executor:' + data.get('reason', 'failure')
            return
        stage = data.get('stage')
        if (
            data.get('event') == 'success'
            and data.get('action') == 'verification'
            and data.get('verification') == 'grasp'
        ):
            self.grasp_verified = True
        if (
            self.controlled_verification
            and data.get('event') == 'request'
            and data.get('action') == 'verification'
        ):
            result = Bool()
            result.data = True
            if data.get('verification') == 'grasp':
                self.grasp_result_publisher.publish(result)
            elif data.get('verification') == 'lift':
                self.lift_result_publisher.publish(result)
        if (
            data.get('event') == 'sending'
            and data.get('action') == 'arm'
            and stage in TRANSPORT_STAGES
        ):
            if not self.transport_locked or not self.attached_status:
                self.failure = f'{stage}_before_transport_ready'
                return
            if str(self.transport_plan_id) != data.get('plan_id'):
                self.failure = f'{stage}_wrong_plan_id'
                return
            if stage == 'lift':
                self.lift_transport_id = True
                if self.latest_joints is not None and self.lift_first is not None:
                    self.transport_start_delta = max(
                        abs(self.latest_joints[name] - value)
                        for name, value in zip(JOINTS, self.lift_first)
                    )
            self.pending_scene_checks.append((stage, True))
        if (
            data.get('event') == 'success'
            and data.get('action') == 'gripper'
            and stage == 'place'
            and data.get('command') == 'open'
        ):
            self.open_succeeded = True
        if (
            data.get('event') == 'sending'
            and data.get('action') == 'arm'
            and stage == 'retreat'
        ):
            if not self.detached_status or 'detached' not in self.scene_checks:
                self.failure = 'retreat_before_verified_detach'
            else:
                self.retreat_after_detach = True

    def joints(self, message):
        if len(message.name) != len(message.position):
            return
        sample = dict(zip(message.name, message.position))
        if all(name in sample and math.isfinite(sample[name]) for name in JOINTS):
            self.latest_joints = sample

    def lift_trajectory(self, message):
        trajectory = message.joint_trajectory
        if tuple(trajectory.joint_names) == JOINTS and trajectory.points:
            self.lift_first = tuple(trajectory.points[0].positions)

    def poll_scene(self):
        """Issue and complete one independent PlanningScene query at a time."""
        if self.scene_future is not None:
            if not self.scene_future.done():
                return
            try:
                scene = self.scene_future.result().scene
            except Exception as error:
                self.failure = f'planning_scene_query_failed:{error}'
                return
            self.validate_scene(self.scene_check_name, scene)
            self.scene_future = None
            self.scene_check_name = ''
            return
        if not self.pending_scene_checks or not self.scene_client.service_is_ready():
            return
        self.scene_check_name, _ = self.pending_scene_checks.pop(0)
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        self.scene_future = self.scene_client.call_async(request)

    def validate_scene(self, name, scene):
        world = [obj for obj in scene.world.collision_objects if obj.id == 'target_object']
        attached = [
            obj for obj in scene.robot_state.attached_collision_objects
            if obj.object.id == 'target_object'
        ]
        expected_attached = name != 'detached'
        if (len(world), len(attached)) != ((0, 1) if expected_attached else (1, 0)):
            self.failure = f'{name}_scene_not_exclusive:w{len(world)}:a{len(attached)}'
            return
        collision = attached[0].object if expected_attached else world[0]
        if len(collision.primitives) != 1 or len(collision.primitive_poses) != 1:
            self.failure = f'{name}_geometry_count_invalid'
            return
        primitive = collision.primitives[0]
        if (
            primitive.type != primitive.CYLINDER
            or len(primitive.dimensions) != 2
            or abs(primitive.dimensions[primitive.CYLINDER_RADIUS] - 0.035) > 1e-9
            or abs(primitive.dimensions[primitive.CYLINDER_HEIGHT] - 0.10) > 1e-9
        ):
            self.failure = f'{name}_cylinder_invalid'
            return
        if expected_attached:
            payload = attached[0]
            if payload.link_name != 'assembly_tcp' or tuple(payload.touch_links) != (
                'panda_leftfinger', 'panda_rightfinger'
            ):
                self.failure = f'{name}_attachment_contract_invalid'
                return
            pose = collision.primitive_poses[0]
            signature = tuple(primitive.dimensions) + (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            )
            if self.attachment_signature is None:
                self.attachment_signature = signature
                self.relative_pose = signature[2:]
            elif any(abs(a - b) > 1e-6 for a, b in zip(signature, self.attachment_signature)):
                self.failure = f'{name}_attachment_changed'
                return
        self.scene_checks.add(name)

    def complete(self):
        required_scenes = {'attached', 'lift', 'pre_place', 'place', 'detached'}
        return (
            self.grasp_locked and self.grasp_verified and self.attached_status
            and self.transport_locked and self.lift_transport_id
            and required_scenes.issubset(self.scene_checks)
            and self.open_succeeded and self.detached_status
            and self.retreat_after_detach
            and self.transport_start_delta is not None
        )


def stop(process):
    """Stop the launched process group with bounded escalation."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=300.0)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument(
        '--controlled-verification', action='store_true',
        help=(
            'Disable gripper actuation/verifier and inject positive verifier '
            'Bool results to exercise the real MoveIt transition path. This '
            'does not prove a physical grasp.'
        ),
    )
    args = parser.parse_args()
    run_dir = args.run_dir or ROOT / 'runs' / (
        'payload_transitions_' + time.strftime('%Y%m%d_%H%M%S')
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    world = Path(get_package_share_directory('adaptive_assembly_sim')) / (
        'worlds/adaptive_assembly_physical_workcell.sdf'
    )
    launch_log_path = run_dir / 'launch.log'
    launch_log = launch_log_path.open('w', encoding='utf-8')
    command = [
            'ros2', 'launch', 'adaptive_assembly_bringup',
            'adaptive_assembly_full_physical_pick_place_demo.launch.py',
            'use_sim_time:=true', 'launch_fake_object_pose_node:=false',
            'launch_object_pose_observer:=true', f'gz_args:=-s {world}',
        ]
    if args.controlled_verification:
        command.extend([
            'send_gripper_commands:=false',
            'launch_grasp_verifier:=false',
        ])
    process = subprocess.Popen(
        command,
        cwd=ROOT, stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True,
    )
    checker = None
    try:
        rclpy.init()
        checker = PayloadTransitionChecker(args.controlled_verification)
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline and not checker.complete():
            if process.poll() is not None:
                checker.failure = f'launch_exited:{process.returncode}'
                break
            rclpy.spin_once(checker, timeout_sec=0.1)
            checker.poll_scene()
            if checker.failure:
                break
        status_path = run_dir / 'evidence.log'
        status_path.write_text('\n'.join(checker.status_log) + '\n', encoding='utf-8')
        if checker.failure or not checker.complete():
            reason = checker.failure or 'timeout_before_complete_transition_proof'
            print(f'FAIL: {reason}; evidence={status_path}; launch_log={launch_log_path}')
            return 1
        print(
            'PASS: actual PlanningScene proved world->attached->world exclusivity; '
            f'grasp_plan_id={checker.grasp_plan_id}; '
            f'transport_plan_id={checker.transport_plan_id}; '
            f'relative_pose={checker.relative_pose}; '
            f'transport_start_max_delta={checker.transport_start_delta:.6f}; '
            f'controlled_verification={args.controlled_verification}; '
            f'evidence={status_path}; launch_log={launch_log_path}'
        )
        return 0
    finally:
        if checker is not None:
            checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop(process)
        launch_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
