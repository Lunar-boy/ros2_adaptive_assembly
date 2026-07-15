"""Execute PR65 arm stages with simulator-only gripper interleaving."""

import math
import time
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


DEFAULT_STAGE_NAMES = 'pre_grasp,grasp,lift,pre_place,place,retreat'
MODE = 'physical_pick_place'
MODE_STATUS_TOKEN = 'mode=physical_pick_place'
VERIFICATION_TERMINAL_REASONS = (
    'grasp_verification_failed',
    'grasp_verification_timeout',
    'lift_verification_failed',
    'lift_verification_timeout',
)
VERIFICATION_SKIP_REASONS = (
    'require_grasp_verification_false',
    'require_lift_verification_false',
)
DEFAULT_TRAJECTORY_TOPICS = {
    'pre_grasp': '/pre_grasp_trajectory',
    'grasp': '/grasp_trajectory',
    'lift': '/lift_trajectory',
    'pre_place': '/pre_place_trajectory',
    'place': '/place_trajectory',
    'retreat': '/retreat_trajectory',
}


def parse_status(status: str) -> Dict[str, str]:
    """Parse semicolon-delimited key/value status fields."""
    fields = {}
    for fragment in status.split(';'):
        if '=' not in fragment:
            continue
        key, value = fragment.split('=', 1)
        if key.strip():
            fields[key.strip()] = value.strip()
    return fields


def parse_stage_names(stage_names: str) -> list[str]:
    """Return non-empty comma-separated stage names."""
    return [name.strip() for name in stage_names.split(',') if name.strip()]


class PhysicalPickPlaceExecutorNode(Node):
    """Run a simulator-only pick-place sequence with gripper commands."""

    def __init__(self) -> None:
        super().__init__('physical_pick_place_executor_node')

        self.declare_parameter('stage_names', DEFAULT_STAGE_NAMES)
        for stage, topic in DEFAULT_TRAJECTORY_TOPICS.items():
            self.declare_parameter(
                f'{stage}_trajectory_topic', topic
            )
        self.declare_parameter(
            'arm_controller_action_name',
            '/panda_arm_controller/follow_joint_trajectory',
        )
        self.declare_parameter('wait_for_arm_controller_sec', 5.0)
        self.declare_parameter('arm_result_timeout_sec', 10.0)
        self.declare_parameter('cancel_arm_goal_on_timeout', True)
        self.declare_parameter('send_arm_goals', True)
        self.declare_parameter('gripper_command_topic', '/gripper_command')
        self.declare_parameter(
            'gripper_status_topic', '/physical_gripper_command_status'
        )
        self.declare_parameter(
            'gripper_success_topic', '/physical_gripper_command_success'
        )
        self.declare_parameter(
            'gripper_closed_topic', '/physical_gripper_closed'
        )
        self.declare_parameter('send_gripper_commands', True)
        self.declare_parameter('require_gripper_success', True)
        self.declare_parameter('open_gripper_before_first_arm_stage', True)
        self.declare_parameter('gripper_command_timeout_sec', 5.0)
        self.declare_parameter('expected_target_object', 'target_object')
        self.declare_parameter('require_grasp_verification', True)
        self.declare_parameter('require_lift_verification', True)
        self.declare_parameter('require_physical_grasp_preflight', True)
        self.declare_parameter(
            'physical_grasp_preflight_status_topic',
            '/physical_grasp_preflight_status',
        )
        self.declare_parameter('physical_grasp_preflight_timeout_sec', 5.0)
        self.declare_parameter(
            'grasp_verification_request_topic',
            '/grasp_verification_request',
        )
        self.declare_parameter(
            'grasp_verification_status_topic',
            '/grasp_verification_status',
        )
        self.declare_parameter('grasp_verified_topic', '/grasp_verified')
        self.declare_parameter('lift_verified_topic', '/lift_verified')
        self.declare_parameter('verification_timeout_sec', 5.0)
        self.declare_parameter('close_after_stage', 'grasp')
        self.declare_parameter('open_after_stage', 'place')
        self.declare_parameter('require_non_empty_trajectory', True)
        self.declare_parameter('require_panda_joints', True)
        self.declare_parameter('expected_joint_prefix', 'panda_joint')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('require_joint_state', True)
        self.declare_parameter('start_state_tolerance', 0.05)
        self.declare_parameter('simulated_execution_only', True)

        self._stage_names = parse_stage_names(
            str(self.get_parameter('stage_names').value)
        )
        self._stage_sequence = ','.join(self._stage_names)
        self._trajectory_topics = {
            stage: str(self.get_parameter(f'{stage}_trajectory_topic').value)
            for stage in DEFAULT_TRAJECTORY_TOPICS
        }
        self._arm_controller_action_name = str(
            self.get_parameter('arm_controller_action_name').value
        )
        self._wait_for_arm_controller_sec = float(
            self.get_parameter('wait_for_arm_controller_sec').value
        )
        self._arm_result_timeout_sec = float(
            self.get_parameter('arm_result_timeout_sec').value
        )
        self._cancel_arm_goal_on_timeout = bool(
            self.get_parameter('cancel_arm_goal_on_timeout').value
        )
        self._send_arm_goals = bool(
            self.get_parameter('send_arm_goals').value
        )
        self._gripper_command_topic = str(
            self.get_parameter('gripper_command_topic').value
        )
        self._gripper_status_topic = str(
            self.get_parameter('gripper_status_topic').value
        )
        self._gripper_success_topic = str(
            self.get_parameter('gripper_success_topic').value
        )
        self._gripper_closed_topic = str(
            self.get_parameter('gripper_closed_topic').value
        )
        self._send_gripper_commands = bool(
            self.get_parameter('send_gripper_commands').value
        )
        self._require_gripper_success = bool(
            self.get_parameter('require_gripper_success').value
        )
        self._open_gripper_before_first_arm_stage = bool(
            self.get_parameter(
                'open_gripper_before_first_arm_stage'
            ).value
        )
        self._gripper_command_timeout_sec = float(
            self.get_parameter('gripper_command_timeout_sec').value
        )
        self._expected_target_object = str(
            self.get_parameter('expected_target_object').value
        ).strip()
        self._require_grasp_verification = bool(
            self.get_parameter('require_grasp_verification').value
        )
        self._require_lift_verification = bool(
            self.get_parameter('require_lift_verification').value
        )
        self._require_physical_grasp_preflight = bool(
            self.get_parameter('require_physical_grasp_preflight').value
        )
        self._physical_grasp_preflight_status_topic = str(
            self.get_parameter(
                'physical_grasp_preflight_status_topic'
            ).value
        )
        self._physical_grasp_preflight_timeout_sec = float(
            self.get_parameter(
                'physical_grasp_preflight_timeout_sec'
            ).value
        )
        self._grasp_verification_request_topic = str(
            self.get_parameter('grasp_verification_request_topic').value
        )
        self._grasp_verification_status_topic = str(
            self.get_parameter('grasp_verification_status_topic').value
        )
        self._grasp_verified_topic = str(
            self.get_parameter('grasp_verified_topic').value
        )
        self._lift_verified_topic = str(
            self.get_parameter('lift_verified_topic').value
        )
        self._verification_timeout_sec = float(
            self.get_parameter('verification_timeout_sec').value
        )
        self._close_after_stage = str(
            self.get_parameter('close_after_stage').value
        )
        self._open_after_stage = str(
            self.get_parameter('open_after_stage').value
        )
        self._require_non_empty_trajectory = bool(
            self.get_parameter('require_non_empty_trajectory').value
        )
        self._require_panda_joints = bool(
            self.get_parameter('require_panda_joints').value
        )
        self._expected_joint_prefix = str(
            self.get_parameter('expected_joint_prefix').value
        )
        self._joint_state_topic = str(
            self.get_parameter('joint_state_topic').value
        )
        self._require_joint_state = bool(
            self.get_parameter('require_joint_state').value
        )
        self._start_state_tolerance = float(
            self.get_parameter('start_state_tolerance').value
        )
        self._simulated_execution_only = bool(
            self.get_parameter('simulated_execution_only').value
        )
        self._expected_joints = [
            f'{self._expected_joint_prefix}{index}' for index in range(1, 8)
        ]
        self._validate_parameters()

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        stage_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, '/physical_pick_place_execution_status', retained_qos
        )
        self._success_publisher = self.create_publisher(
            Bool, '/physical_pick_place_execution_success', retained_qos
        )
        self._duration_publisher = self.create_publisher(
            Float64,
            '/physical_pick_place_execution_duration_ms',
            retained_qos,
        )
        self._stage_status_publisher = self.create_publisher(
            String, '/physical_pick_place_stage_status', stage_qos
        )
        self._gripper_command_publisher = self.create_publisher(
            String, self._gripper_command_topic, 10
        )
        self._verification_request_publisher = self.create_publisher(
            String, self._grasp_verification_request_topic, 10
        )
        self.create_subscription(
            String,
            self._gripper_status_topic,
            self._gripper_status_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool, self._gripper_success_topic, self._gripper_success_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool, self._gripper_closed_topic, self._gripper_closed_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            self._grasp_verification_status_topic,
            self._verification_status_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            self._grasp_verified_topic,
            self._grasp_verified_callback,
            retained_qos,
        )
        self.create_subscription(
            Bool,
            self._lift_verified_topic,
            self._lift_verified_callback,
            retained_qos,
        )
        self.create_subscription(
            String,
            self._physical_grasp_preflight_status_topic,
            self._physical_grasp_preflight_status_callback,
            retained_qos,
        )
        self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_callback, 10
        )
        self._trajectory_subscriptions = [
            self.create_subscription(
                RobotTrajectory,
                self._trajectory_topics[stage],
                lambda trajectory, name=stage: self._trajectory_callback(
                    name, trajectory
                ),
                10,
            )
            for stage in self._stage_names
        ]

        self._action_client = ActionClient(
            self, FollowJointTrajectory, self._arm_controller_action_name
        )
        self._trajectories: Dict[str, RobotTrajectory] = {}
        self._current_joint_positions: Dict[str, float] = {}
        self._joint_state_received = False
        self._state = 'WAIT_INPUTS'
        self._started = False
        self._completed = False
        self._current_stage_index = 0
        self._current_stage = ''
        self._current_goal_handle = None
        self._arm_result_timer = None
        self._sequence_start = 0.0
        self._stage_start = 0.0
        self._gripper_deadline: Optional[float] = None
        self._active_gripper_command: Optional[str] = None
        self._active_gripper_stage = ''
        self._active_gripper_command_id = ''
        self._gripper_command_count = 0
        self._verification_deadline: Optional[float] = None
        self._active_verification: Optional[str] = None
        self._active_verification_stage = ''
        self._preflight_success = not self._require_physical_grasp_preflight
        self._preflight_failure = False
        self._preflight_failure_reason = ''
        self._preflight_deadline: Optional[float] = None
        self._timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            'Simulator-only physical pick-place executor ready: '
            f"stage_names='{self._stage_sequence}', "
            f"arm_controller='{self._arm_controller_action_name}', "
            f"gripper_command_topic='{self._gripper_command_topic}', "
            f"gripper_status_topic='{self._gripper_status_topic}', "
            f'send_arm_goals={self._send_arm_goals}, '
            f'send_gripper_commands={self._send_gripper_commands}, '
            'open_gripper_before_first_arm_stage='
            f'{self._open_gripper_before_first_arm_stage}, '
            'require_physical_grasp_preflight='
            f'{self._require_physical_grasp_preflight}, '
            f'require_grasp_verification={self._require_grasp_verification}, '
            f'require_lift_verification={self._require_lift_verification}, '
            'simulated_execution_only=true, real_hardware=false.'
        )

    def _validate_parameters(self) -> None:
        """Reject unsupported or internally inconsistent settings."""
        if not self._simulated_execution_only:
            raise ValueError(
                'simulated_execution_only must remain true; real hardware is '
                'not supported'
            )
        if not self._stage_names:
            raise ValueError('stage_names must not be empty')
        if self._close_after_stage not in self._stage_names:
            raise ValueError('close_after_stage must be present in stage_names')
        if self._open_after_stage not in self._stage_names:
            raise ValueError('open_after_stage must be present in stage_names')
        close_index = self._stage_names.index(self._close_after_stage)
        open_index = self._stage_names.index(self._open_after_stage)
        if open_index < close_index:
            raise ValueError('open_after_stage must not come before close_after_stage')
        if self._wait_for_arm_controller_sec < 0.0:
            raise ValueError(
                'wait_for_arm_controller_sec must be greater than or equal to zero'
            )
        if self._arm_result_timeout_sec <= 0.0:
            raise ValueError('arm_result_timeout_sec must be greater than zero')
        if self._gripper_command_timeout_sec <= 0.0:
            raise ValueError('gripper_command_timeout_sec must be greater than zero')
        if not self._expected_target_object:
            raise ValueError('expected_target_object must not be empty')
        if self._verification_timeout_sec <= 0.0:
            raise ValueError('verification_timeout_sec must be greater than zero')
        if self._physical_grasp_preflight_timeout_sec <= 0.0:
            raise ValueError(
                'physical_grasp_preflight_timeout_sec must be greater than zero'
            )
        if self._require_panda_joints and not self._expected_joint_prefix:
            raise ValueError(
                'expected_joint_prefix must not be empty when '
                'require_panda_joints is true'
            )
        if self._start_state_tolerance < 0.0:
            raise ValueError(
                'start_state_tolerance must be greater than or equal to zero'
            )

    def _trajectory_callback(
        self, stage: str, trajectory: RobotTrajectory
    ) -> None:
        """Store the first trajectory received for each configured stage."""
        if self._started or self._completed or stage in self._trajectories:
            return
        self._trajectories[stage] = trajectory
        self._try_start_sequence()

    def _joint_state_callback(self, joint_state: JointState) -> None:
        """Retain the latest complete finite Panda joint-state sample."""
        if len(joint_state.name) != len(joint_state.position):
            return
        positions = dict(zip(joint_state.name, joint_state.position))
        if not all(
            name in positions and math.isfinite(positions[name])
            for name in self._expected_joints
        ):
            return
        self._current_joint_positions = {
            name: positions[name] for name in self._expected_joints
        }
        self._joint_state_received = True
        self._try_start_sequence()

    def _gripper_status_callback(self, message: String) -> None:
        """Advance or fail based on the active gripper bridge status."""
        if self._completed or self._active_gripper_command is None:
            return
        fields = parse_status(message.data)
        command = fields.get('command', '')
        if command != self._active_gripper_command:
            return
        command_id = fields.get('command_id', '')
        if command_id != self._active_gripper_command_id:
            return
        event = fields.get('event', '')
        if event == 'success':
            result = fields.get('result', 'success')
            permitted = result == 'success' or (
                command == 'close' and result == 'contact_limited_success'
            )
            if not permitted:
                self._publish_gripper_failure(fields, 'internal_error')
                return
            self._finish_gripper_command(
                True, result=result, result_fields=fields
            )
            return
        initial_open = self._active_gripper_stage == 'initial_open'
        if event == 'failure' and (
            self._require_gripper_success or initial_open
        ):
            self._publish_gripper_failure(
                fields, fields.get('result', fields.get('reason', 'internal_error'))
            )
            return
        if event == 'failure':
            self._publish_stage_status(
                f'event=skipped;mode={MODE};stage={self._active_gripper_stage};'
                'action=gripper;reason=gripper_failure_ignored;'
                f'command={self._active_gripper_command};real_hardware=false'
            )
            self._finish_gripper_command(
                True,
                result='failure_ignored',
                result_fields=fields,
            )

    def _publish_gripper_failure(
        self, fields: Dict[str, str], classified_result: str
    ) -> None:
        """Preserve the bridge classification in stage and terminal status."""
        stage = self._active_gripper_stage
        command = self._active_gripper_command or 'unknown'
        action = (
            'initial_gripper_open' if stage == 'initial_open' else 'gripper'
        )
        self._publish_stage_status(
            f'event=failure;mode={MODE};stage={stage};action={action};'
            f'command={command};gripper_result={classified_result};'
            f'gripper_reason={fields.get("reason", classified_result)};'
            f'left_contact={fields.get("left_contact", "unknown")};'
            f'right_contact={fields.get("right_contact", "unknown")};'
            f'action_status={fields.get("action_status", "unknown")};'
            'real_hardware=false'
        )
        self.get_logger().warning(
            f'gripper {command} failed: '
            f'result={classified_result} '
            f'left_contact={fields.get("left_contact", "unknown")} '
            f'right_contact={fields.get("right_contact", "unknown")}'
        )
        failure_reason = (
            'initial_gripper_open_failed'
            if stage == 'initial_open'
            else 'gripper_command_failed'
        )
        self._publish_failure(
            stage, failure_reason,
            gripper_result=classified_result,
            gripper_reason=fields.get('reason', classified_result),
        )

    def _gripper_success_callback(self, _: Bool) -> None:
        """Subscribe for validation visibility; status strings drive state."""

    def _gripper_closed_callback(self, _: Bool) -> None:
        """Subscribe for validation visibility; status strings drive state."""

    def _verification_status_callback(self, message: String) -> None:
        """Advance or fail when the verifier publishes a matching result."""
        if self._completed or self._active_verification is None:
            return
        fields = parse_status(message.data)
        if fields.get('mode') != 'grasp_verifier':
            return
        if fields.get('verification') != self._active_verification:
            return
        event = fields.get('event', '')
        if event == 'success':
            self._finish_verification(True, '')
            return
        if event == 'failure':
            self._finish_verification(False, fields.get('reason', 'failure'))

    def _physical_grasp_preflight_status_callback(
        self, message: String
    ) -> None:
        """Record preflight success or fail before arm execution starts."""
        if self._completed or not self._require_physical_grasp_preflight:
            return
        fields = parse_status(message.data)
        if fields.get('mode') != 'physical_grasp_preflight':
            return
        event = fields.get('event', '')
        if event == 'success':
            self._preflight_success = True
            self._try_start_sequence()
            return
        if event == 'failure':
            self._preflight_failure = True
            self._preflight_failure_reason = fields.get('reason', 'failure')
            if self._normal_prerequisites_ready():
                self._publish_preflight_failure()

    def _grasp_verified_callback(self, message: Bool) -> None:
        """Accept Bool grasp results while a grasp verification is pending."""
        if self._active_verification != 'grasp':
            return
        self._finish_verification(bool(message.data), 'grasp_verifier_bool_false')

    def _lift_verified_callback(self, message: Bool) -> None:
        """Accept Bool lift results while a lift verification is pending."""
        if self._active_verification != 'lift':
            return
        self._finish_verification(bool(message.data), 'lift_verifier_bool_false')

    def _timer_callback(self) -> None:
        """Run deadline checks without sleeps."""
        if self._completed:
            return
        self._try_start_sequence()
        if (
            self._state == 'WAIT_GRIPPER_RESULT'
            and self._gripper_deadline is not None
            and time.monotonic() >= self._gripper_deadline
        ):
            initial_open = self._active_gripper_stage == 'initial_open'
            self._publish_failure(
                self._active_gripper_stage or 'gripper_command',
                'initial_gripper_open_timeout'
                if initial_open else 'gripper_result_timeout',
            )
        if (
            self._state == 'WAIT_VERIFICATION_RESULT'
            and self._verification_deadline is not None
            and time.monotonic() >= self._verification_deadline
        ):
            verification = self._active_verification or 'verification'
            reason = f'{verification}_verification_timeout'
            self._publish_failure(
                self._active_verification_stage or verification,
                reason,
            )
        if (
            self._state == 'WAIT_PREFLIGHT'
            and self._preflight_deadline is not None
            and time.monotonic() >= self._preflight_deadline
        ):
            self._publish_failure('preflight', 'physical_grasp_preflight_timeout')

    def _normal_prerequisites_ready(self) -> bool:
        """Return whether trajectories and joint state are ready."""
        if not all(stage in self._trajectories for stage in self._stage_names):
            return False
        if self._require_joint_state and not self._joint_state_received:
            return False
        return True

    def _try_start_sequence(self) -> None:
        """Start once all configured trajectories and joint state are present."""
        if self._started or self._completed:
            return
        if not self._normal_prerequisites_ready():
            return
        if (
            self._require_physical_grasp_preflight
            and not self._preflight_success
        ):
            if self._preflight_failure:
                self._publish_preflight_failure()
                return
            self._state = 'WAIT_PREFLIGHT'
            if self._preflight_deadline is None:
                self._preflight_deadline = (
                    time.monotonic()
                    + self._physical_grasp_preflight_timeout_sec
                )
                self._publish_stage_status(
                    f'event=waiting;mode={MODE};stage=preflight;'
                    'action=physical_grasp_preflight;'
                    'reason=waiting_for_physical_grasp_preflight;'
                    'simulated_execution_only=true;real_hardware=false'
                )
            return

        self._started = True
        self._state = 'EXEC_ARM_STAGE'
        self._sequence_start = time.monotonic()
        for stage in self._stage_names:
            valid, reason = self._validate_trajectory(self._trajectories[stage])
            if not valid:
                self._publish_failure(stage, reason)
                return

        if self._send_arm_goals and not self._action_client.wait_for_server(
            timeout_sec=self._wait_for_arm_controller_sec
        ):
            self._publish_failure(self._stage_names[0], 'arm_controller_unavailable')
            return

        self._log_start_state_difference(self._trajectories[self._stage_names[0]])
        if self._open_gripper_before_first_arm_stage:
            self._send_gripper_command('initial_open', 'open')
        else:
            self._send_current_arm_stage()

    def _publish_preflight_failure(self) -> None:
        """Fail startup when the physical grasp preflight reports failure."""
        self._publish_stage_status(
            f'event=failure;mode={MODE};stage=preflight;'
            'action=physical_grasp_preflight;'
            f'preflight_reason={self._preflight_failure_reason};'
            'simulated_execution_only=true;real_hardware=false'
        )
        self._publish_failure(
            'preflight',
            'physical_grasp_preflight_failed',
            preflight_reason=self._preflight_failure_reason,
        )

    def _validate_trajectory(
        self, trajectory: RobotTrajectory
    ) -> Tuple[bool, str]:
        """Validate one exported joint trajectory before action use."""
        joint_trajectory = trajectory.joint_trajectory
        if (
            self._require_non_empty_trajectory
            and not joint_trajectory.joint_names
        ):
            return False, 'empty_joint_names'
        if (
            self._require_non_empty_trajectory
            and not joint_trajectory.points
        ):
            return False, 'empty_trajectory'
        if (
            self._require_panda_joints
            and list(joint_trajectory.joint_names) != self._expected_joints
        ):
            return False, 'controller_joint_mismatch'
        joint_count = len(joint_trajectory.joint_names)
        previous_time = -1.0
        for index, point in enumerate(joint_trajectory.points):
            if len(point.positions) != joint_count:
                return False, f'invalid_position_count_point_{index}'
            for field_name in ('velocities', 'accelerations', 'effort'):
                values = getattr(point, field_name)
                if values and len(values) != joint_count:
                    return False, f'invalid_{field_name}_count_point_{index}'
            values = (
                list(point.positions) + list(point.velocities)
                + list(point.accelerations) + list(point.effort)
            )
            if not all(math.isfinite(value) for value in values):
                return False, f'non_finite_value_point_{index}'
            point_time = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1.0e-9
            )
            if point_time < 0.0:
                return False, f'negative_time_point_{index}'
            if index > 0 and point_time <= previous_time:
                return False, f'non_increasing_time_point_{index}'
            previous_time = point_time
        return True, ''

    def _log_start_state_difference(self, trajectory: RobotTrajectory) -> None:
        """Report whether execution starts from the state used by planning."""
        if (
            not self._joint_state_received
            or not trajectory.joint_trajectory.points
        ):
            return
        first = trajectory.joint_trajectory.points[0].positions
        current = [
            self._current_joint_positions[name] for name in self._expected_joints
        ]
        max_delta = max(abs(a - b) for a, b in zip(current, first))
        log = self.get_logger().warning if (
            max_delta > self._start_state_tolerance
        ) else self.get_logger().info
        log(
            'Physical pick-place start-state comparison: '
            f'current={current}, planned_first={list(first)}, '
            f'max_delta={max_delta:.6f}, '
            f'tolerance={self._start_state_tolerance:.6f}.'
        )

    def _send_current_arm_stage(self) -> None:
        """Send or simulate the current arm stage."""
        if self._current_stage_index >= len(self._stage_names):
            self._publish_success()
            return
        stage = self._stage_names[self._current_stage_index]
        trajectory = self._trajectories[stage]
        self._current_stage = stage
        self._stage_start = time.monotonic()
        self._state = 'EXEC_ARM_STAGE'
        self._publish_stage_status(
            f'event=sending;mode={MODE};stage={stage};'
            f'stage_index={self._current_stage_index};'
            f'stage_count={len(self._stage_names)};action=arm;'
            'real_hardware=false'
        )
        if not self._send_arm_goals:
            self._publish_arm_stage_success(stage)
            self._after_arm_stage_success(stage)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed_future: self._goal_response_callback(
                stage, completed_future
            )
        )

    def _goal_response_callback(self, stage: str, future) -> None:
        """Handle arm goal acceptance and request the action result."""
        if self._completed or stage != self._current_stage:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(
                f'Failed to send {stage} controller goal: {error}'
            )
            self._publish_failure(stage, 'arm_goal_send_failed')
            return
        if not goal_handle.accepted:
            self._publish_failure(stage, 'arm_goal_rejected')
            return

        self._current_goal_handle = goal_handle
        self._publish_stage_status(
            f'event=accepted;mode={MODE};stage={stage};'
            f'stage_index={self._current_stage_index};action=arm;'
            'controller_goal_accepted=true;real_hardware=false'
        )
        self._state = 'WAIT_ARM_RESULT'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future: self._arm_result_callback(
                stage, completed_future
            )
        )
        self._arm_result_timer = self.create_timer(
            self._arm_result_timeout_sec,
            lambda: self._arm_result_timeout_callback(stage),
        )

    def _arm_result_timeout_callback(self, stage: str) -> None:
        """Fail deterministically if the arm controller result is late."""
        if self._completed or stage != self._current_stage:
            return
        self._cancel_arm_result_timer()
        if self._cancel_arm_goal_on_timeout and self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()
        self._publish_failure(stage, 'arm_result_timeout')

    def _arm_result_callback(self, stage: str, future) -> None:
        """Advance after one arm stage succeeds."""
        if self._completed or stage != self._current_stage:
            return
        self._cancel_arm_result_timer()
        try:
            wrapped_result = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(
                f'Failed to receive {stage} controller result: {error}'
            )
            self._publish_failure(stage, 'arm_result_failed')
            return
        action_succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped_result.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not action_succeeded:
            self._publish_failure(stage, 'arm_result_failed')
            return
        self._publish_arm_stage_success(stage)
        self._after_arm_stage_success(stage)

    def _publish_arm_stage_success(self, stage: str) -> None:
        duration_ms = (time.monotonic() - self._stage_start) * 1000.0
        self._publish_stage_status(
            f'event=success;mode={MODE};stage={stage};'
            f'stage_index={self._current_stage_index};action=arm;'
            f'duration_ms={duration_ms:.6f};real_hardware=false'
        )

    def _after_arm_stage_success(self, stage: str) -> None:
        """Interleave gripper commands or continue to the next arm stage."""
        if stage == self._close_after_stage:
            self._send_gripper_command(stage, 'close')
            return
        if stage == 'lift':
            self._request_verification(stage, 'lift')
            return
        if stage == self._open_after_stage:
            self._send_gripper_command(stage, 'open')
            return
        self._advance_to_next_arm_stage()

    def _send_gripper_command(self, stage: str, command: str) -> None:
        """Publish or simulate one gripper command."""
        self._state = 'SEND_GRIPPER_COMMAND'
        self._active_gripper_stage = stage
        self._active_gripper_command = command
        self._gripper_command_count += 1
        self._active_gripper_command_id = str(self._gripper_command_count)
        action = (
            'initial_gripper_open' if stage == 'initial_open' else 'gripper'
        )
        self._publish_stage_status(
            f'event=sending;mode={MODE};stage={stage};action={action};'
            f'command={command};real_hardware=false'
        )
        if not self._send_gripper_commands:
            self._publish_stage_status(
                f'event=skipped;mode={MODE};stage={stage};action={action};'
                f'command={command};reason=send_gripper_commands_disabled;'
                'simulated=true;real_hardware=false'
            )
            self._finish_gripper_command(True, result='success')
            return

        message = String()
        message.data = (
            f'event=command;command={command};'
            f'command_id={self._active_gripper_command_id};'
            f'expected_target_object={self._expected_target_object};'
            'source=physical_pick_place_executor;'
            f'stage={stage};simulated=true;real_hardware=false'
        )
        self._gripper_command_publisher.publish(message)
        self._state = 'WAIT_GRIPPER_RESULT'
        self._gripper_deadline = (
            time.monotonic() + self._gripper_command_timeout_sec
        )

    def _finish_gripper_command(
        self,
        success: bool,
        *,
        result: str,
        result_fields: Optional[Dict[str, str]] = None,
    ) -> None:
        """Publish gripper success and continue the deterministic sequence."""
        if self._completed or self._active_gripper_command is None:
            return
        stage = self._active_gripper_stage
        command = self._active_gripper_command
        self._gripper_deadline = None
        self._active_gripper_command = None
        self._active_gripper_stage = ''
        self._active_gripper_command_id = ''
        if success:
            action = (
                'initial_gripper_open'
                if stage == 'initial_open' else 'gripper'
            )
            self._publish_stage_status(
                f'event=success;mode={MODE};stage={stage};action={action};'
                f'command={command};gripper_result={result};'
                'real_hardware=false'
            )
            if command == 'close':
                fields = result_fields or {}
                self.get_logger().info(
                    'gripper close completed: '
                    f'result={result} '
                    'target='
                    f'{fields.get("expected_target_object", self._expected_target_object)} '
                    f'left_contact={fields.get("left_contact", "unknown")} '
                    f'right_contact={fields.get("right_contact", "unknown")} '
                    f'settle_sec={fields.get("settle_sec", "0.0")} '
                    f'action_status={fields.get("action_status", "unknown")}'
                )
        if command == 'close':
            self._request_verification(stage, 'grasp')
            return
        if stage == 'initial_open':
            self._send_current_arm_stage()
            return
        self._advance_to_next_arm_stage()

    def _request_verification(self, stage: str, verification: str) -> None:
        """Request grasp or lift verification, or publish an explicit skip."""
        required = (
            self._require_grasp_verification
            if verification == 'grasp'
            else self._require_lift_verification
        )
        if not required:
            self._publish_verification_skipped(stage, verification)
            self._advance_to_next_arm_stage()
            return

        self._state = 'WAIT_VERIFICATION_RESULT'
        self._active_verification = verification
        self._active_verification_stage = stage
        self._verification_deadline = time.monotonic() + self._verification_timeout_sec
        message = String()
        message.data = (
            f'event=request;verification={verification};stage={stage};'
            'source=physical_pick_place_executor;simulated=true;'
            'real_hardware=false'
        )
        self._publish_stage_status(
            f'event=request;mode={MODE};stage={stage};'
            f'verification={verification};action=verification;'
            'simulated=true;real_hardware=false'
        )
        self._verification_request_publisher.publish(message)

    def _finish_verification(self, success: bool, verifier_reason: str) -> None:
        """Advance or fail after a verifier result."""
        if self._completed or self._active_verification is None:
            return
        verification = self._active_verification
        stage = self._active_verification_stage
        self._verification_deadline = None
        self._active_verification = None
        self._active_verification_stage = ''
        if success:
            self._publish_stage_status(
                f'event=success;mode={MODE};stage={stage};'
                f'verification={verification};action=verification;'
                'simulated=true;real_hardware=false'
            )
            self._advance_to_next_arm_stage()
            return

        self._publish_stage_status(
            f'event=failure;mode={MODE};stage={stage};'
            f'verification={verification};action=verification;'
            f'verifier_reason={verifier_reason};'
            'simulated=true;real_hardware=false'
        )
        self._publish_failure(stage, f'{verification}_verification_failed')

    def _publish_verification_skipped(
        self, stage: str, verification: str
    ) -> None:
        """Publish an explicit simulator-only verification skip."""
        self._publish_stage_status(
            f'event=verification_skipped;mode={MODE};stage={stage};'
            f'verification={verification};'
            f'reason=require_{verification}_verification_false;'
            'simulated=true;real_hardware=false'
        )

    def _advance_to_next_arm_stage(self) -> None:
        self._current_stage_index += 1
        if self._current_stage_index >= len(self._stage_names):
            self._publish_success()
            return
        self._send_current_arm_stage()

    def _publish_failure(
        self,
        stage: str,
        reason: str,
        *,
        preflight_reason: Optional[str] = None,
        gripper_result: Optional[str] = None,
        gripper_reason: Optional[str] = None,
    ) -> None:
        """Publish one retained terminal failure."""
        if self._sequence_start == 0.0:
            self._sequence_start = time.monotonic()
        duration_ms = (time.monotonic() - self._sequence_start) * 1000.0
        status = (
            f'event=failure;mode={MODE};stage={stage};reason={reason};'
        )
        if preflight_reason is not None:
            status += f'preflight_reason={preflight_reason};'
        if gripper_result is not None:
            status += f'gripper_result={gripper_result};'
        if gripper_reason is not None:
            status += f'gripper_reason={gripper_reason};'
        status += (
            'execution=false;simulated_execution_only=true;'
            'real_hardware=false'
        )
        self._publish_final_result(False, status, duration_ms)
        detail = (
            f', preflight_reason={preflight_reason}'
            if preflight_reason is not None else ''
        )
        self.get_logger().warning(
            f'Physical pick-place execution failed: stage={stage}, '
            f'reason={reason}{detail}.'
        )

    def _publish_success(self) -> None:
        """Publish one retained terminal success."""
        duration_ms = (time.monotonic() - self._sequence_start) * 1000.0
        status = (
            f'event=success;mode={MODE};stage_count={len(self._stage_names)};'
            f'stage_sequence={self._stage_sequence};'
            f'gripper_command_count={self._gripper_command_count};'
            f'gripper_interleaving=true;duration_ms={duration_ms:.6f};'
            'execution=true;simulated_execution_only=true;'
            'real_hardware=false'
        )
        self._publish_final_result(True, status, duration_ms)
        self.get_logger().info(
            'Simulator-only physical pick-place execution completed.'
        )

    def _publish_stage_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._stage_status_publisher.publish(message)

    def _publish_final_result(
        self, success: bool, status: str, duration_ms: float
    ) -> None:
        """Publish retained terminal result exactly once."""
        if self._completed:
            return
        self._state = 'SUCCESS' if success else 'FAILURE'
        self._cancel_arm_result_timer()
        self._verification_deadline = None
        self._preflight_deadline = None
        self._active_verification = None
        self._completed = True

        status_message = String()
        status_message.data = status
        self._status_publisher.publish(status_message)

        success_message = Bool()
        success_message.data = success
        self._success_publisher.publish(success_message)

        duration_message = Float64()
        duration_message.data = duration_ms
        self._duration_publisher.publish(duration_message)

    def _cancel_arm_result_timer(self) -> None:
        if self._arm_result_timer is None:
            return
        self._arm_result_timer.cancel()
        self.destroy_timer(self._arm_result_timer)
        self._arm_result_timer = None


def main(args=None) -> None:
    """Run the simulator-only physical pick-place executor."""
    rclpy.init(args=args)
    node = PhysicalPickPlaceExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
