"""Forward assembly trajectories to a simulated ros2_control action."""

import math
import time
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus

from control_msgs.action import FollowJointTrajectory

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool, Float64, String


class Ros2ControlSequenceExecutorNode(Node):
    """Execute one exported two-stage sequence through ros2_control."""

    def __init__(self) -> None:
        """Declare parameters and create the ROS interfaces."""
        super().__init__('ros2_control_sequence_executor_node')

        self.declare_parameter(
            'pre_grasp_trajectory_topic', '/pre_grasp_trajectory'
        )
        self.declare_parameter(
            'assembly_trajectory_topic', '/assembly_trajectory'
        )
        self.declare_parameter(
            'controller_action_name',
            '/panda_arm_controller/follow_joint_trajectory',
        )
        self.declare_parameter(
            'status_topic', '/assembly_ros2_control_execution_status'
        )
        self.declare_parameter(
            'success_topic', '/assembly_ros2_control_execution_success'
        )
        self.declare_parameter(
            'duration_topic',
            '/assembly_ros2_control_execution_duration_ms',
        )
        self.declare_parameter(
            'stage_status_topic',
            '/assembly_ros2_control_execution_stage_status',
        )
        self.declare_parameter('wait_for_controller_sec', 5.0)
        self.declare_parameter('result_timeout_sec', 10.0)
        self.declare_parameter('cancel_on_timeout', True)
        self.declare_parameter('send_goals', True)
        self.declare_parameter('require_non_empty_trajectory', True)
        self.declare_parameter('require_panda_joints', True)
        self.declare_parameter('expected_joint_prefix', 'panda_joint')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('require_joint_state', True)
        self.declare_parameter('start_state_tolerance', 0.05)
        self.declare_parameter('simulated_execution_only', True)

        self._pre_grasp_topic = self.get_parameter(
            'pre_grasp_trajectory_topic'
        ).value
        self._assembly_topic = self.get_parameter(
            'assembly_trajectory_topic'
        ).value
        self._controller_action_name = self.get_parameter(
            'controller_action_name'
        ).value
        self._status_topic = self.get_parameter('status_topic').value
        self._success_topic = self.get_parameter('success_topic').value
        self._duration_topic = self.get_parameter('duration_topic').value
        self._stage_status_topic = self.get_parameter(
            'stage_status_topic'
        ).value
        self._wait_for_controller_sec = self.get_parameter(
            'wait_for_controller_sec'
        ).value
        self._send_goals = self.get_parameter('send_goals').value
        self._result_timeout_sec = self.get_parameter(
            'result_timeout_sec'
        ).value
        self._cancel_on_timeout = self.get_parameter(
            'cancel_on_timeout'
        ).value
        self._require_non_empty_trajectory = self.get_parameter(
            'require_non_empty_trajectory'
        ).value
        self._require_panda_joints = self.get_parameter(
            'require_panda_joints'
        ).value
        self._expected_joint_prefix = self.get_parameter(
            'expected_joint_prefix'
        ).value
        self._joint_state_topic = self.get_parameter(
            'joint_state_topic'
        ).value
        self._require_joint_state = self.get_parameter(
            'require_joint_state'
        ).value
        self._start_state_tolerance = self.get_parameter(
            'start_state_tolerance'
        ).value
        self._expected_joints = [
            f'{self._expected_joint_prefix}{index}' for index in range(1, 8)
        ]
        self._simulated_execution_only = self.get_parameter(
            'simulated_execution_only'
        ).value
        self._validate_parameters()

        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, result_qos
        )
        self._success_publisher = self.create_publisher(
            Bool, self._success_topic, result_qos
        )
        self._duration_publisher = self.create_publisher(
            Float64, self._duration_topic, result_qos
        )
        self._stage_status_publisher = self.create_publisher(
            String, self._stage_status_topic, 10
        )

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._controller_action_name,
        )
        self._pre_grasp_subscription = self.create_subscription(
            RobotTrajectory,
            self._pre_grasp_topic,
            self._pre_grasp_callback,
            10,
        )
        self._assembly_subscription = self.create_subscription(
            RobotTrajectory,
            self._assembly_topic,
            self._assembly_callback,
            10,
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            10,
        )

        self._pre_grasp_trajectory: Optional[RobotTrajectory] = None
        self._assembly_trajectory: Optional[RobotTrajectory] = None
        self._sequence_start = 0.0
        self._stage_start = 0.0
        self._started = False
        self._completed = False
        self._result_timer = None
        self._current_joint_positions = {}
        self._joint_state_received = False

        self.get_logger().info(
            'Simulator-only ros2_control sequence executor ready: '
            f"pre_grasp_topic='{self._pre_grasp_topic}', "
            f"assembly_topic='{self._assembly_topic}', "
            f"controller='{self._controller_action_name}', "
            f'wait_for_controller_sec={self._wait_for_controller_sec}, '
            f'result_timeout_sec={self._result_timeout_sec}, '
            f'cancel_on_timeout={self._cancel_on_timeout}, '
            f'send_goals={self._send_goals}, '
            f"joint_state_topic='{self._joint_state_topic}', "
            f'require_joint_state={self._require_joint_state}, '
            'simulated_execution_only=true, real_hardware=false.'
        )

    def _validate_parameters(self) -> None:
        """Reject unsafe or internally inconsistent settings."""
        if self._wait_for_controller_sec < 0.0:
            raise ValueError(
                'wait_for_controller_sec must be greater than or equal to zero'
            )
        if self._result_timeout_sec <= 0.0:
            raise ValueError('result_timeout_sec must be greater than zero')
        if self._require_panda_joints and not self._expected_joint_prefix:
            raise ValueError(
                'expected_joint_prefix must not be empty when '
                'require_panda_joints is true'
            )
        if self._start_state_tolerance < 0.0:
            raise ValueError(
                'start_state_tolerance must be greater than or equal to zero'
            )
        if not self._simulated_execution_only:
            raise ValueError(
                'simulated_execution_only must remain true; real hardware is '
                'not supported'
            )

    def _pre_grasp_callback(self, trajectory: RobotTrajectory) -> None:
        """Store the first pre-grasp trajectory and try execution."""
        if self._started or self._pre_grasp_trajectory is not None:
            return
        self._pre_grasp_trajectory = trajectory
        self._try_start_sequence()

    def _assembly_callback(self, trajectory: RobotTrajectory) -> None:
        """Store the first assembly trajectory and try execution."""
        if self._started or self._assembly_trajectory is not None:
            return
        self._assembly_trajectory = trajectory
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

    def _try_start_sequence(self) -> None:
        """Validate both trajectories and connect to the controller once."""
        if self._started:
            return
        if self._pre_grasp_trajectory is None:
            return
        if self._assembly_trajectory is None:
            return
        if self._require_joint_state and not self._joint_state_received:
            return

        self._started = True
        self._sequence_start = time.monotonic()

        valid, reason = self._validate_trajectory(self._pre_grasp_trajectory)
        if not valid:
            self._publish_failure('pre_grasp', reason)
            return
        valid, reason = self._validate_trajectory(self._assembly_trajectory)
        if not valid:
            self._publish_failure('assembly', reason)
            return

        if not self._send_goals:
            self._publish_skipped('send_goals_disabled')
            return

        if not self._action_client.wait_for_server(
            timeout_sec=self._wait_for_controller_sec
        ):
            self._publish_skipped('controller_unavailable')
            return

        self._log_start_state_difference(self._pre_grasp_trajectory)
        self._send_stage('pre_grasp', self._pre_grasp_trajectory)

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

    def _trajectory_summary(self, trajectory: RobotTrajectory) -> str:
        """Return validation-relevant trajectory fields for diagnostics."""
        joint_trajectory = trajectory.joint_trajectory
        times = [
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1.0e-9
            for point in joint_trajectory.points
        ]
        first_positions = (
            list(joint_trajectory.points[0].positions)
            if joint_trajectory.points else []
        )
        return (
            f'joints={list(joint_trajectory.joint_names)}, '
            f'point_count={len(joint_trajectory.points)}, '
            f'first_positions={first_positions}, times={times}'
        )

    def _log_start_state_difference(self, trajectory: RobotTrajectory) -> None:
        """Report whether execution starts from the state used by planning."""
        if (
            not self._joint_state_received
            or not trajectory.joint_trajectory.points
        ):
            return
        first = trajectory.joint_trajectory.points[0].positions
        current = [
            self._current_joint_positions[name]
            for name in self._expected_joints
        ]
        max_delta = max(abs(a - b) for a, b in zip(current, first))
        log = self.get_logger().warning if (
            max_delta > self._start_state_tolerance
        ) else self.get_logger().info
        log(
            'Pre-grasp execution start-state comparison: '
            f'current={current}, planned_first={list(first)}, '
            f'max_delta={max_delta:.6f}, '
            f'tolerance={self._start_state_tolerance:.6f}.'
        )

    def _send_stage(
        self, stage: str, trajectory: RobotTrajectory
    ) -> None:
        """Send one FollowJointTrajectory goal asynchronously."""
        self._stage_start = time.monotonic()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        # A zero stamp means "start now" and avoids MoveIt export timestamps
        # becoming stale while the two-stage sequence waits for the controller.
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        self.get_logger().info(
            f'Sending {stage} trajectory: '
            f'{self._trajectory_summary(trajectory)}'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed_future: self._goal_response_callback(
                stage, trajectory, completed_future
            )
        )

    def _goal_response_callback(
        self, stage: str, trajectory: RobotTrajectory, future
    ) -> None:
        """Handle goal acceptance and request the action result."""
        if self._completed:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(
                f'Failed to send {stage} controller goal: {error}'
            )
            self._publish_failure(stage, 'action_send_failed')
            return

        if not goal_handle.accepted:
            summary = self._trajectory_summary(trajectory)
            current = [
                self._current_joint_positions.get(name, float('nan'))
                for name in self._expected_joints
            ]
            self.get_logger().error(
                f'Controller rejected {stage} goal: {summary}, '
                f'expected_joints={self._expected_joints}, '
                f'current_positions={current}, '
                'header_stamp_normalized=true.'
            )
            self._publish_stage_status(
                f'event=rejected;mode=ros2_control;stage={stage};'
                f'point_count={len(trajectory.joint_trajectory.points)};'
                f'joint_count={len(trajectory.joint_trajectory.joint_names)};'
                'header_stamp_normalized=true;real_hardware=false'
            )
            self._publish_failure(stage, 'goal_rejected')
            return

        joint_trajectory = trajectory.joint_trajectory
        self._publish_stage_status(
            f'event=sent;mode=ros2_control;stage={stage};'
            f'point_count={len(joint_trajectory.points)};'
            f'joint_count={len(joint_trajectory.joint_names)};'
            f'controller={self._controller_action_name};real_hardware=false'
        )
        result_future = goal_handle.get_result_async()
        self._result_timer = self.create_timer(
            self._result_timeout_sec,
            lambda: self._result_timeout_callback(stage, goal_handle),
        )
        result_future.add_done_callback(
            lambda completed_future: self._result_callback(
                stage, completed_future
            )
        )

    def _result_timeout_callback(self, stage: str, goal_handle) -> None:
        """Terminate deterministically when a controller result is late."""
        if self._completed:
            return
        self._cancel_result_timer()
        if self._cancel_on_timeout:
            goal_handle.cancel_goal_async()
        self._publish_failure(stage, 'result_timeout')

    def _result_callback(self, stage: str, future) -> None:
        """Advance after a successful result or terminate on failure."""
        if self._completed:
            return
        self._cancel_result_timer()
        try:
            wrapped_result = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.get_logger().error(
                f'Failed to receive {stage} controller result: {error}'
            )
            self._publish_failure(stage, 'action_result_failed')
            return

        action_succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped_result.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not action_succeeded:
            self.get_logger().warning(
                f'Controller result failed for stage={stage}: '
                f'action_status={wrapped_result.status}, '
                f'error_code={wrapped_result.result.error_code}, '
                f"error_string='{wrapped_result.result.error_string}'"
            )
            self._publish_failure(stage, 'action_result_failed')
            return

        stage_duration_ms = (time.monotonic() - self._stage_start) * 1000.0
        self._publish_stage_status(
            f'event=success;mode=ros2_control;stage={stage};'
            f'duration_ms={stage_duration_ms:.6f};real_hardware=false'
        )

        if stage == 'pre_grasp':
            self._send_stage('assembly', self._assembly_trajectory)
            return

        duration_ms = (time.monotonic() - self._sequence_start) * 1000.0
        status = (
            'event=success;mode=ros2_control;stage_count=2;'
            f'duration_ms={duration_ms:.6f};execution=true;'
            'simulated_execution_only=true;real_hardware=false'
        )
        self._publish_final_result(True, status, duration_ms)
        self.get_logger().info(
            'Simulator-only ros2_control sequence completed successfully.'
        )

    def _publish_skipped(self, reason: str) -> None:
        """Publish a deterministic non-crashing skipped result."""
        duration_ms = (time.monotonic() - self._sequence_start) * 1000.0
        status = (
            f'event=skipped;mode=ros2_control;reason={reason};'
            f'controller={self._controller_action_name};execution=false;'
            'simulated_execution_only=true;real_hardware=false'
        )
        self._publish_final_result(False, status, duration_ms)
        self.get_logger().warning(
            f'ros2_control sequence skipped: reason={reason}, '
            f"controller='{self._controller_action_name}'."
        )

    def _publish_failure(self, stage: str, reason: str) -> None:
        """Publish a deterministic terminal failure result."""
        duration_ms = (time.monotonic() - self._sequence_start) * 1000.0
        status = (
            f'event=failure;mode=ros2_control;stage={stage};reason={reason};'
            'execution=false;simulated_execution_only=true;'
            'real_hardware=false'
        )
        self._publish_final_result(False, status, duration_ms)
        self.get_logger().warning(
            f'ros2_control sequence failed: stage={stage}, reason={reason}.'
        )

    def _publish_stage_status(self, status: str) -> None:
        """Publish one non-retained stage event."""
        message = String()
        message.data = status
        self._stage_status_publisher.publish(message)

    def _publish_final_result(
        self, success: bool, status: str, duration_ms: float
    ) -> None:
        """Publish the retained terminal result exactly once."""
        if self._completed:
            return
        self._cancel_result_timer()
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

    def _cancel_result_timer(self) -> None:
        """Cancel and release the active result watchdog, if any."""
        if self._result_timer is None:
            return
        self._result_timer.cancel()
        self.destroy_timer(self._result_timer)
        self._result_timer = None


def main(args=None) -> None:
    """Run the simulator-only ros2_control sequence executor."""
    rclpy.init(args=args)
    node = Ros2ControlSequenceExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
