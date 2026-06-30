"""Validate and simulate exported assembly sequence trajectories."""

import time
from typing import Optional, Tuple

from moveit_msgs.msg import RobotTrajectory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float64, String


class DryRunSequenceExecutorNode(Node):
    """Perform a one-shot, message-only dry run of a two-stage sequence."""

    def __init__(self) -> None:
        """Declare parameters and create dry-run ROS interfaces."""
        super().__init__('dry_run_sequence_executor_node')

        self.declare_parameter(
            'pre_grasp_trajectory_topic', '/pre_grasp_trajectory'
        )
        self.declare_parameter(
            'assembly_trajectory_topic', '/assembly_trajectory'
        )
        self.declare_parameter('status_topic', '/assembly_execution_status')
        self.declare_parameter('success_topic', '/assembly_execution_success')
        self.declare_parameter(
            'duration_topic', '/assembly_execution_duration_ms'
        )
        self.declare_parameter(
            'stage_status_topic', '/assembly_execution_stage_status'
        )
        self.declare_parameter('require_panda_joints', True)
        self.declare_parameter('expected_joint_prefix', 'panda_joint')
        self.declare_parameter('simulate_real_time', False)
        self.declare_parameter('simulated_stage_duration_ms', 10.0)
        self.declare_parameter('publish_stage_status', True)

        self._pre_grasp_topic = self.get_parameter(
            'pre_grasp_trajectory_topic'
        ).value
        self._assembly_topic = self.get_parameter(
            'assembly_trajectory_topic'
        ).value
        self._status_topic = self.get_parameter('status_topic').value
        self._success_topic = self.get_parameter('success_topic').value
        self._duration_topic = self.get_parameter('duration_topic').value
        self._stage_status_topic = self.get_parameter(
            'stage_status_topic'
        ).value
        self._require_panda_joints = self.get_parameter(
            'require_panda_joints'
        ).value
        self._expected_joint_prefix = self.get_parameter(
            'expected_joint_prefix'
        ).value
        self._simulate_real_time = self.get_parameter(
            'simulate_real_time'
        ).value
        self._simulated_stage_duration_ms = self.get_parameter(
            'simulated_stage_duration_ms'
        ).value
        self._publish_stage_status = self.get_parameter(
            'publish_stage_status'
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

        self._pre_grasp_trajectory: Optional[RobotTrajectory] = None
        self._assembly_trajectory: Optional[RobotTrajectory] = None
        self._completed = False

        self.get_logger().info(
            'Dry-run sequence executor ready: '
            f"pre_grasp_topic='{self._pre_grasp_topic}', "
            f"assembly_topic='{self._assembly_topic}', "
            f"status_topic='{self._status_topic}', "
            f'require_panda_joints={self._require_panda_joints}, '
            f"expected_joint_prefix='{self._expected_joint_prefix}', "
            f'simulate_real_time={self._simulate_real_time}, '
            'real_execution=false. No robot commands will be sent.'
        )

    def _validate_parameters(self) -> None:
        """Reject settings that cannot produce a valid dry run."""
        if self._simulated_stage_duration_ms < 0.0:
            raise ValueError(
                'simulated_stage_duration_ms must be greater than or equal '
                'to zero'
            )
        if self._require_panda_joints and not self._expected_joint_prefix:
            raise ValueError(
                'expected_joint_prefix must not be empty when '
                'require_panda_joints is true'
            )

    def _pre_grasp_callback(self, trajectory: RobotTrajectory) -> None:
        """Store the first pre-grasp trajectory and try the dry run."""
        if self._completed or self._pre_grasp_trajectory is not None:
            return
        self._pre_grasp_trajectory = trajectory
        self._try_dry_run()

    def _assembly_callback(self, trajectory: RobotTrajectory) -> None:
        """Store the first assembly trajectory and try the dry run."""
        if self._completed or self._assembly_trajectory is not None:
            return
        self._assembly_trajectory = trajectory
        self._try_dry_run()

    def _try_dry_run(self) -> None:
        """Validate and simulate both stages once both trajectories exist."""
        if self._completed:
            return
        if self._pre_grasp_trajectory is None:
            return
        if self._assembly_trajectory is None:
            return

        self._completed = True
        start = time.monotonic()

        pre_grasp_valid, reason = self._validate_trajectory(
            'pre_grasp', self._pre_grasp_trajectory
        )
        if not pre_grasp_valid:
            self._publish_failure(reason, 'pre_grasp', start)
            return

        assembly_valid, reason = self._validate_trajectory(
            'assembly', self._assembly_trajectory
        )
        if not assembly_valid:
            self._publish_failure(reason, 'assembly', start)
            return

        try:
            self._simulate_stage('pre_grasp', self._pre_grasp_trajectory)
            self._simulate_stage('assembly', self._assembly_trajectory)
        except Exception as error:  # pragma: no cover - defensive guard
            self.get_logger().error(
                f'Dry-run stage simulation failed: {error}'
            )
            self._publish_failure('stage_simulation_failed', '', start)
            return

        duration_ms = (time.monotonic() - start) * 1000.0
        status = (
            'event=success;mode=dry_run;pre_grasp_received=true;'
            'assembly_received=true;pre_grasp_valid=true;assembly_valid=true;'
            f'stage_count=2;duration_ms={duration_ms:.6f};execution=true;'
            'real_execution=false'
        )
        self._publish_final_result(True, status, duration_ms)
        self.get_logger().info(
            'Dry-run assembly sequence succeeded for 2 stages. '
            'No real trajectory execution was attempted.'
        )

    def _validate_trajectory(
        self, stage: str, trajectory: RobotTrajectory
    ) -> Tuple[bool, str]:
        """Validate the joint names and points for one exported trajectory."""
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.joint_names:
            return False, f'empty_{stage}_joint_names'
        if not joint_trajectory.points:
            return False, f'empty_{stage}_trajectory'
        if self._require_panda_joints and not any(
            name.startswith(self._expected_joint_prefix)
            for name in joint_trajectory.joint_names
        ):
            return False, 'missing_expected_joints'
        return True, ''

    def _simulate_stage(
        self, stage: str, trajectory: RobotTrajectory
    ) -> None:
        """Simulate one stage without invoking any execution interface."""
        start = time.monotonic()
        if self._simulate_real_time:
            time.sleep(self._simulated_stage_duration_ms / 1000.0)
        duration_ms = (time.monotonic() - start) * 1000.0

        if self._publish_stage_status:
            message = String()
            message.data = (
                f'event=success;mode=dry_run;stage={stage};'
                f'point_count={len(trajectory.joint_trajectory.points)};'
                f'joint_count={len(trajectory.joint_trajectory.joint_names)};'
                f'duration_ms={duration_ms:.6f};execution=true;'
                'real_execution=false'
            )
            self._stage_status_publisher.publish(message)

    def _publish_failure(
        self, reason: str, stage: str, start_time: float
    ) -> None:
        """Publish one deterministic aggregate dry-run failure."""
        duration_ms = (time.monotonic() - start_time) * 1000.0
        status = f'event=failure;mode=dry_run;reason={reason}'
        if stage:
            status += f';stage={stage}'
        status += ';execution=false;real_execution=false'
        self._publish_final_result(False, status, duration_ms)
        self.get_logger().warning(
            f'Dry-run sequence failed: reason={reason}, stage={stage}. '
            'No real trajectory execution was attempted.'
        )

    def _publish_final_result(
        self, success: bool, status: str, duration_ms: float
    ) -> None:
        """Publish retained aggregate result messages."""
        status_message = String()
        status_message.data = status
        self._status_publisher.publish(status_message)

        success_message = Bool()
        success_message.data = success
        self._success_publisher.publish(success_message)

        duration_message = Float64()
        duration_message.data = duration_ms
        self._duration_publisher.publish(duration_message)


def main(args=None) -> None:
    """Run the dry-run sequence executor node."""
    rclpy.init(args=args)
    node = DryRunSequenceExecutorNode()
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
