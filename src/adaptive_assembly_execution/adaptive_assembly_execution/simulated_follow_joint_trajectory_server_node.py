"""Provide a deterministic simulator-only trajectory action fixture."""

import time

from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class SimulatedFollowJointTrajectoryServerNode(Node):
    """Return configurable deterministic FollowJointTrajectory results."""

    def __init__(self) -> None:
        """Declare fixture behavior and create the action server."""
        super().__init__('simulated_follow_joint_trajectory_server_node')
        self.declare_parameter(
            'action_name', '/panda_arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('result_mode', 'success')
        self.declare_parameter('result_delay_sec', 0.1)
        self.declare_parameter('require_non_empty_trajectory', True)
        self.declare_parameter('expected_joint_prefix', 'panda_joint')

        self._action_name = self.get_parameter('action_name').value
        self._result_mode = self.get_parameter('result_mode').value
        self._result_delay_sec = self.get_parameter(
            'result_delay_sec'
        ).value
        self._require_non_empty = self.get_parameter(
            'require_non_empty_trajectory'
        ).value
        self._expected_joint_prefix = self.get_parameter(
            'expected_joint_prefix'
        ).value
        self._validate_parameters()

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            'Deterministic simulated FollowJointTrajectory server ready: '
            f"action='{self._action_name}', mode={self._result_mode}, "
            f'result_delay_sec={self._result_delay_sec}, '
            'simulated_execution_only=true, real_hardware=false.'
        )

    def _validate_parameters(self) -> None:
        if self._result_mode not in {'success', 'reject', 'fail', 'timeout'}:
            raise ValueError(
                'result_mode must be success, reject, fail, or timeout'
            )
        if self._result_delay_sec < 0.0:
            raise ValueError('result_delay_sec must not be negative')

    def _goal_callback(self, goal_request) -> GoalResponse:
        trajectory = goal_request.trajectory
        valid = True
        if self._require_non_empty:
            valid = bool(trajectory.joint_names and trajectory.points)
        if self._expected_joint_prefix:
            valid = valid and all(
                name.startswith(self._expected_joint_prefix)
                for name in trajectory.joint_names
            )
        if self._result_mode == 'reject' or not valid:
            self.get_logger().warning('Rejecting simulated trajectory goal.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """Return the configured result while allowing cancellation."""
        if self._result_mode == 'timeout':
            while not goal_handle.is_cancel_requested:
                time.sleep(0.05)
            goal_handle.canceled()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'Canceled after simulated timeout.'
            return result

        time.sleep(self._result_delay_sec)
        result = FollowJointTrajectory.Result()
        if self._result_mode == 'fail':
            goal_handle.abort()
            result.error_code = (
                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            )
            result.error_string = 'Deterministic simulated failure.'
            return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Deterministic simulated success.'
        return result

    def destroy_node(self) -> None:
        """Destroy the action server before its owning node."""
        self._server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """Run the deterministic simulator-only action fixture."""
    rclpy.init(args=args)
    node = SimulatedFollowJointTrajectoryServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
