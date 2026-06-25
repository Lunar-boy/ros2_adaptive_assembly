#!/usr/bin/env python3
"""Validate one dynamic target PlanningScene status event."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUIRED_KEYS = {
    'event',
    'object_id',
    'frame',
    'x',
    'y',
    'z',
    'distance_from_last_update',
    'min_update_distance',
    'ready',
}
VALID_EVENTS = {
    'updated',
    'skipped_small_motion',
    'failed',
    'skipped_empty_frame',
    'cleared',
    'clear_failed',
}
NORMAL_EVENTS = {
    'updated',
    'skipped_small_motion',
    'failed',
    'cleared',
    'clear_failed',
}


class DynamicTargetSceneStatusChecker(Node):
    """Subscribe once to the dynamic target scene status topic."""

    def __init__(self, topic: str) -> None:
        super().__init__('dynamic_target_scene_status_checker')
        self.message = None
        self.subscription = self.create_subscription(
            String,
            topic,
            self._callback,
            10,
        )

    def _callback(self, message: String) -> None:
        self.message = message.data


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Validate one /dynamic_target_scene_status event.'
    )
    parser.add_argument(
        '--topic',
        default='/dynamic_target_scene_status',
        help='Status topic to subscribe to.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=20.0,
        help='Maximum time to wait for one status message.',
    )
    return parser.parse_args()


def parse_status(status: str) -> dict:
    """Parse a semicolon-separated key-value status string."""
    parsed = {}
    for part in status.split(';'):
        if '=' not in part:
            raise RuntimeError(f"status field is missing '=': {part}")
        key, value = part.split('=', 1)
        parsed[key] = value
    return parsed


def require_float(fields: dict, key: str) -> None:
    """Require that one parsed field is a float."""
    try:
        float(fields[key])
    except ValueError as error:
        raise RuntimeError(f"{key} is not parseable as float: {fields[key]}") from error


def validate_status(status: str) -> None:
    """Validate status keys and values."""
    fields = parse_status(status)
    missing_keys = sorted(REQUIRED_KEYS - set(fields))
    if missing_keys:
        raise RuntimeError('missing required keys: ' + ', '.join(missing_keys))

    event = fields['event']
    if event not in VALID_EVENTS:
        raise RuntimeError(f'invalid event: {event}')
    if not fields['object_id']:
        raise RuntimeError('object_id is empty')
    if event != 'skipped_empty_frame' and not fields['frame']:
        raise RuntimeError('frame is empty for normal update/skip event')
    if event in NORMAL_EVENTS:
        for key in ('x', 'y', 'z'):
            require_float(fields, key)

    require_float(fields, 'min_update_distance')

    if fields['ready'] not in {'true', 'false'}:
        raise RuntimeError(f"ready must be true or false, got: {fields['ready']}")


def main() -> int:
    """Wait for and validate one dynamic target scene status message."""
    args = parse_args()
    rclpy.init()
    node = DynamicTargetSceneStatusChecker(args.topic)

    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.message is None:
            print(
                f'FAIL: timed out after {args.timeout_sec:.1f}s waiting for '
                f'{args.topic}'
            )
            return 1

        validate_status(node.message)
        print(f'PASS: dynamic target scene status is valid: {node.message}')
        return 0
    except RuntimeError as error:
        print(f'FAIL: {error}')
        if node.message is not None:
            print(f'Raw status: {node.message}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
