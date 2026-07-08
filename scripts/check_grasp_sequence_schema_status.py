#!/usr/bin/env python3
"""Validate grasp candidate and sequence status schemas without simulation."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_fields(value):
    """Parse semicolon-delimited key=value fields."""
    return dict(field.split('=', 1) for field in value.split(';') if '=' in field)


class SchemaValidator(Node):
    """Wait for and validate both explicit schema messages."""

    def __init__(self):
        super().__init__('grasp_sequence_schema_validator')
        self.valid_candidates = False
        self.valid_status = False
        self.create_subscription(String, '/grasp_candidates', self.candidates, 10)
        self.create_subscription(String, '/grasp_sequence_status', self.status, 10)

    def candidates(self, message):
        fields = parse_fields(message.data)
        required = {'event', 'count', 'selected_index', 'frame_id', 'candidates'}
        self.valid_candidates = required <= fields.keys() and fields['event'] == 'grasp_candidates'

    def status(self, message):
        fields = parse_fields(message.data)
        required = {
            'event', 'status', 'selected_index', 'candidate_count', 'frame_id',
            'target_x', 'target_y', 'target_z', 'selected_x', 'selected_y',
            'selected_z', 'lift_z', 'object_place_x', 'object_place_y',
            'object_place_z', 'assembly_pose_mode', 'execution',
            'simulated_only', 'real_hardware',
        }
        self.valid_status = (
            required <= fields.keys() and fields['event'] == 'grasp_sequence'
            and fields['execution'] == 'false'
            and fields['simulated_only'] == 'true'
            and fields['real_hardware'] == 'false')


def main():
    rclpy.init()
    node = SchemaValidator()
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not (
            node.valid_candidates and node.valid_status):
        rclpy.spin_once(node, timeout_sec=0.2)
    success = node.valid_candidates and node.valid_status
    node.destroy_node()
    rclpy.shutdown()
    print('PASS: grasp sequence schemas are valid' if success else
          'FAIL: timed out waiting for valid grasp sequence schemas')
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
