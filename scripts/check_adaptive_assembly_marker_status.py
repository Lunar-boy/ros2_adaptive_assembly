#!/usr/bin/env python3
"""Validate one adaptive assembly marker status message."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUIRED_KEYS = {"event", "source", "frame", "x", "y", "z"}
VALID_EVENTS = {"marker_updated", "skipped_empty_frame"}


def parse_status(status_text):
    fields = {}
    for part in status_text.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


class MarkerStatusChecker(Node):
    """Subscribe once to the marker status topic."""

    def __init__(self):
        super().__init__("check_adaptive_assembly_marker_status")
        self.message = None
        self.subscription = self.create_subscription(
            String,
            "/adaptive_assembly_marker_status",
            self._callback,
            10,
        )

    def _callback(self, msg):
        self.message = msg.data


def validate(fields):
    missing = REQUIRED_KEYS - fields.keys()
    if missing:
        return False, f"missing required keys: {sorted(missing)}"

    event = fields["event"]
    if event not in VALID_EVENTS:
        return False, f"invalid event '{event}'"

    if not fields["source"]:
        return False, "source is empty"

    if event == "marker_updated" and not fields["frame"]:
        return False, "frame is empty for marker_updated event"

    for key in ("x", "y", "z"):
        try:
            float(fields[key])
        except ValueError:
            return False, f"{key} is not parseable as float: {fields[key]}"

    return True, "marker status format is valid"


def main():
    rclpy.init()
    node = MarkerStatusChecker()
    deadline = time.monotonic() + 20.0

    try:
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.message is None:
            print("FAIL: Timed out waiting for /adaptive_assembly_marker_status")
            return 1

        fields = parse_status(node.message)
        ok, reason = validate(fields)
        if not ok:
            print(f"FAIL: {reason}")
            print(f"Raw status: {node.message}")
            return 1

        print(f"PASS: {reason}")
        print(f"Status: {node.message}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
