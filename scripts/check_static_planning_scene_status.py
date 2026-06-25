#!/usr/bin/env python3
"""Validate one static PlanningScene status message."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


VALID_EVENTS = {
    "applied",
    "apply_failed",
    "no_objects_enabled",
    "cleared",
    "clear_failed",
}

REQUIRED_KEYS = {
    "event",
    "object_ids",
    "frame",
    "ready",
}


def parse_status(status: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for field in status.split(";"):
        if "=" not in field:
            continue
        key, value = field.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


class StaticPlanningSceneStatusChecker(Node):
    """Subscribe until one static PlanningScene status message is received."""

    def __init__(self) -> None:
        super().__init__("static_planning_scene_status_checker")
        self.message: String | None = None
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            String,
            "/static_planning_scene_status",
            self._on_status,
            qos_profile,
        )

    def _on_status(self, message: String) -> None:
        self.message = message


def main() -> int:
    timeout_sec = 20.0
    rclpy.init()
    node = StaticPlanningSceneStatusChecker()

    try:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.message is None:
            print("FAIL: timed out waiting for /static_planning_scene_status")
            return 1

        # Transient-local publishers can replay more than one retained status.
        # Drain briefly so validation reports the latest available event.
        drain_deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        raw_status = node.message.data
        parsed = parse_status(raw_status)
        missing_keys = REQUIRED_KEYS - parsed.keys()
        if missing_keys:
            print(f"FAIL: status is missing required keys: {sorted(missing_keys)}")
            print(f"      raw_status: {raw_status}")
            return 1

        event = parsed["event"]
        if event not in VALID_EVENTS:
            print(f"FAIL: unexpected event '{event}'")
            print(f"      raw_status: {raw_status}")
            return 1

        if not parsed["object_ids"]:
            print("FAIL: object_ids field is empty")
            print(f"      raw_status: {raw_status}")
            return 1

        if not parsed["frame"]:
            print("FAIL: frame field is empty")
            print(f"      raw_status: {raw_status}")
            return 1

        if parsed["ready"] not in {"true", "false"}:
            print(f"FAIL: ready must be true or false, got '{parsed['ready']}'")
            print(f"      raw_status: {raw_status}")
            return 1

        print(f"PASS: /static_planning_scene_status is valid: {raw_status}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
