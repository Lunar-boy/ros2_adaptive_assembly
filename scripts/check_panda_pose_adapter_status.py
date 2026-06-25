#!/usr/bin/env python3
"""Validate one Panda pose adapter status event."""

import sys
import time

import rclpy
from std_msgs.msg import String


REQUIRED_KEYS = {
    "event",
    "input_frame",
    "output_frame",
    "use_tf_transform",
    "x",
    "y",
    "z",
    "fixed_orientation",
}

VALID_EVENTS = {
    "adapted",
    "tf_lookup_failed",
    "skipped_empty_frame",
}


def parse_status(status_text):
    """Parse semicolon-separated key-value status text."""
    fields = {}
    for item in status_text.split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key] = value
    return fields


def main():
    """Subscribe once and validate the adapter status fields."""
    rclpy.init()
    node = rclpy.create_node("check_panda_pose_adapter_status")
    received = {"message": None}

    def callback(message):
        received["message"] = message

    subscription = node.create_subscription(
        String,
        "/panda_pose_adapter_status",
        callback,
        10,
    )
    _ = subscription

    deadline = time.monotonic() + 20.0
    while rclpy.ok() and received["message"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if received["message"] is None:
        print("FAIL: timed out waiting for /panda_pose_adapter_status")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    status_text = received["message"].data
    fields = parse_status(status_text)
    missing = sorted(REQUIRED_KEYS - fields.keys())
    if missing:
        print(f"FAIL: missing required status keys: {missing}")
        print(f"Raw status: {status_text}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    event = fields["event"]
    if event not in VALID_EVENTS:
        print(f"FAIL: invalid event '{event}'")
        print(f"Raw status: {status_text}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if fields["use_tf_transform"] not in {"true", "false"}:
        print(f"FAIL: use_tf_transform is not boolean text: {fields['use_tf_transform']}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if fields["fixed_orientation"] not in {"true", "false"}:
        print(f"FAIL: fixed_orientation is not boolean text: {fields['fixed_orientation']}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    for key in ("x", "y", "z"):
        try:
            float(fields[key])
        except ValueError:
            print(f"FAIL: {key} is not parseable as float: {fields[key]}")
            node.destroy_node()
            rclpy.shutdown()
            return 1

    print("PASS: /panda_pose_adapter_status has expected fields")
    print(f"PASS: event={event}, use_tf_transform={fields['use_tf_transform']}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
