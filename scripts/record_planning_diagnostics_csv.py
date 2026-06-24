#!/usr/bin/env python3
"""Record planning diagnostic status events to CSV."""

import argparse
import csv
from pathlib import Path
import sys
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REQUIRED_FIELDS = (
    'event',
    'frame',
    'x',
    'y',
    'z',
    'distance_from_last_plan',
    'min_replan_distance',
    'duration_ms',
    'execution',
)
CSV_COLUMNS = (
    'wall_time_sec',
    'event',
    'frame',
    'x',
    'y',
    'z',
    'distance_from_last_plan',
    'min_replan_distance',
    'duration_ms',
    'execution',
    'raw_status',
)


def _parse_status(status: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        fields[key] = value
    return fields


class PlanningDiagnosticsCsvRecorder(Node):
    """Subscribe to planning status events and write them to CSV."""

    def __init__(
        self,
        *,
        status_topic: str,
        csv_writer: csv.DictWriter,
        max_events: int,
    ) -> None:
        """Create the recorder node."""
        super().__init__('planning_diagnostics_csv_recorder')
        self._csv_writer = csv_writer
        self._max_events = max_events
        self.event_count = 0
        self.create_subscription(
            String,
            status_topic,
            self._status_callback,
            10,
        )

    def _status_callback(self, message: String) -> None:
        fields = _parse_status(message.data)
        missing_fields = [
            field for field in REQUIRED_FIELDS
            if field not in fields
        ]
        if missing_fields:
            self.get_logger().warn(
                'status event is missing expected field(s): '
                f'{", ".join(missing_fields)}'
            )

        row = {
            'wall_time_sec': f'{time.time():.6f}',
            'raw_status': message.data,
        }
        for field in REQUIRED_FIELDS:
            row[field] = fields.get(field, '')

        self._csv_writer.writerow(row)
        self.event_count += 1

        print(
            'Recorded event '
            f'{self.event_count}/{self._max_events}: '
            f'event={row["event"] or "<missing>"}, '
            f'duration_ms={row["duration_ms"] or "<missing>"}',
            flush=True,
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Record /pre_grasp_planning_status events to CSV.'
    )
    parser.add_argument(
        '--output',
        default='benchmark_results/planning_diagnostics.csv',
        help='Output CSV path.',
    )
    parser.add_argument(
        '--max-events',
        type=int,
        default=20,
        help='Maximum number of events to record.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=120.0,
        help='Maximum recording duration in seconds.',
    )
    parser.add_argument(
        '--status-topic',
        default='/pre_grasp_planning_status',
        help='Planning status topic to record.',
    )
    return parser.parse_args()


def main() -> int:
    """Record planning status events until max-events or timeout."""
    args = _parse_args()
    if args.max_events <= 0:
        print('FAIL: --max-events must be greater than zero')
        return 1
    if args.timeout_sec <= 0.0:
        print('FAIL: --timeout-sec must be greater than zero')
        return 1

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f'Recording planning diagnostics from {args.status_topic}')
    print(f'Output CSV: {output_path}')
    print(f'Max events: {args.max_events}')
    print(f'Timeout: {args.timeout_sec:.1f} sec')

    rclpy.init()
    try:
        with output_path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            recorder = PlanningDiagnosticsCsvRecorder(
                status_topic=args.status_topic,
                csv_writer=writer,
                max_events=args.max_events,
            )

            deadline = time.monotonic() + args.timeout_sec
            try:
                while (
                    rclpy.ok()
                    and recorder.event_count < args.max_events
                    and time.monotonic() < deadline
                ):
                    rclpy.spin_once(recorder, timeout_sec=0.2)
            finally:
                recorder.destroy_node()

        if recorder.event_count == 0:
            print('FAIL: no planning diagnostic events were recorded')
            print(
                'Start the Panda planning demo first with: ros2 launch '
                'adaptive_assembly_bringup '
                'adaptive_assembly_panda_planning_demo.launch.py'
            )
            return 1

        print(
            f'PASS: recorded {recorder.event_count} planning diagnostic '
            f'event(s) to {output_path}'
        )
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
