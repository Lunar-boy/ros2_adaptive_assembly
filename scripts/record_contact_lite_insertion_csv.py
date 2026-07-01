#!/usr/bin/env python3
"""Record contact-lite insertion status events to CSV."""

import argparse
import csv
from pathlib import Path
import sys
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CSV_COLUMNS = (
    'trial_id',
    'timestamp_sec',
    'success',
    'position_error_mm',
    'orientation_error_deg',
    'position_tolerance_mm',
    'orientation_tolerance_deg',
    'execution_required',
    'execution_success',
    'achieved_pose_source',
    'status',
)


def _parse_status(status: str) -> Dict[str, str]:
    fields: Dict[str, str] = {}
    for item in status.split(';'):
        if '=' not in item:
            continue
        key, value = item.split('=', 1)
        fields[key] = value
    return fields


class ContactLiteInsertionCsvRecorder(Node):
    """Subscribe to insertion status events and write rows to CSV."""

    def __init__(
        self,
        *,
        status_topic: str,
        csv_writer: csv.DictWriter,
        max_trials: int,
    ) -> None:
        """Create the recorder subscription."""
        super().__init__('contact_lite_insertion_csv_recorder')
        self._csv_writer = csv_writer
        self._max_trials = max_trials
        self.trial_count = 0
        self.create_subscription(
            String,
            status_topic,
            self._status_callback,
            10,
        )

    def _status_callback(self, message: String) -> None:
        fields = _parse_status(message.data)
        self.trial_count += 1
        row = {
            'trial_id': str(self.trial_count),
            'timestamp_sec': f'{time.time():.6f}',
            'status': message.data,
        }
        for column in CSV_COLUMNS:
            if column in {'trial_id', 'timestamp_sec', 'status'}:
                continue
            row[column] = fields.get(column, '')
        self._csv_writer.writerow(row)
        print(
            f'Recorded insertion trial {self.trial_count}/{self._max_trials}: '
            f'success={row["success"] or "<missing>"}, '
            f'position_error_mm={row["position_error_mm"] or "<missing>"}, '
            'orientation_error_deg='
            f'{row["orientation_error_deg"] or "<missing>"}',
            flush=True,
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Record /assembly_insertion_status events to CSV.'
    )
    parser.add_argument(
        '--output',
        default='benchmark_results/contact_lite_insertion.csv',
        help='Output CSV path.',
    )
    parser.add_argument(
        '--max-trials',
        type=int,
        default=20,
        help='Maximum number of insertion status events to record.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=120.0,
        help='Maximum recording duration in seconds.',
    )
    parser.add_argument(
        '--status-topic',
        default='/assembly_insertion_status',
        help='Insertion status topic to record.',
    )
    return parser.parse_args()


def main() -> int:
    """Record insertion status events until max-trials or timeout."""
    args = _parse_args()
    if args.max_trials <= 0:
        print('FAIL: --max-trials must be greater than zero')
        return 1
    if args.timeout_sec <= 0.0:
        print('FAIL: --timeout-sec must be greater than zero')
        return 1

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f'Recording contact-lite insertion status from {args.status_topic}')
    print(f'Output CSV: {output_path}')
    print(f'Max trials: {args.max_trials}')
    print(f'Timeout: {args.timeout_sec:.1f} sec')

    rclpy.init()
    try:
        with output_path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            recorder = ContactLiteInsertionCsvRecorder(
                status_topic=args.status_topic,
                csv_writer=writer,
                max_trials=args.max_trials,
            )
            deadline = time.monotonic() + args.timeout_sec
            try:
                while (
                    rclpy.ok()
                    and recorder.trial_count < args.max_trials
                    and time.monotonic() < deadline
                ):
                    rclpy.spin_once(recorder, timeout_sec=0.2)
            finally:
                recorder.destroy_node()

        if recorder.trial_count == 0:
            print('FAIL: no insertion status events were recorded')
            print(
                'Start adaptive_assembly_contact_lite_insertion_benchmark.'
                'launch.py before running this recorder.'
            )
            return 1

        print(
            f'PASS: recorded {recorder.trial_count} insertion trial(s) '
            f'to {output_path}'
        )
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
