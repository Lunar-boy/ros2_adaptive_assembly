#!/usr/bin/env python3
"""Compare multiple planning diagnostics benchmark CSV files."""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import statistics
import sys
from typing import Iterable, List, Sequence, Tuple


REQUIRED_COLUMNS = {'event', 'duration_ms'}
EVENT_SUCCESS = 'success'
EVENT_FAILURE = 'failure'
EVENT_SKIPPED = 'skipped_small_motion'


@dataclass
class BenchmarkSummary:
    """Computed metrics for one benchmark CSV."""

    label: str
    total_events: int
    success_count: int
    failure_count: int
    skipped_count: int
    planning_attempts: int
    planning_success_rate: str
    overall_success_fraction: str
    average_duration_ms: str
    median_duration_ms: str
    min_duration_ms: str
    max_duration_ms: str


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Compare planning diagnostics benchmark CSV files.'
    )
    parser.add_argument(
        '--input',
        action='append',
        required=True,
        help=(
            'Input CSV. Use label=path for explicit labels, or pass a plain '
            'path to use the file stem.'
        ),
    )
    return parser.parse_args()


def _parse_input(input_value: str) -> Tuple[str, Path]:
    if '=' in input_value:
        label, path_text = input_value.split('=', 1)
        if not label:
            raise RuntimeError(f'input label is empty: {input_value}')
        if not path_text:
            raise RuntimeError(f'input path is empty: {input_value}')
        return label, Path(path_text)

    path = Path(input_value)
    return path.stem, path


def _read_rows(input_path: Path) -> List[dict]:
    if not input_path.exists():
        raise RuntimeError(f'input CSV does not exist: {input_path}')

    with input_path.open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise RuntimeError(f'input CSV is empty or missing header: {input_path}')

        missing_columns = sorted(REQUIRED_COLUMNS - set(reader.fieldnames))
        if missing_columns:
            raise RuntimeError(
                f'{input_path} is missing required column(s): '
                + ', '.join(missing_columns)
            )

        rows = list(reader)

    if not rows:
        raise RuntimeError(f'input CSV contains no planning events: {input_path}')
    return rows


def _duration_values(rows: Iterable[dict], label: str) -> List[float]:
    durations = []
    for row in rows:
        event = row.get('event', '')
        if event not in {EVENT_SUCCESS, EVENT_FAILURE}:
            continue
        try:
            durations.append(float(row.get('duration_ms', '')))
        except ValueError as error:
            raise RuntimeError(
                f'{label}: invalid duration_ms for event={event}: '
                f'{row.get("duration_ms", "")}'
            ) from error
    return durations


def _format_fraction(numerator: int, denominator: int) -> str:
    if denominator == 0:
        return 'n/a'
    return f'{numerator / denominator:.3f}'


def _format_duration(value: float) -> str:
    return f'{value:.3f}'


def _summarize(label: str, rows: List[dict]) -> BenchmarkSummary:
    total_events = len(rows)
    success_count = sum(1 for row in rows if row.get('event') == EVENT_SUCCESS)
    failure_count = sum(1 for row in rows if row.get('event') == EVENT_FAILURE)
    skipped_count = sum(1 for row in rows if row.get('event') == EVENT_SKIPPED)
    planning_attempts = success_count + failure_count
    durations = _duration_values(rows, label)

    if durations:
        average_duration_ms = _format_duration(statistics.mean(durations))
        median_duration_ms = _format_duration(statistics.median(durations))
        min_duration_ms = _format_duration(min(durations))
        max_duration_ms = _format_duration(max(durations))
    else:
        average_duration_ms = 'n/a'
        median_duration_ms = 'n/a'
        min_duration_ms = 'n/a'
        max_duration_ms = 'n/a'

    return BenchmarkSummary(
        label=label,
        total_events=total_events,
        success_count=success_count,
        failure_count=failure_count,
        skipped_count=skipped_count,
        planning_attempts=planning_attempts,
        planning_success_rate=_format_fraction(success_count, planning_attempts),
        overall_success_fraction=_format_fraction(success_count, total_events),
        average_duration_ms=average_duration_ms,
        median_duration_ms=median_duration_ms,
        min_duration_ms=min_duration_ms,
        max_duration_ms=max_duration_ms,
    )


def _print_table(summaries: Sequence[BenchmarkSummary]) -> None:
    headers = [
        'profile',
        'total',
        'success',
        'failure',
        'skipped',
        'attempts',
        'plan_success_rate',
        'overall_success',
        'avg_ms',
        'median_ms',
        'min_ms',
        'max_ms',
    ]
    rows = [
        [
            summary.label,
            str(summary.total_events),
            str(summary.success_count),
            str(summary.failure_count),
            str(summary.skipped_count),
            str(summary.planning_attempts),
            summary.planning_success_rate,
            summary.overall_success_fraction,
            summary.average_duration_ms,
            summary.median_duration_ms,
            summary.min_duration_ms,
            summary.max_duration_ms,
        ]
        for summary in summaries
    ]
    widths = [
        max(len(header), *(len(row[index]) for row in rows))
        for index, header in enumerate(headers)
    ]

    print('Planning benchmark comparison')
    print('-----------------------------')
    print('  '.join(
        header.ljust(widths[index])
        for index, header in enumerate(headers)
    ))
    print('  '.join('-' * width for width in widths))
    for row in rows:
        print('  '.join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        ))


def main() -> int:
    """Compare one or more planning diagnostics CSV files."""
    args = _parse_args()
    try:
        summaries = []
        for input_value in args.input:
            label, input_path = _parse_input(input_value)
            summaries.append(_summarize(label, _read_rows(input_path)))
        _print_table(summaries)
    except RuntimeError as error:
        print(f'FAIL: {error}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
