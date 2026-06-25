#!/usr/bin/env python3
"""Compare multiple planning diagnostics benchmark CSV files."""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
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


@dataclass
class BenchmarkInput:
    """Resolved benchmark CSV input."""

    label: str
    path: Path


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
    parser.add_argument(
        '--output-markdown',
        help='Optional path for a Markdown benchmark comparison report.',
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


def _markdown_row(values: Sequence[str]) -> str:
    return '| ' + ' | '.join(values) + ' |'


def _planner_id_values(rows: Sequence[dict]) -> List[str]:
    if not rows or 'planner_id' not in rows[0]:
        return []

    values = sorted({
        row.get('planner_id', '').strip() or '<default>'
        for row in rows
        if row.get('planner_id', '').strip() or 'planner_id' in row
    })
    return values


def _write_markdown_report(
    output_path: Path,
    inputs: Sequence[BenchmarkInput],
    summaries: Sequence[BenchmarkSummary],
    rows_by_label: dict[str, List[dict]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

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

    lines = [
        '# Planning Benchmark Comparison',
        '',
        f'Generated: {datetime.now(timezone.utc).isoformat()}',
        '',
        '## Inputs',
        '',
    ]
    lines.extend(
        f'- `{benchmark_input.label}`: `{benchmark_input.path}`'
        for benchmark_input in inputs
    )
    lines.extend([
        '',
        '## Metrics',
        '',
        _markdown_row(headers),
        _markdown_row(['---'] * len(headers)),
    ])

    for summary in summaries:
        lines.append(_markdown_row([
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
        ]))

    planner_lines = []
    for benchmark_input in inputs:
        planner_values = _planner_id_values(rows_by_label[benchmark_input.label])
        if planner_values:
            planner_lines.append(
                f'- `{benchmark_input.label}`: '
                + ', '.join(f'`{value}`' for value in planner_values)
            )

    if planner_lines:
        lines.extend([
            '',
            '## Planner metadata',
            '',
            'Unique `planner_id` values by input:',
            '',
        ])
        lines.extend(planner_lines)

    lines.extend([
        '',
        '## Notes',
        '',
        '- `skipped_small_motion` is not counted as planning failure.',
        '- Trajectories are not executed.',
        '',
    ])

    output_path.write_text('\n'.join(lines), encoding='utf-8')


def main() -> int:
    """Compare one or more planning diagnostics CSV files."""
    args = _parse_args()
    try:
        inputs = []
        summaries = []
        rows_by_label = {}
        for input_value in args.input:
            label, input_path = _parse_input(input_value)
            benchmark_input = BenchmarkInput(label=label, path=input_path)
            inputs.append(benchmark_input)
            rows = _read_rows(input_path)
            rows_by_label[label] = rows
            summaries.append(_summarize(label, rows))
        _print_table(summaries)
        if args.output_markdown:
            output_path = Path(args.output_markdown)
            _write_markdown_report(output_path, inputs, summaries, rows_by_label)
            print(f'Wrote Markdown report: {output_path}')
    except RuntimeError as error:
        print(f'FAIL: {error}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
