# Benchmark report export

PR19 adds Markdown report export to the planning benchmark CSV comparison
tool. This makes benchmark comparisons easier to save, review, and attach to
portfolio notes or PR summaries.

The report reuses the same metrics printed by
`scripts/compare_planning_benchmark_csvs.py`:

- total events
- success/failure/skipped counts
- planning attempts
- planning success rate over attempts
- overall success fraction
- planning duration statistics

Example:

```bash
python3 scripts/compare_planning_benchmark_csvs.py \
  --input no_dynamic=benchmark_results/no_dynamic_target.csv \
  --input with_dynamic=benchmark_results/with_dynamic_target.csv \
  --output-markdown benchmark_results/dynamic_target_ab_report.md
```

Generated reports under `benchmark_results/` are ignored by git along with the
benchmark CSV files.

Validate report export without launching ROS nodes:

```bash
bash scripts/check_benchmark_report_export.sh
```

Notes:

- `skipped_small_motion` is not counted as planning failure.
- trajectories are not executed
- no Gazebo
- no ros2_control integration for this project
- no real hardware
