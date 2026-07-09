# Manual run logging

`scripts/run_full_physical_pick_place_with_logs.sh` is a small wrapper for
manual full physical pick-place simulation attempts. It runs the existing
simulator-only full physical pick-place launch and saves the terminal output to
a timestamped run directory so each attempt leaves a persistent log.

This is the first manual logging step. Later PRs can add structured topic
recording and richer run artifacts without changing this basic workflow.

## Basic usage

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
bash scripts/run_full_physical_pick_place_with_logs.sh
```

By default, the script creates:

```text
runs/<timestamp>/
├── metadata.env
└── launch.log
```

The timestamp comes from `RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}` and the
default run directory is `RUN_DIR=${RUN_DIR:-runs/$RUN_ID}`.

## Override the run ID or directory

Use `RUN_ID` to choose a stable run name while keeping the default `runs/`
layout:

```bash
RUN_ID=test_run_001 bash scripts/run_full_physical_pick_place_with_logs.sh
```

Use `RUN_DIR` to write logs somewhere else:

```bash
RUN_DIR=runs/contact_test/manual_001 bash scripts/run_full_physical_pick_place_with_logs.sh
```

## Pass launch arguments

Arguments after the script name are forwarded to:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_physical_pick_place_demo.launch.py
```

For example:

```bash
RUN_ID=test_run_001 bash scripts/run_full_physical_pick_place_with_logs.sh enable_arm_collisions:=true
```

## Created files

`metadata.env` records basic run metadata, including the run ID, run directory,
launch package, launch file, current working directory, start timestamp, and
forwarded launch arguments.

`launch.log` contains stdout and stderr from the launch process. The script uses
`tee`, so output remains visible in the terminal while it is saved. The script
also preserves the real exit code from `ros2 launch`.

## Current limits

This PR intentionally does not add:

- structured topic recording
- rosbag capture
- `summary.md`
- automatic failure classification
- RViz or Gazebo screenshots
- launch integration

The full physical pick-place launch behavior is unchanged. This wrapper only
adds manual terminal-output logging for simulator-only runs.
