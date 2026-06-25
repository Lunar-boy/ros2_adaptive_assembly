# Dynamic target A/B benchmark

PR15 adds a launch toggle for the dynamic target PlanningScene object and two
seeded benchmark launch profiles for A/B comparison.

The toggle is:

- `use_dynamic_target_scene:=true`: include `dynamic_target_scene_node`
- `use_dynamic_target_scene:=false`: do not include `dynamic_target_scene_node`

Default behavior remains enabled, so the normal Panda planning demo still
starts the dynamic target scene node unless the toggle is set to `false`.

## Run with the dynamic target disabled

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py use_dynamic_target_scene:=false
```

Seeded benchmark profile:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_no_dynamic_target.launch.py
```

## Run with the dynamic target enabled

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py use_dynamic_target_scene:=true
```

Seeded benchmark profile:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark_with_dynamic_target.launch.py
```

## Record both CSVs

Run one benchmark launch at a time.

Before recording each CSV, you can reset the PlanningScene to a clean baseline:

```bash
bash scripts/reset_planning_scene_once.sh
```

This clears the dynamic target object, clears static objects, and reapplies
static objects. See
[planning_scene_reset_workflow.md](planning_scene_reset_workflow.md).

Without dynamic target:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/no_dynamic_target.csv bash scripts/run_seeded_planning_benchmark.sh
```

With dynamic target:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/with_dynamic_target.csv bash scripts/run_seeded_planning_benchmark.sh
```

## Compare

```bash
python3 scripts/compare_planning_benchmark_csvs.py --input no_dynamic=benchmark_results/no_dynamic_target.csv --input with_dynamic=benchmark_results/with_dynamic_target.csv
```

For a manual command checklist:

```bash
bash scripts/run_dynamic_target_ab_benchmark_hint.sh
```

## Validate launch installation

```bash
bash scripts/check_dynamic_target_toggle_launches.sh
```

This is diagnostics-only:

- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no new planning algorithm
