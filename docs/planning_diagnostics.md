# Planning diagnostics

PR10 adds lightweight observability to the plan-only MoveIt2 pre-grasp planning
bridge. The bridge still does not execute trajectories.

The compatibility topic remains available:

- `/pre_grasp_plan_success`: `std_msgs/msg/Bool`

New diagnostic topics:

- `/pre_grasp_planning_status`: `std_msgs/msg/String`
- `/pre_grasp_planning_duration_ms`: `std_msgs/msg/Float64`

The status message is a semicolon-separated key-value string. Example:

```text
event=failure;frame=panda_link0;x=0.450000;y=0.000000;z=0.350000;distance_from_last_plan=none;min_replan_distance=0.030000;duration_ms=1001.234000;execution=false;planner_id=;num_planning_attempts=1;max_velocity_scaling_factor=1.000000;max_acceleration_scaling_factor=1.000000
```

Supported event values:

- `success`: MoveIt2 returned a successful plan
- `failure`: MoveIt2 planning was attempted and failed
- `skipped_small_motion`: planning was skipped because the pose moved less than
  `min_replan_distance`
- `guard_rejected`: an enabled request guard rejected the pose before MoveIt2
  planning

This separates true planning failure from skipped replanning, provides timing
data for future benchmarking, and makes demos easier to debug.

PR20 adds planner setting metadata to each status message:

- `planner_id`
- `num_planning_attempts`
- `max_velocity_scaling_factor`
- `max_acceleration_scaling_factor`

See [planner_settings.md](planner_settings.md).

PR23 adds request guard metadata to each status message:

- `guard_enabled`
- `guard_passed`
- `guard_reason`

See [planning_request_guard.md](planning_request_guard.md).

```text
/panda_pre_grasp_pose
     │
     ▼
pre_grasp_planning_node
     │
     ├── /pre_grasp_plan_success
     ├── /pre_grasp_planning_status
     └── /pre_grasp_planning_duration_ms
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

## Validate

```bash
bash scripts/check_planning_diagnostics.sh
bash scripts/echo_planning_diagnostics_once.sh
python3 scripts/check_planning_status_format.py
bash scripts/check_planner_parameter_status.sh
bash scripts/check_planning_request_guard_status.sh
```

## Benchmark recording

PR11 records these PR10 diagnostic events into CSV and summarizes success,
failure, skipped-event counts, and planning duration statistics. See
[planning_benchmark.md](planning_benchmark.md).

This PR is diagnostics only:

- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no dynamic PlanningScene updates
