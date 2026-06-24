# Static PlanningScene objects

PR9 adds a minimal static PlanningScene node for the Panda MoveIt2 planning
demo. The node injects simple collision objects into MoveIt2 so the plan-only
bridge can plan with basic collision awareness.

This remains plan-only. No trajectories are executed.

PR10 adds planning diagnostics to `pre_grasp_planning_node`. It does not change
the static PlanningScene behavior described here.

PR11 benchmark scripts can be used to measure planning behavior while these
static PlanningScene objects are enabled.

Initial objects:

- `work_table`: a broad table/workcell box in frame `panda_link0`
- `target_support`: a small support block near the target area in frame
  `panda_link0`

```text
static_planning_scene_node
     │
     ├── work_table collision object
     ├── target_support collision object
     └── /planning_scene_objects_ready
               │
               ▼
Panda MoveIt2 planning scene
               │
               ▼
pre_grasp_planning_node plans with collision awareness
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
bash scripts/check_static_planning_scene_available.sh
bash scripts/check_static_planning_scene_ready.sh
ros2 topic echo --once /planning_scene_objects_ready
ros2 topic echo /pre_grasp_plan_success
```

The PlanningScene integration is deliberately minimal:

- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no trajectory execution
- collision objects are static and applied once

## Next PR

A future PR can add dynamic target collision objects or planning
diagnostics/benchmarking.
