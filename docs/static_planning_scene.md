# Static PlanningScene objects

PR9 adds a minimal static PlanningScene node for the Panda MoveIt2 planning
demo. The node injects simple collision objects into MoveIt2 so the plan-only
bridge can plan with basic collision awareness.

This remains plan-only. No trajectories are executed.

PR10 adds planning diagnostics to `pre_grasp_planning_node`. It does not change
the static PlanningScene behavior described here.

PR11 benchmark scripts can be used to measure planning behavior while these
static PlanningScene objects are enabled.

PR14 adds a separate dynamic target collision object node. The static table and
support behavior described here remains unchanged. See
[dynamic_target_scene.md](dynamic_target_scene.md).

PR17 adds reset support for the static objects. The node now exposes services
to clear and reapply `work_table` and `target_support` without restarting the
demo.

PR24 adds a read-only PlanningScene audit node that can verify whether
`work_table`, `target_support`, and `target_object_dynamic` are present in the
MoveIt2 PlanningScene. See [planning_scene_audit.md](planning_scene_audit.md).

Initial objects:

- `work_table`: a broad table/workcell box in frame `panda_link0`
- `target_support`: a small support block near the target area in frame
  `panda_link0`

```text
static_planning_scene_node
     │
     ├── work_table collision object
     ├── target_support collision object
     ├── /planning_scene_objects_ready
     ├── /static_planning_scene_status
     ├── /clear_static_planning_scene
     └── /reapply_static_planning_scene
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
bash scripts/check_static_planning_scene_services.sh
python3 scripts/check_planning_scene_audit_status.py
bash scripts/check_planning_scene_audit_ready.sh
python3 scripts/check_static_planning_scene_status.py
bash scripts/clear_static_planning_scene_once.sh
bash scripts/reapply_static_planning_scene_once.sh
ros2 topic echo --once /planning_scene_objects_ready
ros2 topic echo --once /static_planning_scene_status
ros2 topic echo /pre_grasp_plan_success
```

## Reset services

`static_planning_scene_node` provides:

- `/clear_static_planning_scene`: removes enabled static collision objects from
  the PlanningScene and publishes `/planning_scene_objects_ready` as `false`
- `/reapply_static_planning_scene`: reapplies enabled static collision objects
  and republishes the ready flag
- `/static_planning_scene_status`: semicolon-separated status events with
  `event`, `object_ids`, `frame`, and `ready`

These services are useful before repeated demos or benchmark comparisons. The
dynamic target object has its own independent reset service documented in
[dynamic_target_scene.md](dynamic_target_scene.md).

For repeated demos or A/B benchmark recording, use the unified reset workflow:

```bash
bash scripts/reset_planning_scene_once.sh
```

It clears the dynamic target, clears static objects, and reapplies static
objects in a fixed order. See
[planning_scene_reset_workflow.md](planning_scene_reset_workflow.md).

The PlanningScene integration is deliberately minimal:

- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no trajectory execution
- collision objects are static and can be cleared/reapplied on request

## Next PR

A future PR can compare benchmark results with different PlanningScene
configurations or add richer scene diagnostics.
