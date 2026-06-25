# Dynamic target PlanningScene object

PR14 adds a dynamic target collision object for the Panda MoveIt2 planning
demo. The static PlanningScene already contains table/support objects; this
node represents the moving target object itself as a collision object.

This improves planning-scene realism before trajectory execution is added. The
system remains plan-only.

```text
/pre_grasp_pose
     │
     ▼
panda_pre_grasp_pose_adapter_node
     │
     ▼
/panda_pre_grasp_pose
     │
     ├── pre_grasp_planning_node
     │
     └── dynamic_target_scene_node
               │
               ├── target_object_dynamic collision object
               ├── /dynamic_target_scene_ready
               └── /dynamic_target_scene_status
```

## Default collision pose

The dynamic target scene node subscribes to `/panda_pre_grasp_pose`.

Default behavior:

- frame: incoming pose frame, normally `panda_link0`
- x/y: copied from the adapted pre-grasp pose
- z: adapted pre-grasp z plus `z_offset`, default `-0.20`
- orientation: identity quaternion
- object id: `target_object_dynamic`
- box dimensions: `0.05 x 0.05 x 0.04`
- minimum update distance: `0.02` m

The node publishes:

- `/dynamic_target_scene_ready`: `std_msgs/msg/Bool`, `true` after the first
  successful collision object update
- `/dynamic_target_scene_status`: `std_msgs/msg/String`, semicolon-separated
  key-value event status

The node also provides `/clear_dynamic_target_scene` as a
`std_srvs/srv/Trigger` service. Calling this service removes
`target_object_dynamic`, resets internal state, publishes
`/dynamic_target_scene_ready` as `false`, and emits a `cleared` or
`clear_failed` status event. This is useful before repeated demos or benchmark
comparisons.

The static table/support objects are reset independently through
`/clear_static_planning_scene` and `/reapply_static_planning_scene`. This keeps
static workcell hygiene separate from the dynamic target object.

For repeated demos or A/B benchmark recording, use the unified reset workflow:

```bash
bash scripts/reset_planning_scene_once.sh
```

The workflow clears the dynamic target first, then clears and reapplies the
static objects. If `/panda_pre_grasp_pose` continues publishing, the dynamic
target may be recreated automatically on the next adapted pose. See
[planning_scene_reset_workflow.md](planning_scene_reset_workflow.md).

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
bash scripts/check_dynamic_target_scene_available.sh
bash scripts/check_dynamic_target_scene_ready.sh
python3 scripts/check_dynamic_target_scene_status.py
bash scripts/check_dynamic_target_clear_service.sh
bash scripts/clear_dynamic_target_scene_once.sh
ros2 topic echo /dynamic_target_scene_status
```

## Launch toggle

PR15 makes this node optional from the Panda planning demo launch:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py use_dynamic_target_scene:=false
```

The default is `use_dynamic_target_scene:=true`, preserving existing behavior.
Disabling the node is useful for A/B benchmark comparisons. See
[dynamic_target_ab_benchmark.md](dynamic_target_ab_benchmark.md).

This PR is deliberately limited:

- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
- dynamic target object updates are applied to the PlanningScene only

## Next PR

A future PR can compare benchmarks with and without the dynamic target object,
or add richer PlanningScene diagnostics.
