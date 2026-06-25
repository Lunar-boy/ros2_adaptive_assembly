# PlanningScene audit

PR24 adds a read-only PlanningScene audit node. It queries MoveIt2 for expected
collision object ids and reports which objects are present or missing.

The audit node is introspection-only:

- it does not apply collision objects
- it does not remove collision objects
- it does not execute trajectories

PR25 adds optional RViz2 marker visualization for the adaptive assembly pose
topics. The marker node is independent of this audit node: it publishes
`/adaptive_assembly_markers` and does not query or modify the PlanningScene.
See [rviz_markers.md](rviz_markers.md).

## Expected objects

The default expected object ids are:

- `work_table`
- `target_support`
- `target_object_dynamic`

They are configured through the `expected_object_ids` parameter as a
comma-separated string.

## Node and topics

`planning_scene_audit_node` is provided by `adaptive_assembly_planning`.

Default parameters:

- `expected_object_ids: work_table,target_support,target_object_dynamic`
- `audit_period_sec: 2.0`
- `status_topic: /planning_scene_audit_status`
- `ready_topic: /planning_scene_audit_ready`

Published topics:

- `/planning_scene_audit_ready`: `std_msgs/msg/Bool`
- `/planning_scene_audit_status`: `std_msgs/msg/String`

Status fields:

- `event=audit`
- `expected`
- `present`
- `missing`
- `all_present`

Example:

```text
event=audit;expected=work_table,target_support,target_object_dynamic;present=work_table,target_support,target_object_dynamic;missing=none;all_present=true
```

## Launch

The Panda planning demo starts the audit node by default:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

It can be disabled if needed:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py use_planning_scene_audit:=false
```

The package-local launch is:

```bash
ros2 launch adaptive_assembly_planning planning_scene_audit.launch.py
```

## Validate

```bash
bash scripts/check_planning_scene_audit_available.sh
python3 scripts/check_planning_scene_audit_status.py
bash scripts/check_planning_scene_audit_ready.sh
```

`/planning_scene_audit_ready` is `true` when all expected objects are present.
With the default Panda planning demo, this means the static table/support and
dynamic target object have all been observed in the MoveIt2 PlanningScene.

## Scope

This tool is for scene inspection only:

- no PlanningScene modification
- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
