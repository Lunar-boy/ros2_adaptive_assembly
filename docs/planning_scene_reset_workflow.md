# PlanningScene reset workflow

PR18 adds a small helper workflow for cleaning the Panda planning demo scene
before repeated demos or benchmark runs. It uses the existing static and
dynamic PlanningScene services; it does not change ROS2 node behavior.

The reset order is:

1. clear the dynamic target object with `/clear_dynamic_target_scene`
2. clear static objects with `/clear_static_planning_scene`
3. reapply static objects with `/reapply_static_planning_scene`

This leaves the table/support objects restored and the dynamic target object
cleared at the moment the workflow completes.

If the demo is still publishing `/panda_pre_grasp_pose`, the dynamic target
object may be re-created automatically by `dynamic_target_scene_node` when the
next adapted pose arrives. That behavior is expected.

## Run

Start the Panda planning demo first:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_demo.launch.py
```

Then run the reset workflow from another terminal:

```bash
bash scripts/reset_planning_scene_once.sh
```

## Validate

```bash
bash scripts/check_planning_scene_reset_workflow.sh
```

Optional status checks after a reset:

```bash
python3 scripts/check_static_planning_scene_status.py
python3 scripts/check_dynamic_target_scene_status.py
```

This workflow is useful before A/B benchmark recording because it gives each
run a predictable static scene baseline while clearing any previously applied
dynamic target object.

This remains diagnostics and scene hygiene only:

- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
