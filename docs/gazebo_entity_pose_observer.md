# Gazebo entity pose observer

The simulator-only observer publishes the achieved Gazebo pose of
`target_object` for downstream insertion evaluation. It observes state only;
it never commands Gazebo.

## Interfaces

Input from `ros_gz_bridge`:

- `/world/adaptive_assembly_workcell/pose/info` (`gz.msgs.Pose_V`, represented
  as `tf2_msgs/msg/TFMessage` by the ROS 2 Jazzy bridge)
- `/world/adaptive_assembly_physical_workcell/pose/info` for the physical
  simulator path

Outputs:

- `/gazebo_target_object_pose` (`geometry_msgs/msg/PoseStamped`)
- `/gazebo_target_object_pose_status` (`std_msgs/msg/String`, retained)
- `/gazebo_target_object_pose_available` (`std_msgs/msg/Bool`, retained)
- `/gazebo_target_object_pose_age_ms` (`std_msgs/msg/Float64`, retained)

Launch the bridge and observer:

```bash
ros2 launch adaptive_assembly_sim gazebo_entity_pose_observer.launch.py
```

The standalone observer keeps strict entity matching by default through
`require_model_name_match:=true`. The physical pick-place launch sets
`require_target_entity_exact_match:=false`, allowing Gazebo scope components
such as `world::adaptive_assembly_physical_workcell::target_object` and
`/world/adaptive_assembly_physical_workcell/model/target_object`. Matching is
component-based: similarly prefixed visual, collision, and backup names are
not accepted.

Inspect retained observer diagnostics with:

```bash
ros2 topic echo /gazebo_target_object_pose_status --once
ros2 topic echo /gazebo_target_object_pose_available --once
```

Availability becomes `true` only after a matching valid pose is extracted.
Failed extraction remains machine-readable as `event=skipped` with a concrete
`reason`, while the throttled node warning shows a capped set of candidate
entity/frame names. `reason=entity_names_unavailable` specifically indicates
that the bridged message contained pose entries but omitted every entity/frame
name; the observer does not guess by array position in that unsafe case.

The contact-lite evaluator can consume the observed pose with:

```text
achieved_pose_topic:=/gazebo_target_object_pose
achieved_pose_source:=gazebo_entity_pose_observer
require_execution_success:=true
```

## Validation

```bash
bash scripts/check_gazebo_entity_pose_observer_available.sh
python3 scripts/check_gazebo_entity_pose_observer_synthetic.py
python3 scripts/check_gazebo_entity_pose_observer_stale.py
```

## Limitations

This feature is simulator-only and depends on the Gazebo `pose/info` bridge.
It observes the final model pose only. It does not command Gazebo, perform
grasping, implement a full assembly episode or benchmark recorder, provide
contact-rich insertion or force control, or support real hardware.
