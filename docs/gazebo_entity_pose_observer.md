# Gazebo entity pose observer

The simulator-only observer publishes the achieved Gazebo pose of
`target_object` for downstream insertion evaluation. It observes state only;
it never commands Gazebo.

## Interfaces

Input from `ros_gz_bridge`:

- `/world/adaptive_assembly_workcell/pose/info` (`gz.msgs.Pose_V`, represented
  as `tf2_msgs/msg/TFMessage` by the ROS 2 Jazzy bridge)

Outputs:

- `/gazebo_target_object_pose` (`geometry_msgs/msg/PoseStamped`)
- `/gazebo_target_object_pose_status` (`std_msgs/msg/String`, retained)
- `/gazebo_target_object_pose_available` (`std_msgs/msg/Bool`, retained)
- `/gazebo_target_object_pose_age_ms` (`std_msgs/msg/Float64`, retained)

Launch the bridge and observer:

```bash
ros2 launch adaptive_assembly_sim gazebo_entity_pose_observer.launch.py
```

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
