# Current non-MoveIt pipeline

The current adaptive assembly pipeline is intentionally small and testable. It
connects fake perception to task-level pose generation without starting MoveIt2,
Gazebo, RViz, robot models, ros2_control, or real robot drivers.

```text
fake_object_pose_node
     │
     ├── /target_pose
     └── TF: world -> target_object
               │
               ▼
        assembly_task_node
     ├── /pre_grasp_pose
     └── /assembly_pose
```

## Nodes and interfaces

`fake_object_pose_node` is provided by `adaptive_assembly_perception`.

- Publishes `/target_pose` as `geometry_msgs/msg/PoseStamped`
- Broadcasts TF `world -> target_object`

`assembly_task_node` is provided by `adaptive_assembly_task`.

- Subscribes to `/target_pose`
- Publishes `/pre_grasp_pose` as `geometry_msgs/msg/PoseStamped`
- Publishes `/assembly_pose` as `geometry_msgs/msg/PoseStamped`
- Applies the configured z offsets to the received target pose

`adaptive_assembly_bringup` provides the single launch entry point and parameter
file for the current pipeline.

## Run the pipeline

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py
```

## Validate the pipeline

In another terminal:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
bash scripts/check_pipeline_topics.sh
python3 scripts/check_pipeline_offsets.py
bash scripts/run_pipeline_validation.sh
bash scripts/echo_pipeline_once.sh
```

The validation checks confirm that:

- `/target_pose`, `/pre_grasp_pose`, and `/assembly_pose` exist
- all three pose topics use `geometry_msgs/msg/PoseStamped`
- `pre_grasp_pose.pose.position.z = target_pose.pose.position.z + 0.20`
- `assembly_pose.pose.position.z = target_pose.pose.position.z + 0.05`

## Future MoveIt2 integration

MoveIt2 planning is intentionally not included in this pipeline yet. A future PR
will add motion planning integration after the current perception, task, bringup,
and validation layers are stable.
