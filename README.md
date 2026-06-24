# ROS2 Adaptive Assembly

A ROS2 Jazzy project for simulated adaptive robotic assembly.

## Target

This project aims to build a minimal, testable adaptive manipulation pipeline:

1. Fake perception publishes randomized target object poses.
2. TF2 broadcasts the object frame.
3. A task node computes pre-grasp and assembly poses.
4. Future versions will add MoveIt2 planning, PlanningScene updates, and
   replanning behavior.

## Environment

- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Python / rclpy
- colcon

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current pipeline

The current non-MoveIt pipeline provides:

- `/target_pose`: randomized target object pose from fake perception
- `/pre_grasp_pose`: task-level pose above the target object
- `/assembly_pose`: task-level pose near the assembly target
- TF `world -> target_object`: transform matching the target object pose

MoveIt2 planning, Gazebo, RViz, robot models, and ros2_control are not launched
by the current pipeline.

## Run the current pipeline

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_pipeline.launch.py
```

## Validate the current pipeline

Start the pipeline in one terminal, then run the validation scripts from another
terminal:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
bash scripts/check_pipeline_topics.sh
python3 scripts/check_pipeline_offsets.py
bash scripts/run_pipeline_validation.sh
bash scripts/echo_pipeline_once.sh
```

See [docs/current_pipeline.md](docs/current_pipeline.md) for the architecture
and validation workflow.

## Optional MoveIt2 Panda demo

If MoveIt2 and the Panda demo resources are installed, the current adaptive
assembly pipeline can be launched alongside the standard Panda MoveIt2 demo:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_demo.launch.py
```

This is a demo bringup only; it does not yet send `/pre_grasp_pose` or
`/assembly_pose` to MoveIt2 for planning. See
[docs/moveit2_panda_demo.md](docs/moveit2_panda_demo.md).

## Roadmap

- PR1: fake perception node
- PR2: task pose generation node
- PR3: bringup launch for the non-MoveIt pipeline
- PR4: validation scripts and documentation cleanup
- PR5: optional Panda MoveIt2 demo bringup
- Future PR: MoveIt2 planning integration
