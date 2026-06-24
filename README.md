# ROS2 Adaptive Assembly

A ROS2 Jazzy + MoveIt2 project for simulated adaptive robotic assembly.

## Target

This project aims to build a minimal but complete adaptive manipulation pipeline:

1. Fake perception publishes randomized target object poses.
2. TF2 broadcasts the object frame.
3. A task node computes pre-grasp and assembly poses.
4. MoveIt2 plans robot motion for a simulated Panda arm.
5. Later versions will add PlanningScene updates, replanning, and benchmark scripts.

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
## Roadmap
* PR1: fake perception node
* PR2: Panda MoveIt2 bringup
* PR3: task node for pre-grasp and assembly poses
* PR4: MoveIt2 planning integration

### Active Dev Session
* CLI Resume: `codex resume 019eef8e-994c-7791-8542-7caf4ccc2c0c`
