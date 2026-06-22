# AGENTS.md

This repository is a ROS2 Jazzy project for adaptive robotic assembly.

Environment:
- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Gazebo Harmonic / ros_gz optional
- Python packages use ament_python
- Build with colcon

Rules:
- Keep PRs small and focused.
- Do not commit build/, install/, or log/.
- Do not introduce Isaac Sim.
- Do not require a real robot.
- Prefer simple, testable ROS2 nodes.
- Every package must have a README.
- Every new node must include clear parameters and launch integration.
- Use `colcon build --symlink-install` for validation.
- Avoid large binary files.

Target project:
A simulated adaptive assembly pipeline:
1. fake perception publishes target object pose;
2. task node computes pre-grasp and assembly poses;
3. MoveIt2 plans robot motion;
4. PlanningScene is updated with collision objects;
5. randomized object poses trigger replanning.
