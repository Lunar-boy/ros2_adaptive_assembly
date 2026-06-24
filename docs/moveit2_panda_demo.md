# Optional MoveIt2 Panda demo bringup

PR5 adds a demo bringup entry point only. It launches the existing adaptive
assembly perception/task pipeline together with the standard Panda MoveIt2 demo
and RViz environment.

This does not yet connect `/pre_grasp_pose` or `/assembly_pose` to MoveIt2
planning. No automatic motion planning to the adaptive assembly target poses is
performed in this PR.

The Panda MoveIt2 demo is used as a stable standard robot model and RViz
planning environment so future planning integration can build on a known-good
MoveIt2 setup.

## Install dependencies

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Validate package availability

```bash
bash scripts/check_moveit2_demo_available.sh
```

## Run

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_demo.launch.py
```

## Expected behavior

- Fake perception publishes `/target_pose`
- The task node publishes `/pre_grasp_pose` and `/assembly_pose`
- The standard Panda MoveIt2 demo starts
- RViz may open if the machine has GUI support
- No automatic MoveIt2 planning to `/pre_grasp_pose` or `/assembly_pose`
  happens yet

## Next PR

A future PR will add a planning bridge that reads `/pre_grasp_pose` and sends a
planning request to MoveIt2.
