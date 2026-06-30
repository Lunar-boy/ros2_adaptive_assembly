# adaptive_assembly_sim

Asset-only ROS 2 package for the minimal Gazebo Harmonic adaptive assembly
workcell. The installed SDF world contains a floor, table, target support,
cylindrical target object, and assembly socket fixture made from primitive
geometry.

## Launch

```bash
ros2 launch adaptive_assembly_sim adaptive_assembly_workcell.launch.py
```

The launch requires `ros_gz_sim` and the `gz` executable. On ROS 2 Jazzy they
can be installed with:

```bash
sudo apt install ros-jazzy-ros-gz-sim
```

This package does not spawn a robot, configure controllers, execute
trajectories, or simulate grasping and insertion. The target object is static
and is not synchronized with `/target_pose`.

## Validate

After building and sourcing the workspace:

```bash
bash scripts/check_gazebo_workcell_assets.sh
bash scripts/check_gazebo_workcell_launch_available.sh
```
