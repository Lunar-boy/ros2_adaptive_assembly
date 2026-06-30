# Gazebo workcell bringup

PR34 adds the project's first self-contained Gazebo Harmonic environment. It
is a lightweight visual workcell, not a robot execution stack.

## Contents

The installed `adaptive_assembly_workcell.sdf` world uses box and cylinder
primitives only. It contains:

- a floor and work table;
- a raised target support;
- a static cylindrical target object;
- a four-sided assembly socket fixture.

The names are stable and descriptive so later simulation work can extend the
world without replacing these assets. No meshes or binary assets are used.

## Prerequisites

ROS 2 Jazzy uses Gazebo Harmonic through `ros_gz_sim`. If it is missing:

```bash
sudo apt install ros-jazzy-ros-gz-sim
```

The launch checks for both the `ros_gz_sim` package and the `gz` executable and
reports an actionable error when either is unavailable.

## Launch

Launch only the workcell:

```bash
ros2 launch adaptive_assembly_sim adaptive_assembly_workcell.launch.py
```

Launch the workcell together with the existing fake-perception and task-pose
pipeline:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_gazebo_workcell_demo.launch.py
```

Set `launch_pipeline:=false` to use the bringup wrapper without the pipeline.
Both launch files accept `world` and `gz_args` arguments. For example, a
server-only smoke run can omit the graphical client:

```bash
ros2 launch adaptive_assembly_sim adaptive_assembly_workcell.launch.py \
  gz_args:='-r -s /absolute/path/to/adaptive_assembly_workcell.sdf'
```

## Integration boundary

The optional pipeline publishes its existing ROS topics next to Gazebo, but it
does not move or respawn the static SDF target. The world does not contain a
Panda model, controller configuration, `ros2_control` hardware plugin, gripper,
or attach/detach mechanism. The wrapper deliberately does not include the
existing ros2_control sequence executor.

Consequently, this bringup demonstrates the assembly-cell environment only. It
does not execute trajectories, model grasping, perform contact-rich insertion,
apply force control, or support real hardware. Gazebo controller success-path
execution is deferred to PR35.

## Validation

The checks inspect installed assets and load each launch description without
starting a GPU-heavy simulator:

```bash
bash scripts/check_gazebo_workcell_assets.sh
bash scripts/check_gazebo_workcell_launch_available.sh
```
