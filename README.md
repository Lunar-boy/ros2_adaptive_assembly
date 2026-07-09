# ROS2 Adaptive Assembly

A ROS2 Jazzy + MoveIt2 project for deterministic adaptive robotic assembly in simulation.

This repository builds a lightweight adaptive manipulation pipeline that converts randomized object poses into robot-aware Panda planning targets, maintains TF and PlanningScene state, performs configurable plan-only multi-stage assembly planning, exports trajectories, records planning diagnostics, and provides deterministic benchmark profiles for evaluating robustness under target-pose variation.

> **Current scope:** reproducible adaptive assembly planning plus simulator-only Gazebo Panda arm execution.
> **Not included:** camera/image perception, marker detection, visual servoing,
> real camera hardware, contact-rich insertion, force control, or real robot
> hardware execution.

---

## Key features

- ROS2 Jazzy workspace with modular Python packages
- Deterministic fake target-pose node publishing sampled target object poses
- TF2 target frame broadcasting
- Task-level pre-grasp and assembly pose generation
- Panda-specific pose adapters for MoveIt2 planning
- Plan-only MoveIt2 pre-grasp planning
- Configurable plan-only sequence planning (default: `pre_grasp -> assembly`)
- Static PlanningScene collision objects
- Dynamic target PlanningScene object
- PlanningScene audit and reset workflows
- Planning request guard and safety filter
- Stage-level planning diagnostics
- Trajectory export for downstream execution consumers
- Message-only dry-run execution abstraction
- Closed-loop recovery supervisor with deterministic recovery actions
- Deterministic benchmark profiles and CSV/Markdown report export
- Contact-lite geometric insertion benchmark and report export
- Lightweight Gazebo Harmonic workcell visualization
- Simulator-only Gazebo Panda arm execution through ros2_control
- Simulator-only Panda gripper links and finger joints in the Gazebo model
- Simulator-only gripper trajectory controller and logical command action bridge
- Simulator-only multi-stage pick-place executor with gripper close/open
  interleaving
- Simulator-only Gazebo finger contact sensing and grasp/lift/slip verification
- Simulator-only Gazebo target object synchronization from `/target_pose`

---

## System architecture

```text
fake_object_pose_node
    ├── /target_pose
    └── TF: world -> target_object
              │
              ▼
assembly_task_node
    ├── /grasp_candidates (deterministic string schema)
    ├── /selected_grasp_pose (= /grasp_pose legacy alias)
    ├── /pre_grasp_pose
    ├── /lift_pose
    ├── /assembly_pose (robot hand target)
    └── /object_place_pose (desired final object pose)
              │
              ▼
panda_pose_adapter_node
    ├── /panda_pre_grasp_pose
    └── /panda_assembly_pose
              │
              ▼
MoveIt2 sequence planner
    ├── pre-grasp plan
    ├── assembly plan
    ├── trajectory export
    └── diagnostics / benchmark CSV
              │
              ▼
dry-run execution / simulator-only ros2_control or physical pick-place execution
              │
              ▼
contact-lite insertion evaluator
```

The current pipeline separates target-pose generation, task-level pose generation, robot-specific pose adaptation, planning, diagnostics, and execution abstraction. This makes the system easy to test incrementally and extend toward simulator or hardware execution.

---

## Repository layout

```text
ros2_adaptive_assembly/
├── docs/                         # Design notes, feature documentation, validation workflows
├── scripts/                      # Validation, benchmark, and utility scripts
├── benchmark_results/            # Optional benchmark CSV/Markdown outputs
└── src/
    ├── adaptive_assembly_bringup/     # Launch files and integration entry points
    ├── adaptive_assembly_benchmark/   # Contact-lite geometric benchmark nodes
    ├── adaptive_assembly_execution/   # Dry-run and execution-bridge abstractions
    ├── adaptive_assembly_perception/  # Deterministic fake target-pose generation
    ├── adaptive_assembly_recovery/    # Recovery supervisor and deterministic actions
    ├── adaptive_assembly_sim/         # Gazebo workcell assets and launch files
    └── adaptive_assembly_task/        # Task pose generation and planning adapters
```

---

## Environment

Tested target environment:

- Ubuntu 24.04
- ROS2 Jazzy
- MoveIt2
- Gazebo Harmonic / `ros_gz_sim`
- Python 3
- `rclpy`
- `colcon`

Recommended optional packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

Depending on your local MoveIt2 installation, the standard Panda demo resources may also be required for Panda planning profiles.

---

## Build

```bash
mkdir -p ~/ros2_adaptive_assembly_ws/src
cd ~/ros2_adaptive_assembly_ws/src
git clone https://github.com/Lunar-boy/ros2_adaptive_assembly.git

cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---
## Launch

```
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_physical_pick_place_demo.launch.py
```

This full physical demo remains simulator-only. It uses Gazebo
`gz_ros2_control` as the only controller provider and starts MoveIt planning
without the standard Panda fake-control demo.

To save terminal output from each full physical pick-place simulation attempt,
use the manual run logging wrapper:

```bash
bash scripts/run_full_physical_pick_place_with_logs.sh
```

See `docs/run_logging.md` for `RUN_ID`, `RUN_DIR`, and launch-argument usage.


### Result table

| Profile | Events | Success | Failed | Skipped | Mean planning time |
|---|---:|---:|---:|---:|---:|
| baseline | TBD | TBD | TBD | TBD | TBD |
| narrow target range | TBD | TBD | TBD | TBD | TBD |
| wide target range | TBD | TBD | TBD | TBD | TBD |
| fixed yaw | TBD | TBD | TBD | TBD | TBD |
| guarded planner | TBD | TBD | TBD | TBD | TBD |


---

## License

Apache-2.0 
