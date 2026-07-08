# ROS2 Adaptive Assembly

A ROS2 Jazzy + MoveIt2 project for deterministic adaptive robotic assembly in simulation.

This repository builds a lightweight adaptive manipulation pipeline that converts randomized object poses into robot-aware Panda planning targets, maintains TF and PlanningScene state, performs plan-only two-stage assembly planning, exports trajectories, records planning diagnostics, and provides deterministic benchmark profiles for evaluating robustness under target-pose variation.

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
- Plan-only two-stage `pre_grasp -> assembly` sequence planning
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
dry-run execution / simulator-only ros2_control execution
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

### Contact-lite insertion benchmark

Run the deterministic known-reachable plan-only sequence with geometric
insertion evaluation:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_contact_lite_insertion_benchmark.launch.py
```

The evaluator publishes:

- `/assembly_insertion_status`
- `/assembly_insertion_success`
- `/assembly_insertion_error_mm`
- `/assembly_insertion_error_deg`

By default, the achieved pose is the planned `/panda_assembly_pose`; status
therefore reports `achieved_pose_source=planned_pose`. This benchmark checks
only final-pose geometry. It adds no force control, tactile sensing,
contact-rich peg-in-hole behavior, real hardware execution, or Gazebo contact
physics requirement.

Record CSV data and export a Markdown summary:

```bash
MAX_TRIALS=20 TIMEOUT_SEC=120 \
  bash scripts/run_contact_lite_insertion_benchmark.sh
```

Validate with synthetic poses and report export:

```bash
bash scripts/check_contact_lite_insertion_topics.sh
python3 scripts/check_contact_lite_insertion_success_path.py
python3 scripts/check_contact_lite_insertion_failure_path.py
bash scripts/check_contact_lite_insertion_report_export.sh
```

See
[`docs/contact_lite_insertion_benchmark.md`](docs/contact_lite_insertion_benchmark.md)
for parameters, status fields, and limitations.

### Passive assembly episode supervisor

Launch the status-only episode aggregator and run its isolated synthetic checks:

```bash
ros2 launch adaptive_assembly_episode assembly_episode_supervisor.launch.py
bash scripts/check_assembly_episode_supervisor_available.sh
python3 scripts/check_assembly_episode_supervisor_success_path.py
python3 scripts/check_assembly_episode_supervisor_failure_path.py
python3 scripts/check_assembly_episode_supervisor_timeout_path.py
```

See
[`docs/assembly_episode_supervisor.md`](docs/assembly_episode_supervisor.md)
for subscribed topics, terminal rules, and passive-only limitations.

### Full assembly episode demo

Launch the complete simulator-only composed episode:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_episode_demo.launch.py
```

The episode synchronizes `/target_pose` into the Gazebo `target_object` before
grasp attachment changes `/target_object_control_owner` from `target_sync`.
Synchronization status is available on `/gazebo_target_sync_status`.
MoveIt and the episode consumers start only after both Gazebo controllers are
active and `/joint_states` has a non-zero simulator timestamp.

While it is active, validate launch discovery, topics, and terminal status:

```bash
bash scripts/check_full_episode_launch_available.sh
python3 scripts/check_full_episode_target_sync.py
bash scripts/check_full_episode_topics.sh
python3 scripts/check_full_episode_terminal_status.py
bash scripts/check_gazebo_controllers_active.sh
```

See [`docs/full_assembly_episode_launch.md`](docs/full_assembly_episode_launch.md)
for composition details, success criteria, and limitations.

---

### Single-trial deterministic episode demo

Launch the deterministic-correctness profile with distinct deterministic source and
socket/place poses:

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_episode_visual_demo.launch.py
```

This remains simulator-only, with a logical gripper, kinematic object
attachment, and final-pose geometric insertion evaluation. The deterministic profile
places the cylinder center at `z=0.10` so it rests on the support and uses
`pre_grasp -> grasp -> pre_place -> place -> release -> retreat`. Attachment
occurs after successful `grasp`; release occurs after successful `place`, so
the detached object remains at the socket during retreat. It does not model
physical gripping, force control, or contact-rich insertion. See
[`docs/full_assembly_episode_launch.md`](docs/full_assembly_episode_launch.md).

The deterministic evaluator compares the desired final object pose on
`/object_place_pose` with the observed Gazebo pose on
`/gazebo_target_object_pose`. `/panda_assembly_pose` remains the Panda hand
planning target and is not used as the evaluator target.

The deterministic executor also waits for `/gazebo_target_sync_status` to report
`event=success` before sending its first ros2_control trajectory. This prevents
execution from starting before Gazebo mirrors the planned source pose. The
gate applies only at sequence startup and is disabled by default in generic
execution demos.

Before any planning, execution, grasp lifecycle, evaluation, or supervision
starts, the deterministic launch now waits on retained
`/gazebo_controller_ready_status`. Success requires both Panda controllers to
be active, the simulated `FollowJointTrajectory` server to be available, and
finite values for all seven Panda joints on `/joint_states`. The target-sync
gate remains the second startup gate before the first trajectory goal. The
episode supervisor waits for an explicit terminal executor status (or its
episode timeout) and does not infer failure from an initially absent success.


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
