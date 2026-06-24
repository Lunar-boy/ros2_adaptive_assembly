# Reproducible benchmark profile

PR12 makes planning benchmarks reproducible by adding:

- deterministic fake perception seed support
- benchmark parameter YAML
- `params_file` launch arguments in bringup launch files
- a seeded Panda planning benchmark launch profile

Default demo behavior remains non-deterministic unless a seeded profile is
selected.

Key fake perception parameters:

- `random_seed`: non-negative values make the generated pose sequence
  reproducible; negative values keep non-deterministic behavior
- `frame_id`: frame used by `/target_pose` and TF parent
- `target_frame_id`: TF child frame for the target object
- `yaw_min`: minimum sampled yaw in radians
- `yaw_max`: maximum sampled yaw in radians
- `publish_immediately`: publish one pose at startup before the timer

```text
adaptive_assembly_panda_planning_benchmark.launch.py
     │
     ├── params_file = adaptive_assembly_benchmark_params.yaml
     │
     ▼
fake_object_pose_node
     │ deterministic random_seed=42
     ▼
/target_pose
     │
     ▼
/pre_grasp_pose -> /panda_pre_grasp_pose -> MoveIt2 plan-only bridge
     │
     ▼
planning diagnostics -> CSV benchmark recorder
```

## Build

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Validate the profile

```bash
bash scripts/check_benchmark_profile_available.sh
bash scripts/check_seeded_fake_perception_params.sh
```

## Run the seeded benchmark launch

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_panda_planning_benchmark.launch.py
```

## Record a seeded benchmark

In another terminal:

```bash
MAX_EVENTS=20 TIMEOUT_SEC=120 OUTPUT=benchmark_results/seeded_planning_diagnostics.csv bash scripts/run_seeded_planning_benchmark.sh
```

This profile is still plan-only:

- no trajectory execution
- no Gazebo
- no ros2_control integration for this project
- no real hardware
- no dynamic PlanningScene updates

## Next PR

A future PR can add controlled benchmark profiles for wider or narrower target
distributions, or compare planning with and without static PlanningScene
objects.
