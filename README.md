# ROS 2 Adaptive Assembly: Full Physical Pick-and-Place

A ROS 2 Jazzy, MoveIt 2, Gazebo Harmonic, and `ros2_control` project for one simulator-only task: a Franka Panda picks a dynamic cylindrical object and places it into a socket fixture.

The primary and supported entry point is:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py
```

## Project goal

The task is complete only when the simulated Panda:

1. observes the Gazebo `target_object`;
2. moves to pre-grasp and grasp poses;
3. closes the gripper and verifies the grasp;
4. lifts and transports the object;
5. moves to the socket place pose;
6. releases the object;
7. retreats; and
8. leaves the object stably inside the socket.

The final condition must be verified from Gazebo-observed object state. Planning success, controller success, gripper success, or executor completion alone is not end-to-end task success.

## Current status

| Capability | Status |
|---|---|
| Physical Gazebo workcell with Panda, support, dynamic target, and socket | Implemented |
| Gazebo target-object pose observation | Implemented |
| Six-stage MoveIt planning | Implemented; grasp uses validated Pilz `LIN` |
| Immutable physical plan generation | Implemented; one volatile plan lock |
| Dynamic target collision object | Implemented; cylinder, finger-only ACM |
| Gazebo `ros2_control` arm execution | Implemented |
| Simulated gripper close/open | Implemented |
| Bilateral contact-aware close handling | Implemented |
| Grasp verification | Implemented |
| Lift/slip verification | Implemented |
| Final post-release socket insertion verification | **Not yet implemented** |

The current executor publishes `/physical_pick_place_execution_success`. That topic means the configured execution sequence completed; it does **not** prove that the released object remains inside the socket.

## Runtime sequence

```text
Gazebo target-object pose
        |
        v
/gazebo_target_object_pose_raw
        |
        v
gazebo_entity_pose_observer_node
        |
        +--> /gazebo_target_object_pose
        +--> /gazebo_target_object_pose_available
        |
        v
gazebo_target_pose_adapter_node
        |
        v
/target_pose
        |
        v
assembly_task_node
        |
        +--> pre_grasp
        +--> grasp
        +--> lift
        +--> pre_place
        +--> place
        +--> retreat
        |
        v
Panda pose adapters and MoveIt sequence planning
        |
        v
six RobotTrajectory topics
        |
        v
physical_pick_place_executor_node
        |
        +--> Panda arm FollowJointTrajectory action
        +--> initial gripper open before pre_grasp
        +--> gripper close after grasp
        +--> grasp verification
        +--> lift/slip verification
        +--> gripper open after place
        +--> retreat
```

The default stage order is:

```text
initial open -> pre_grasp -> grasp -> close -> verify grasp
          -> lift -> verify lift/slip
          -> pre_place -> place -> open -> retreat
```

The canonical MoveIt Panda description retains its standard second-finger
mimic relation. The Gazebo-only renderer removes that one mimic element and
the simulator controller commands both Panda finger joints to equal position
targets; this remains simulator-only position control, not force control.

## Environment

Target environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- Gazebo Harmonic / `ros_gz_sim`
- `gz_ros2_control`
- Python 3
- `colcon`

This repository supports Gazebo simulation only. It does not provide real robot drivers or real-hardware execution.

## Build

Use a standard colcon workspace:

```bash
mkdir -p ~/ros2_adaptive_assembly_ws/src
cd ~/ros2_adaptive_assembly_ws/src
git clone https://github.com/Lunar-boy/ros2_adaptive_assembly.git

cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The repository root is then:

```text
~/ros2_adaptive_assembly_ws/src/ros2_adaptive_assembly
```

## Run

From the workspace root:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py
```

The launch starts:

- `adaptive_assembly_physical_workcell.sdf`;
- the Gazebo Panda model and simulated controllers;
- `/clock` and simulation-time consumers;
- target-object pose bridging and observation;
- the physical task-pose pipeline;
- MoveIt sequence planning for `assembly_tcp`;
- arm and gripper execution;
- contact processing;
- physical preflight;
- grasp verification; and
- lift/slip verification.

Fake target-pose perception is disabled by default in this launch.

### Headless Gazebo

Use a server-only Gazebo run with:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py \
  gz_args:="-s $(ros2 pkg prefix adaptive_assembly_sim)/share/adaptive_assembly_sim/worlds/adaptive_assembly_physical_workcell.sdf"
```

## Save run logs

After sourcing the workspace, run the logging wrapper from the repository root:

```bash
cd ~/ros2_adaptive_assembly_ws/src/ros2_adaptive_assembly
bash scripts/run_full_physical_pick_place_with_logs.sh
```

Optional launch arguments can be passed after the script name:

```bash
RUN_ID=physical_trial_001 \
  bash scripts/run_full_physical_pick_place_with_logs.sh \
  use_sim_time:=true
```

Logs are written under `runs/<RUN_ID>/`.

## Observe execution

Useful retained terminal topics:

```bash
ros2 topic echo /physical_pick_place_execution_status --once
ros2 topic echo /physical_pick_place_execution_success --once
ros2 topic echo /physical_pick_place_execution_duration_ms --once
```

Stage-level transitions:

```bash
ros2 topic echo /physical_pick_place_stage_status
```

Target observation and preflight:

```bash
ros2 topic echo /gazebo_target_object_pose_status --once
ros2 topic echo /gazebo_target_object_pose_available --once
ros2 topic echo /gazebo_target_object_pose --once
ros2 topic echo /target_pose --once
ros2 topic echo /physical_grasp_preflight_status --once
```

Contact and verification:

```bash
ros2 topic echo /grasp_contact_status
ros2 topic echo /physical_gripper_command_status
ros2 topic echo /grasp_verification_status
ros2 topic echo /grasp_verified
ros2 topic echo /lift_verified
ros2 topic echo /grasp_slip_distance_mm
```

## Success semantics

### What is already verified

The current physical path can verify:

- the physical simulation prerequisites are available;
- all six planned trajectories are non-empty and controller-compatible;
- the Panda controller accepts and completes arm stages;
- gripper close/open results are valid;
- gripper close has the required target-object contact evidence;
- the object is grasped after close; and
- the object is lifted without exceeding the configured slip limit.

### What is still missing

A complete task requires a final verifier that evaluates the Gazebo-observed target pose after release and retreat. It should require a fresh pose, configurable socket position/orientation tolerances, and a stable settle interval.

Until that verifier exists, do not interpret:

```text
/physical_pick_place_execution_success = true
```

as proof that the object was successfully inserted into the socket.

## Configuration

The physical task profile is:

```text
src/adaptive_assembly_bringup/config/
  adaptive_assembly_physical_pick_place_params.yaml
```

Important defaults include:

- target source: dynamic Gazebo `target_object`;
- end-effector target link: `assembly_tcp`;
- stage order: `pre_grasp,grasp,lift,pre_place,place,retreat`;
- socket center: `(0.62, -0.18, 0.10)` in `world`;
- required gripper open before `pre_grasp`;
- gripper close after `grasp`;
- gripper open after `place`;
- required physical preflight;
- required grasp verification;
- required lift verification; and
- simulation time enabled.

The Gazebo geometry source of truth is:

```text
src/adaptive_assembly_sim/worlds/
  adaptive_assembly_physical_workcell.sdf
```

The corresponding MoveIt static collision profile is:

```text
src/adaptive_assembly_bringup/config/
  physical_workcell_planning_scene.yaml
```

Keep both descriptions geometrically consistent.

## Validation

### Package tests

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon test --packages-select \
  adaptive_assembly_bringup \
  adaptive_assembly_execution \
  adaptive_assembly_manipulation \
  adaptive_assembly_planning \
  adaptive_assembly_sim \
  adaptive_assembly_task \
  --event-handlers console_direct+

colcon test-result --verbose
```

### Physical-path static checks

Run from the repository root after sourcing the workspace:

```bash
cd ~/ros2_adaptive_assembly_ws/src/ros2_adaptive_assembly

python3 scripts/check_physical_pick_place_launch_static.py
python3 scripts/check_physical_grasp_preflight_static.py
python3 scripts/check_physical_planning_scene_parity.py
python3 scripts/check_physical_pick_place_executor_static.py
```

### Bounded runtime checks

```bash
python3 scripts/check_full_physical_pick_place_arm_motion.py
python3 scripts/check_full_physical_pick_place_tcp_contract.py
python3 scripts/check_full_physical_pick_place_plan_lock_runtime.py
```

The arm-motion check proves initial controller acceptance and motion. The TCP check proves bounded Cartesian agreement for pre-grasp and grasp. Neither check proves contact grasp, lift, final placement, or socket insertion.

### Robot-model parity

```bash
ros2 run adaptive_assembly_sim check_robot_model_parity \
  --current-panda-models \
  --reference-tool-link assembly_tcp \
  --candidate-tool-link assembly_tcp
```

This checks model and forward-kinematics parity; it does not prove task success.

## Relevant repository layout

```text
ros2_adaptive_assembly/
├── AGENTS.md
├── README.md
├── docs/
│   ├── physical_pick_place_execution.md
│   ├── gazebo_contact_grasp_verification.md
│   └── run_logging.md
├── scripts/
│   ├── check_physical_*.py
│   ├── check_full_physical_pick_place_*.py
│   └── run_full_physical_pick_place_with_logs.sh
└── src/
    ├── adaptive_assembly_bringup/
    ├── adaptive_assembly_execution/
    ├── adaptive_assembly_manipulation/
    ├── adaptive_assembly_planning/
    ├── adaptive_assembly_sim/
    └── adaptive_assembly_task/
```

The repository may temporarily retain older demos and regression fixtures, but future development and root-level documentation should remain focused on `adaptive_assembly_full_physical_pick_place_demo.launch.py` and its dependencies.

## Scope and limitations

In scope:

- Gazebo physics simulation;
- MoveIt planning;
- simulated Panda arm and gripper control;
- object-pose observation;
- contact-aware grasp verification;
- lift/slip verification;
- final socket-placement verification; and
- deterministic logging and validation.

Out of scope unless explicitly introduced in a future project change:

- real robot hardware;
- real camera perception;
- marker detection;
- visual servoing;
- force-controlled insertion;
- tactile control;
- VLA policies; and
- Isaac Sim.

## License

Apache-2.0
