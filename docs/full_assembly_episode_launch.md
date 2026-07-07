# Full Assembly Episode Launch

## Purpose

`adaptive_assembly_full_episode_demo.launch.py` composes one complete simulator-only assembly episode from existing runtime components. It adds no new node behavior.

The launch includes full Gazebo Panda execution, Gazebo target pose synchronization, the logical grasp lifecycle, kinematic Gazebo attach/detach, the Gazebo achieved object pose observer, contact-lite insertion evaluation, and the passive assembly episode supervisor.

Before grasp attachment takes ownership, the target synchronizer mirrors
`/target_pose` into the Gazebo `target_object`. It publishes retained status on
`/gazebo_target_sync_status` and writes only while
`/target_object_control_owner` is `target_sync`. While attached,
`gripper_attach` owns the object; after release, the `released` state leaves it
at its final world pose.

## Run

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_episode_demo.launch.py
```

The contact-lite evaluator uses:

```text
target_pose_topic=/panda_assembly_pose
achieved_pose_topic=/gazebo_target_object_pose
achieved_pose_source=gazebo_entity_pose_observer
require_execution_success=true
```

The episode succeeds when the passive supervisor reports `event=success` after planning, execution, logical release, Gazebo attachment, and insertion success.

## Validation

With the workspace built and sourced:

```bash
bash scripts/check_full_episode_launch_available.sh
python3 scripts/check_full_episode_target_sync.py
bash scripts/check_full_episode_topics.sh
python3 scripts/check_full_episode_terminal_status.py
```

The topic and terminal checks run while the full episode launch is active. Use `--allow-non-success` with the terminal check when deterministic failure or timeout is expected in a dependency-limited environment.

## Limitations

- Simulator-only
- Logical gripper only
- Kinematic Gazebo attachment only
- No physical grasping
- Final-pose geometric insertion evaluation only
- No force control or contact-rich insertion
- No camera/image perception, marker detection, or visual servoing
- No real robot hardware
- No CSV/Markdown benchmark recorder is added by this launch


## Single-trial deterministic episode demo

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_episode_deterministic_demo.launch.py
```

The deterministic launch starts Gazebo and its ros2_control configuration first. It
then waits for retained `/gazebo_controller_ready_status` success before it
starts MoveIt sequence planning, execution, logical grasp/attachment,
evaluation, or the episode supervisor. Readiness requires the
`joint_state_broadcaster` and `panda_arm_controller` to be active, the
`FollowJointTrajectory` action server to exist, and one finite seven-joint
Panda `/joint_states` message. The bounded gate defaults to 60 seconds and
remains simulator-only.

This deterministic-correctness launch uses the deterministic source pose
`(0.442, 0.148, 0.15)` and a distinct fixed socket/place pose
`(0.62, -0.18, 0.10)`. It synchronizes the source pose into Gazebo before
logical attachment, then evaluates `/gazebo_target_object_pose` against
the desired final object pose on `/object_place_pose` with
`achieved_pose_source=gazebo_entity_pose_observer`.

`/panda_assembly_pose` remains the Panda hand planning target. It is distinct
from the desired final object pose even though both currently use the fixed
socket pose for backward compatibility.

Before the first `FollowJointTrajectory` goal is sent, the deterministic demo gates
execution on `/gazebo_target_sync_status` reporting `event=success`. This
ensures Gazebo `target_object` has been moved to the planned source pose before
the `pre_grasp -> grasp -> pre_place -> place -> release -> retreat` sequence
starts. The object attaches after successful `grasp`, remains attached through
`pre_place` and `place`, and releases before `retreat`. The gate has a 10-second
timeout after trajectories and joint state are ready; generic execution demos
leave it disabled by default.

Controller readiness precedes target synchronization and execution. The
supervisor also remains pending until the executor publishes an explicit
terminal success/failure/skipped/rejected/timeout event; absence of execution
success is not itself a failure. Its `episode_timeout_sec` remains the bounded
fallback when no terminal execution status arrives.

The demo remains simulator-only. Its gripper is logical, attachment is
kinematic set-pose following, and insertion evaluation is final-pose geometry
only. It provides no physical fingers, force control, contact-rich insertion,
deterministic servoing, camera, or real-robot support.

Validation:

```bash
python3 scripts/check_fixed_socket_assembly_pose.py
bash scripts/check_deterministic_episode_launch_available.sh
python3 scripts/check_deterministic_episode_config.py
python3 scripts/check_gazebo_controller_ready.py
python3 scripts/check_deterministic_episode_runtime_order.py
```

