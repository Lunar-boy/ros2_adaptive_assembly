# Full Assembly Episode Launch

## Purpose

`adaptive_assembly_full_episode_demo.launch.py` composes one complete simulator-only assembly episode from existing runtime components. It adds no new node behavior.

The launch includes full Gazebo Panda execution, Gazebo target pose synchronization, the logical grasp lifecycle, kinematic Gazebo attach/detach, the Gazebo achieved object pose observer, contact-lite insertion evaluation, and the passive assembly episode supervisor.

The visual single-trial launch uses the explicit sequence `pre_grasp -> grasp
-> assembly`. Its source cylinder center is at `z=0.10`, which places the
0.10 m-tall object on the support instead of floating above it. Logical and
kinematic attachment occurs only after the ros2_control `grasp` stage reports
success; aggregate execution success still releases the object.

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
- No real camera or visual servoing
- No real robot hardware
- No CSV/Markdown benchmark recorder is added by this launch

## Single-trial visual episode demo

```bash
ros2 launch adaptive_assembly_bringup adaptive_assembly_full_episode_visual_demo.launch.py
```

This visual-correctness launch uses the deterministic source pose
`(0.442, 0.148, 0.15)` and a distinct fixed socket/place pose
`(0.62, -0.18, 0.10)`. It synchronizes the source pose into Gazebo before
logical attachment, then evaluates `/gazebo_target_object_pose` against
the desired final object pose on `/object_place_pose` with
`achieved_pose_source=gazebo_entity_pose_observer`.

`/panda_assembly_pose` remains the Panda hand planning target. It is distinct
from the desired final object pose even though both currently use the fixed
socket pose for backward compatibility.

The demo remains simulator-only. Its gripper is logical, attachment is
kinematic, and insertion evaluation is final-pose geometry only; it provides
no real gripper physics, force/contact insertion, visual servoing, camera, or
hardware support.

Validation:

```bash
python3 scripts/check_fixed_socket_assembly_pose.py
bash scripts/check_visual_episode_launch_available.sh
python3 scripts/check_visual_episode_config.py
```
