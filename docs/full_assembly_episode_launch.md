# Full Assembly Episode Launch

## Purpose

`adaptive_assembly_full_episode_demo.launch.py` composes one complete simulator-only assembly episode from existing runtime components. It adds no new node behavior.

The launch includes full Gazebo Panda execution, the logical grasp lifecycle, kinematic Gazebo attach/detach, the Gazebo achieved object pose observer, contact-lite insertion evaluation, and the passive assembly episode supervisor.

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
bash scripts/check_full_episode_topics.sh
python3 scripts/check_full_episode_terminal_status.py
```

The topic and terminal checks run while the full episode launch is active. Use `--allow-non-success` with the terminal check when deterministic failure or timeout is expected in a dependency-limited environment.

## Limitations

- Simulator-only
- Logical gripper only
- Kinematic Gazebo attachment only
- Final-pose geometric insertion evaluation only
- No force control or contact-rich insertion
- No real camera or visual servoing
- No real robot hardware
- No CSV/Markdown benchmark recorder is added by this launch
