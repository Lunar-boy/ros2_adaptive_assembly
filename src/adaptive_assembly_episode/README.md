# adaptive_assembly_episode

This package provides a passive, simulator-only supervisor that aggregates
existing planning, execution, logical-grasp, Gazebo-attachment, and insertion
evaluation topics into assembly episode status outputs.

```bash
ros2 launch adaptive_assembly_episode assembly_episode_supervisor.launch.py
```

The node only subscribes and publishes status. It does not call services,
command trajectories, publish target poses, or trigger recovery. See
[`docs/assembly_episode_supervisor.md`](../../docs/assembly_episode_supervisor.md)
for the topic contract and validation commands.
