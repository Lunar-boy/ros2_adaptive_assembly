# Adaptive Assembly Benchmark

This package provides `contact_lite_insertion_evaluator_node`, a deterministic
final-pose geometric evaluator. It compares configurable target and achieved
`PoseStamped` topics and publishes insertion success, position error,
orientation error, and a semicolon-delimited status.

The default achieved pose is the planned `/panda_assembly_pose`; status
therefore reports `achieved_pose_source=planned_pose`. This is not force
control, tactile sensing, contact-rich peg-in-hole simulation, or real hardware
execution.

Launch through bringup:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_contact_lite_insertion_benchmark.launch.py
```

Focused validation:

```bash
bash scripts/check_contact_lite_insertion_topics.sh
python3 scripts/check_contact_lite_insertion_success_path.py
python3 scripts/check_contact_lite_insertion_failure_path.py
bash scripts/check_contact_lite_insertion_report_export.sh
```

See `docs/contact_lite_insertion_benchmark.md` for launch and validation
commands.
