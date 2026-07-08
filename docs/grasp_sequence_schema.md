# Explicit Grasp Sequence Schema

The task node exposes a deterministic, simulator-only planning interface:

```text
/grasp_candidates -> /selected_grasp_pose (= /grasp_pose legacy alias)
                  -> /pre_grasp_pose -> /lift_pose
/object_place_pose -> /pre_place_pose -> /place_pose -> /retreat_pose
```

`/grasp_candidates` is a `std_msgs/String` schema instead of a custom ROS
message, avoiding message generation in this PR. Its stable fields are
`event`, `count`, `selected_index`, `frame_id`, and `candidates`; candidate
values use six decimal places. `/grasp_sequence_status` reports acceptance or
small-motion skipping, selected and place coordinates, mode, and explicit
`execution=false`, `simulated_only=true`, and `real_hardware=false` fields.

This schema is not physical grasping. It does not add gripper control,
contact-rich insertion, Gazebo physics grasping, or real hardware execution.
`/assembly_pose` remains the backward-compatible planning target: it is an
alias of `/place_pose` in `fixed_socket` mode and retains its existing
target-offset behavior in `target_offset` mode.
