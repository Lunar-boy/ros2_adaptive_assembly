# Roadmap

## Near-term engineering priorities:

1. Add gripper abstraction and logical grasp lifecycle.
2. Synchronize Gazebo target object pose with the adaptive perception pipeline.
3. Add richer benchmark reports with real recorded tables and plots.
4. Add screenshots or GIFs for RViz, PlanningScene, and Gazebo workcell demos.
5. Extend fake perception toward AprilTag/ArUco or RGB-D based target pose estimation.
6. Introduce task-level retry orchestration that consumes recovery supervisor actions.
7. Add contact-aware insertion approximation or force-control extension points.

## Longer-term extensions:

- UR5e or xArm support for more industrial robot variants;
- Nav2 + mobile manipulator integration;
- learned grasp or assembly target proposal;
- HPC-based batch evaluation over randomized target poses and obstacle layouts;
- hardware interface preparation for real robot experiments.


## PR
- PR1: fake perception node
- PR2: task pose generation node
- PR3: bringup launch for the non-MoveIt pipeline
- PR4: validation scripts and documentation cleanup
- PR5: optional Panda MoveIt2 demo bringup
- PR6: plan-only MoveIt2 pre-grasp planning bridge
- PR7: Panda pre-grasp pose adapter for robot-aware planning targets
- PR8: frame-aware Panda pre-grasp pose adapter
- PR9: static PlanningScene collision objects for Panda planning demo
- PR10: planning diagnostics and timing topics
- PR11: planning diagnostics CSV benchmark recorder
- PR12: reproducible seeded planning benchmark profile
- PR13: deterministic benchmark profile suite and CSV comparison tools
- PR14: dynamic target collision object in PlanningScene
- PR15: dynamic target PlanningScene toggle and A/B benchmark profiles
- PR16: dynamic target PlanningScene clear/reset service
- PR17: static PlanningScene clear/reapply services
- PR18: unified PlanningScene reset workflow
- PR19: Markdown benchmark report export
- PR20: configurable MoveIt2 planner settings for benchmarks
- PR21: planner-settings benchmark profiles
- PR22: TF2-based Panda pose adapter with status diagnostics
- PR23: planning request guard and safety filter
- PR24: PlanningScene object audit tool
- PR25: simple RViz marker visualization
- PR26: plan-only Panda pre-grasp and assembly sequence planning
- PR27: deterministic fixed-start assembly sequence planning fallback
- PR28: deterministic known-reachable assembly sequence profile
- PR29: assembly sequence stage-level diagnostics
- PR30: successful assembly sequence trajectory export
- PR31: message-only dry-run sequence execution
- PR32: closed-loop recovery state machine and deterministic actions
- PR33: optional simulator-only Gazebo/ros2_control execution bridge
- PR34: full Gazebo workcell bringup
- PR35: ros2_control success-path execution
- PR36: gripper abstraction and logical grasp lifecycle
- PR37: recovery action orchestration and bounded simulated retry loop (complete)
