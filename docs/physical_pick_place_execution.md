# Simulator-only physical pick-place execution

PR66 adds `physical_pick_place_executor_node`, a simulator-only executor that
consumes the PR65 multi-stage arm trajectory exports and interleaves PR63
gripper bridge commands. PR67 adds optional simulator-only contact, lift, and
slip verification after gripper close and after the lift stage. The executor
now waits for `/physical_grasp_preflight_status` by default before sending arm
goals, so the physical world, object pose observer, contact topics, and
kinematic attach separation are checked before execution starts.

The physical profile enables `open_gripper_before_first_arm_stage`, so the
default sequence is:

```text
initial gripper open -> pre_grasp -> grasp -> close gripper -> grasp verification
          -> lift -> lift/slip verification
          -> pre_place -> place -> open gripper -> retreat
```

The initial open is a command-ID-correlated state-machine transition, not a
sleep or an assumption based on the URDF initial position. Rejection, abort,
cancel, or timeout terminates execution before `pre_grasp`. Set
`open_gripper_before_first_arm_stage=false` only for focused compatibility
tests that require the former start sequence.

## Linear grasp planning and immutable plan generation

The physical sequence planner uses the normal `ompl` pipeline for
`pre_grasp`, `lift`, `pre_place`, `place`, and `retreat`. Only `grasp` uses
`planning_pipeline_id=pilz_industrial_motion_planner` and `planner_id=LIN`.
That stage has a `0.002 m` position tolerance, `0.01 rad` orientation
tolerance, and `0.05` maximum velocity and acceleration scaling. A Pilz
failure is terminal for that planning attempt; the physical path does not
fall back to OMPL for `grasp`, and `place` is not a linear stage.

The LIN result is independently checked with FK for every trajectory
waypoint of `assembly_tcp`. The validator measures Cartesian path length,
lateral and orientation deviation, endpoint error, and projected progress
against the direct segment from the successful `pre_grasp` final state to the
requested grasp pose. It rejects lateral deviation above `0.002 m`,
orientation deviation above `0.01 rad`, endpoint position error above
`0.002 m`, endpoint orientation error above `0.01 rad`, path-length ratio
above `1.02`, non-monotonic progress, or endpoint overshoot.

Every planning attempt copies all six fresh poses into one immutable,
timestamp-coherent snapshot. Trajectories remain local candidates until all
six stages and the LIN geometry check succeed, so a failed attempt publishes
no executable trajectory. Success publishes one trajectory per stage and
then one volatile `/assembly_sequence_plan_lock_status` `locked` event with a
single `plan_id`. Later pose updates are diagnosed and ignored; restarting the
single-shot planner begins another episode.

The physical executor requires that volatile lock, an exactly matching stage
sequence, and all six trajectories. DDS delivery order does not matter: the
lock may precede the last trajectory or follow the complete set. At start the
executor copies the buffered set into its immutable execution dictionary and
includes the locked `plan_id` in execution statuses. The generic executor
default remains `require_plan_lock=false` for non-physical compatibility.

`physical_target_planning_scene_node` transforms the Gazebo-derived
`/target_pose` into `panda_link0` and applies `target_object` as a MoveIt
cylinder with radius `0.035 m` and height `0.10 m`. Its ACM permits target
contact with exactly `panda_leftfinger` and `panda_rightfinger`; contact with
the hand, TCP, and all arm links remains collision-checked. On plan lock it
retains the last applied scene pose and stops applying later target updates.
This freezes only the planning-scene snapshot: it does not alter, attach, or
freeze the actual Gazebo model or its physics.

The ACM also retains one unrelated physical-mount allowance between
`panda_link0` and `work_table`, whose collision meshes overlap at the fixed
base/table interface. This does not broaden the target-object allowlist.

These contracts prove a bounded, collision-aware grasp approach and a
single planner/executor generation. They do not prove gripper contact, a
successful grasp or lift, placement, or socket insertion. Payload attachment
and final Gazebo-state placement verification remain future work.

It subscribes to `moveit_msgs/msg/RobotTrajectory` stages on:

- `/pre_grasp_trajectory`
- `/grasp_trajectory`
- `/lift_trajectory`
- `/pre_place_trajectory`
- `/place_trajectory`
- `/retreat_trajectory`

The stage list is configurable with `stage_names`, defaulting to
`pre_grasp,grasp,lift,pre_place,place,retreat`. The `close_after_stage` and
`open_after_stage` parameters default to `grasp` and `place`; both stages must
be present and `open_after_stage` cannot come before `close_after_stage`.

Arm trajectories are validated before execution. By default, the executor
requires non-empty trajectories with exactly `panda_joint1` through
`panda_joint7`, finite values, valid point field lengths, and non-negative
strictly increasing `time_from_start`. Outgoing arm goals normalize the
trajectory header stamp to zero before sending to the simulated
`/panda_arm_controller/follow_joint_trajectory` action.

Gripper commands are published to `/gripper_command` as semicolon-delimited
`std_msgs/msg/String` messages:

```text
event=command;command=open;source=physical_pick_place_executor;stage=initial_open;simulated=true;real_hardware=false
event=command;command=close;source=physical_pick_place_executor;stage=grasp;simulated=true;real_hardware=false
event=command;command=open;source=physical_pick_place_executor;stage=place;simulated=true;real_hardware=false
```

The executor waits for `/physical_gripper_command_status` from
`gripper_action_bridge_node`. `event=success` advances only when `result` is
`success` or `contact_limited_success`. The latter is valid only for physical
close and is logged distinctly. If
`require_gripper_success=true`, `event=failure;command=<active_command>` fails
the run. Its classified `gripper_result` and `gripper_reason` are retained in
stage and terminal diagnostics. Per-command IDs prevent retained results from
an old close from satisfying a later operation.

After either permitted close result, the executor publishes:

```text
event=request;verification=grasp;stage=grasp;source=physical_pick_place_executor;simulated=true;real_hardware=false
```

This is deliberately not whole-task success. `contact_limited_success` only
permits the existing grasp verification, lift, lift/slip verification,
placement, opening, and retreat stages to continue.

After the lift arm stage succeeds, it publishes:

```text
event=request;verification=lift;stage=lift;source=physical_pick_place_executor;simulated=true;real_hardware=false
```

The executor waits up to `verification_timeout_sec` for
`/grasp_verification_status`, `/grasp_verified`, or `/lift_verified`. Required
verification failures terminate with `grasp_verification_failed`,
`grasp_verification_timeout`, `lift_verification_failed`, or
`lift_verification_timeout`. Set `require_grasp_verification=false` and
`require_lift_verification=false` for message-only dry runs that do not depend
on Gazebo contact or pose topics.

Published executor topics:

- `/physical_pick_place_execution_status` (`std_msgs/msg/String`, retained)
- `/physical_pick_place_execution_success` (`std_msgs/msg/Bool`, retained)
- `/physical_pick_place_execution_duration_ms` (`std_msgs/msg/Float64`, retained)
- `/physical_pick_place_stage_status` (`std_msgs/msg/String`, transient local)

All status strings include `mode=physical_pick_place` and
`real_hardware=false`.

When the simulated arm controller accepts a stage goal, the stage topic
publishes an explicit acceptance transition:

```text
event=accepted;mode=physical_pick_place;stage=pre_grasp;stage_index=0;action=arm;controller_goal_accepted=true;real_hardware=false
```

This distinguishes an attempted send from evidence that the simulated
`panda_arm_controller` accepted the trajectory.

When preflight is required and fails, the terminal execution status uses:

```text
event=failure;mode=physical_pick_place;stage=preflight;reason=physical_grasp_preflight_failed;preflight_reason=object_pose_unavailable;execution=false;simulated_execution_only=true;real_hardware=false
```

`physical_grasp_preflight_failed` remains the stable executor-level reason.
The `preflight_reason` field preserves the concrete reason received from
`/physical_grasp_preflight_status`, and both the preflight node and executor
log that detail on terminal failure. Inspect the observer and preflight gates
with:

```bash
ros2 topic echo /gazebo_target_object_pose_status --once
ros2 topic echo /gazebo_target_object_pose_available --once
ros2 topic echo /physical_grasp_preflight_status --once
```

The compatibility launch argument `require_target_entity_exact_match` remains
available, but it only affects Pose_V/TFMessage input. The physical launch's
dedicated `pose_stamped` mode does not perform entity-name matching. The
observer's standalone strict-match default is unchanged.

If trajectories and joint state are ready but no preflight success arrives
within the configured timeout, the reason is
`physical_grasp_preflight_timeout`.

Launch the composed simulator-only entry point:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_physical_pick_place_execution.launch.py
```

For the full Gazebo path, prefer:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py
```

The full physical launch composes Gazebo, `gz_ros2_control`, MoveIt planning,
contact bridges, pose observation, preflight, grasp verification, and the
physical executor. In this path, Gazebo is the only `/controller_manager`
provider. The nested Panda planning launch is run with
`use_standard_panda_demo:=false`, so it starts `move_group` directly and does
not include `moveit_resources_panda_moveit_config/launch/demo.launch.py`, the
MoveIt resources fake `ros2_control_node`, or fake Panda controller spawners.

## Physical TCP and pose-topic contract

The full physical path explicitly propagates
`end_effector_link:=assembly_tcp` through every nested launch. The sequence
planner validates that this link exists, configures it as the MoveIt
end-effector link, applies each pose target to it explicitly, and reports it in
planning status. An invalid link fails as
`configured_end_effector_link_invalid`; the physical path does not fall back
to the SRDF's implicit `panda_link8` arm tip.

`assembly_tcp` is fixed to `panda_hand` at translation `(0, 0, 0.1034) m` and
identity rotation. The finger joint origins are at hand Z `0.0584 m`, and the
opposing near-planar collision contact pads are centered `0.045 m` farther
along +Z. Symmetric finger travel along hand +/-Y leaves the midpoint fixed
while the fingers move. The adapters' normalized quaternion
`(x=1, y=0, z=0, w=0)` points the TCP +Z axis downward for the vertical
cylindrical grasp.

The physical public pose meanings are:

- `/target_pose`: Gazebo-observed target-cylinder geometric center in `world`;
- `/panda_pre_grasp_pose`, `/panda_grasp_pose`, `/panda_lift_pose`,
  `/panda_pre_place_pose`, `/panda_place_pose`, and `/panda_retreat_pose`:
  desired `assembly_tcp` poses in `panda_link0`.

The cylinder model pose is its center and its length is `0.10 m`. The full
physical launch uses `target_reference_z_offset:=0.0`, and the physical task
profile uses `grasp_height_offset: 0.0`, so the grasp TCP Z equals the observed
cylinder-center Z. Pre-grasp and lift remain `0.20 m` above that center. The
fixed socket's `socket_z: 0.10` is also an object/TCP center and its physical
`place_height_offset` is `0.0`. Generic visual and plan-only profiles retain
their previous defaults. The physical planning wrapper uses `0.005 m` and
`0.005 rad` planning tolerances. The tighter physical orientation tolerance
ensures the actual pre-grasp endpoint can seed the independently validated
`0.01 rad` LIN orientation bound. These remain below the independent `0.02 m`
and `0.10 rad`
runtime acceptance tolerances; generic planner defaults remain unchanged.

The MoveIt planner nodes remain plan-only: their status messages retain
`execution=false`, and they only publish six `RobotTrajectory` messages. The
separate `physical_pick_place_executor_node` is the component that sends those
trajectories to the simulator-only
`/panda_arm_controller/follow_joint_trajectory` action. A planning success is
therefore not, by itself, evidence of controller acceptance or arm motion.

It also sets `launch_fake_object_pose_node:=false`. A model-local Gazebo
`PosePublisher` publishes one simulator pose on `/model/target_object/pose` at
30 Hz. `ros_gz_bridge` maps it to `/gazebo_target_object_pose_raw` as
`geometry_msgs/msg/PoseStamped`, and the retained
`gazebo_entity_pose_observer_node` publishes the simulated dynamic
`target_object` model-center pose on `/gazebo_target_object_pose`. This
dedicated path does not depend on SceneBroadcaster entity names. The
`gazebo_target_pose_adapter_node` publishes the task input on `/target_pose`.
The adapter preserves XY and orientation and adds the physical launch's
`target_reference_z_offset:=0.0` to Z, retaining the cylinder center. The adapter
does not synthesize a target before an observation arrives. Its
`output_frame_id:=world` setting overrides only the frame label and does not
perform a TF transform.

The fixed socket object/TCP center target remains `(0.62, -0.18, 0.10)`. This
contract does not demonstrate completed placement or insertion.

Ordinary and plan-only demos retain `launch_fake_object_pose_node:=true` and
therefore keep their existing deterministic fake-perception behavior. In the
full physical launch, inverse fake-source and adapter conditions ensure that
only one intended publisher supplies `/target_pose`.

For a running full physical demo, inspect the source relationship with:

```bash
ros2 topic info -v /target_pose
ros2 topic echo /gazebo_target_object_pose --once
ros2 topic echo /target_pose --once
```

The expected relationship is equal X/Y and
`target_pose.z = gazebo_target_object_pose.z + target_reference_z_offset`.

The full entry point defaults to `use_sim_time:=true` and expects the `/clock`
bridge from Gazebo. This typed Boolean value is propagated to the direct
`move_group`, sequence planner, timestamped-pose pipeline, pose adapters, and
physical execution, observation, contact, preflight, and verification nodes.
This keeps MoveIt's current-state freshness checks in the same time domain as
Gazebo `/joint_states`. Reusable plan-only launches keep `use_sim_time:=false`
by default and therefore do not require a `/clock` publisher.

For a running full physical demo, confirm the shared time domain with:

```bash
ros2 topic echo /clock --once
ros2 param get /move_group use_sim_time
ros2 param get /assembly_sequence_planning_node use_sim_time
ros2 param get /physical_pick_place_executor_node use_sim_time
ros2 param get /physical_target_object_pose_observer use_sim_time
```

For a message-only executor dry run, provide trajectories and joint states with
the validation script:

```bash
python3 scripts/check_physical_pick_place_executor_dry_run.py
```

For the bounded PR75 acceptance check, build and source the workspace, then
run:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/check_full_physical_pick_place_arm_motion.py
```

The checker launches the full demo with server-only Gazebo, requires active
controllers, valid Panda joint states, the dedicated target pose and preflight,
successful six-stage planning, six non-empty trajectories, the executor's
`pre_grasp` send and acceptance transitions, and at least `0.01 rad` motion in
one arm joint. It stops at proof of initial arm motion; it does not claim grasp,
lift, placement, or insertion success. On failure it prints the concrete
missing or rejected condition and writes `launch.log` plus
`status_topics.log` under `runs/pr75_arm_motion_<timestamp>/`.

For bounded Cartesian TCP verification, run:

```bash
python3 scripts/check_full_physical_pick_place_tcp_contract.py
```

The headless checker requires active Gazebo controllers, real Gazebo joint
states and target observation, successful physical preflight, six nonempty
plans, accepted and successful `pre_grasp` and `grasp` arm goals, and the
authoritative runtime TF `panda_link0 -> assembly_tcp` within `0.02 m` and
`0.10 rad` of both targets. Orientation error is the shortest relative
quaternion angle. It exits once grasp TCP evidence is complete, before a later
gripper-close or contact failure can affect this bounded result. It writes
`launch.log`, `status_topics.log`, `tcp_targets.csv`, `tcp_actual.csv`, and
`result.json` under `runs/tcp_contract_<timestamp>/`.

Model parity, joint-space controller success, and Cartesian TCP success are
distinct evidence. This checker supplies the third by measuring runtime TF;
it does not claim gripper closure, contact grasp, lift, placement, retreat, or
insertion success.

See `docs/gazebo_contact_grasp_verification.md` for the Gazebo contact sensor
plumbing and verifier status schemas. This remains simulator-only. It does not
add force control, tactile feedback, camera perception, MoveIt Servo, learned
grasping, kinematic-attachment success claims, or real hardware execution.
