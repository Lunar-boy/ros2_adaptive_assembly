# Gazebo contact grasp verification

PR67 adds simulator-only Gazebo contact sensing and deterministic grasp,
lift, and slip verification for the physical pick-place path.

## Gazebo wiring

The Panda finger collision links in
`src/adaptive_assembly_sim/urdf/panda_gazebo_ros2_control.urdf.xacro` include
contact sensors:

- `panda_leftfinger_contact_sensor` on `/panda_leftfinger_contact`
- `panda_rightfinger_contact_sensor` on `/panda_rightfinger_contact`

The workcell includes the Gazebo Contact system plugin. Physical verification
uses `adaptive_assembly_physical_workcell.sdf`, where `target_object` is
dynamic. The original `adaptive_assembly_workcell.sdf` keeps the target object
static for existing visual and logical demos.

Finger contact messages depend on the Panda finger collision geometry. Use the
full physical wrapper launch or pass `enable_arm_collisions:=true` when starting
Gazebo for this path. The older kinematic attach visual demo may keep arm
collisions disabled because it does not prove physical contact.

The kinematic attach path
(`adaptive_assembly_gazebo_grasp_attach_demo.launch.py`,
`logical_grasp_lifecycle_node.py`, and `gazebo_attach_detach_node.py`) remains
available for visual/logical demos. If `gazebo_attach_detach_node` is running,
the result is not a physical grasp verification run.

## Physical preflight

`physical_grasp_preflight_node` publishes retained diagnostics on
`/physical_grasp_preflight_status` before the physical executor sends arm goals.
It checks that the physical Gazebo pose topic is used, target object pose is
available, kinematic attachment is not active, raw left/right finger contact
topics have been observed, and `/grasp_contact_status` has been observed.

Status strings use:

```text
event=success|failure|waiting;mode=physical_grasp_preflight;physical_world=true|false;object_pose_available=true|false;kinematic_attach_active=true|false;left_contact_observed=true|false;right_contact_observed=true|false;contact_status_observed=true|false;reason=<reason>;simulated_only=true;real_hardware=false
```

Important failure reasons include `kinematic_attach_node_active`,
`left_contact_topic_unobserved`, `right_contact_topic_unobserved`,
`contact_status_topic_unobserved`, `object_pose_unavailable`, and
`wrong_pose_info_topic`.

## Contact status node

`gazebo_grasp_contact_status_node` consumes bridged
`ros_gz_interfaces/msg/Contacts` messages and publishes retained status:

- `/left_gripper_contact_detected` (`std_msgs/msg/Bool`)
- `/right_gripper_contact_detected` (`std_msgs/msg/Bool`)
- `/both_gripper_contacts_detected` (`std_msgs/msg/Bool`)
- `/grasp_contact_status` (`std_msgs/msg/String`)

Status strings use:

```text
event=success|failure|waiting|skipped;mode=gazebo_grasp_contact_status;left_contact=true|false;right_contact=true|false;both_contacts=true|false;target_object_name=target_object;reason=<reason>;simulated_only=true;real_hardware=false
```

Reasons include `left_contact_stale`, `right_contact_stale`,
`no_left_contact`, `no_right_contact`, `no_target_object_contact`,
`unsupported_contact_message`, and `simulated_only_false`.

## Grasp and lift verifier

`grasp_verifier_node` consumes contact status, gripper bridge outputs, and
`/gazebo_target_object_pose`. Requests are semicolon key/value
`std_msgs/msg/String` messages on `/grasp_verification_request`:

```text
event=request;verification=grasp;stage=grasp;real_hardware=false
event=request;verification=lift;stage=lift;real_hardware=false
event=reset;reason=<reason>;real_hardware=false
```

On grasp requests, the verifier checks gripper command success, gripper closed
state, both finger contacts when required, and object pose availability. A
successful grasp stores the current target object pose as the lift baseline.

On lift requests, the verifier computes:

- `lift_delta_m = current_z - baseline_z`
- `slip_distance_m = horizontal Euclidean distance from the baseline`

Lift verification succeeds only when `lift_delta_m >= min_lift_delta_m` and
`slip_distance_m <= max_slip_distance_m`.

Verifier outputs:

- `/grasp_verification_status` (`std_msgs/msg/String`)
- `/grasp_verified` (`std_msgs/msg/Bool`)
- `/lift_verified` (`std_msgs/msg/Bool`)
- `/grasp_slip_distance_mm` (`std_msgs/msg/Float64`)

Status strings use:

```text
event=success|failure|waiting|reset;mode=grasp_verifier;verification=grasp|lift;stage=grasp|lift;grasp_verified=true|false;lift_verified=true|false;left_contact=true|false;right_contact=true|false;both_contacts=true|false;gripper_closed=true|false;object_pose_available=true|false;lift_delta_m=<float>;slip_distance_m=<float>;reason=<reason>;simulated_only=true;real_hardware=false
```

## Launch example

Start the full simulator-only physical demo:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_full_physical_pick_place_demo.launch.py
```

Or start Gazebo with the physical workcell, then launch the pick-place
execution:

```bash
cd ~/ros2_adaptive_assembly_ws
source install/setup.bash
ros2 launch adaptive_assembly_sim adaptive_assembly_panda_gazebo.launch.py \
  world:=install/adaptive_assembly_sim/share/adaptive_assembly_sim/worlds/adaptive_assembly_physical_workcell.sdf \
  world_name:=adaptive_assembly_physical_workcell \
  enable_arm_collisions:=true
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_physical_pick_place_execution.launch.py
```

The physical pick-place launch starts the contact bridge, contact status node,
target object pose observer, grasp verifier, gripper bridge, and executor by
default. It also starts the preflight node by default and the executor waits for
preflight success before sending arm goals. Verification can be disabled for dry
runs with:

```bash
ros2 launch adaptive_assembly_bringup \
  adaptive_assembly_physical_pick_place_execution.launch.py \
  require_physical_grasp_preflight:=false \
  require_grasp_verification:=false \
  require_lift_verification:=false
```

## Debug checklist

```bash
ros2 topic echo /physical_grasp_preflight_status
ros2 topic echo /physical_pick_place_stage_status
ros2 topic echo /physical_pick_place_execution_status --once
ros2 topic echo /grasp_contact_status
ros2 topic echo /grasp_verification_status
ros2 topic echo /physical_gripper_command_status
ros2 topic echo /gazebo_target_object_pose_status
ros2 topic echo /panda_leftfinger_contact
ros2 topic echo /panda_rightfinger_contact
```

If `/physical_grasp_preflight_status` reports
`reason=kinematic_attach_node_active`, stop the kinematic attach demo before
using the physical verification launch.

## Limitations

- Simulator-only; real hardware remains unsupported.
- Contact sensing depends on Gazebo physics and contact messages.
- No force control.
- No tactile hardware.
- No real robot.
- No camera perception.
- No learned grasping.
- No benchmark CSV or report export yet; that belongs to PR68.
- Physical verification does not use kinematic attachment as success evidence.
