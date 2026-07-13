# Robot model parity diagnostic

The full physical pick-place demo plans and executes against one canonical
Panda kinematic description:

- MoveIt expands `adaptive_assembly_sim/urdf/panda.urdf.xacro`;
- Gazebo expands a local simulator wrapper that includes that same project
  xacro before adding simulator-only extensions and spawning the robot.

The project xacro includes the upstream Panda description and defines the
project-owned `assembly_tcp` exactly once. Its fixed parent is `panda_hand`,
with `xyz="0 0 0.1034"` and `rpy="0 0 0"`. The finger joint origins are at
hand Z `0.0584 m`; the near-planar opposing collision contact pads are centered
`0.045 m` farther along +Z. Because the fingers translate symmetrically along
hand +/-Y, the midpoint remains fixed while the fingers move.

Matching controller joint names are not enough to make those descriptions
equivalent. A joint trajectory can be accepted and completed by
`panda_arm_controller` while the simulated end effector reaches a different
Cartesian transform. Joint-space controller success confirms that the named
joints followed a command; it does not confirm that MoveIt and Gazebo assign
the same link transforms, axes, limits, chain topology, or tool frame to those
joint values.

## Run the current-model comparison

Build and source the workspace, then use the installed repository preset:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select adaptive_assembly_sim
source install/setup.bash
ros2 run adaptive_assembly_sim check_robot_model_parity \
  --current-panda-models
```

The preset resolves the installed project package share through the ament index. It uses
`panda_link0` as both base links and `panda_link8` as both tool endpoints. It
never downloads a dependency and does not require a ROS graph, MoveIt process,
Gazebo process, RViz, or graphical environment.

Verify the explicit TCP chain and FK with the same structural tolerances:

```bash
ros2 run adaptive_assembly_sim check_robot_model_parity \
  --current-panda-models \
  --reference-tool-link assembly_tcp \
  --candidate-tool-link assembly_tcp
```

Two explicit expanded URDF or xacro paths can also be supplied:

```bash
cd ~/ros2_adaptive_assembly_ws
source /opt/ros/jazzy/setup.bash
python3 -m adaptive_assembly_sim.robot_model_parity \
  /path/to/reference.urdf.xacro \
  src/adaptive_assembly_sim/urdf/panda_gazebo_ros2_control.urdf.xacro \
  --reference-tool-link panda_link8 \
  --candidate-tool-link panda_link8
```

Xacro sources are expanded with the installed `xacro` executable using a
subprocess argument list. Repeat `--reference-xacro-arg KEY:=VALUE` or
`--candidate-xacro-arg KEY:=VALUE` when a model needs mappings. Use
`--tool-link` when both models intentionally use the same tool link.

The default FK check evaluates three deterministic, non-zero Panda joint
samples. Each sample is inside the intersection of the two models' declared
joint limits. Custom samples use this repeatable form:

```text
--fk-sample sample_name:panda_joint1=0.2,panda_joint2=-0.5,...
```

Use `--no-fk` for a structural-only comparison. Translation, rotation, axis,
and joint-limit tolerances each default to `1e-6` and have corresponding CLI
options.

## Verdict and exit codes

The structural contract checks:

- configured base and tool link existence;
- required arm joint existence and type;
- each required joint's parent, child, origin `xyz`, origin `rpy`, axis, and
  numeric limits;
- configured base-to-tool chain topology.

The lightweight FK implementation supports fixed, revolute, continuous, and
prismatic URDF joints. For each sample it reports both tool positions, position
error in metres, and orientation error in radians. Orientation error comes
from the clamped relative-rotation trace, not component-wise RPY subtraction.

Exit codes are stable:

- `0`: structural and requested FK parity checks pass;
- `1`: one or more parity mismatches are detected;
- `2`: invalid arguments, missing input or package, malformed XML, xacro
  failure, invalid samples, or another diagnostic setup error.

## JSON output

Add `--json` for automation:

```bash
python3 -m adaptive_assembly_sim.robot_model_parity \
  --current-panda-models --json > /tmp/panda_model_parity.json
```

The module form keeps redirected JSON clean for machine validation.

The schema includes `schema_version`, `passed`, `sources`, `configured_links`,
`arm_joints`, `tolerances`, `structural_mismatches`, `fk_samples`, and
`mismatch_counts`. Structural records include a category, affected subject and
field, reference and candidate values when available, and absolute numerical
error. Output ordering is deterministic.

## Current expected result

The current-model command must return exit `0` and report `Robot model parity:
PASS`, three passing FK samples, and zero structural, FK, and total mismatches.
The Gazebo wrapper includes the canonical lower-level Panda description instead
of the MoveIt configuration's top-level xacro because the latter also emits
mock `ros2_control` systems. The wrapper therefore adds exactly one usable
`gz_ros2_control` system while retaining canonical links, joint transforms,
axes, limits, topology, meshes, collisions, inertials, hand, and mimic finger.

The default parity command proves that identical arm joint values produce the
same `panda_link0 -> panda_link8` transform. The explicit TCP command proves
the same structural and FK contract through `assembly_tcp`. Neither command
proves that an executed controller goal reached its Cartesian target; that is
the bounded runtime check documented in
`docs/physical_pick_place_execution.md`. Model parity does not prove contact
grasp success, lift, placement, insertion, or force-control behavior.
