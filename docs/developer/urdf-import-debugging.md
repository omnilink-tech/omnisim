# URDF Import Debugging

This note is the fast path for debugging robot models loaded through the URDF importer.

The key constraint is that URDF and Webots do not describe every joint and collision detail in exactly the same way. A model can look "almost right" while still losing important information at import time. The importer now exposes that loss instead of hiding it.

## What To Check First

- Joint axes: URDF defines the joint axis in the joint frame, while Webots expects the axis in the parent solid frame. The importer rotates the axis through the joint origin's RPY before emission.
- Collision coverage: a link with one supported collision emits a single `boundingObject`; a link with multiple supported collisions emits a `boundingObject Group { children [...] }`.
- Mesh geometry: visual and collision meshes are imported when the path resolves (supports `package://`, `file://`, absolute, and relative paths). Unresolved meshes are skipped with a warning. ⚠️ **Resolving is not decoding** — a Draco-compressed glTF resolves, and the engine's assimp build refuses it whole; see the Static Import Report section and [newton-physics-backend.md](../guide/newton-physics-backend.md#known-gaps--what-does-not-work-yet). Webots cannot apply a non-unit scale inside a `boundingObject`, so collision-mesh scale is dropped (visual-mesh scale is honoured via a `Transform` wrapper).
- Inertia fidelity: the URDF `<inertia ixx ixy ixz iyy iyz izz>` is passed through to Webots' `Physics.inertiaMatrix` when the tensor is positive definite. Non-PD tensors are rejected with a warning and the boundingObject-derived inertia is used instead. `centerOfMass` is always emitted alongside the matrix. The `<inertial>` origin's `rpy` is folded into the tensor, which is emitted in the link's axes (I' = R I R^T); until 2026-10-15 it was dropped, as plain MuJoCo 3.11's URDF import still drops it. `OMNISIM_URDF_INERTIAL_RPY=0` restores the unrotated tensor. Pins: `tests/dev/test_urdf_import_inertial_rpy.py`, `tests/test_newton_urdf_inertial_rpy.py`.
- Sensors: `<gazebo reference="LINK"><sensor>` blocks (modern Gazebo) are mapped to Webots devices — `imu` → `InertialUnit` + `Gyro` + `Accelerometer`, `gps` → `GPS`, `camera`/`depth` → `Camera`, `ray`/`gpu_ray`/`lidar` → `Lidar`. Legacy `<gazebo><plugin filename="libhector_gazebo_ros_imu.so">` / `libhector_gazebo_ros_gps.so` patterns are also recognized so unmodified Clearpath URDFs (Jackal, Husky) import with their full sensor suite.
- Joint damping and friction: imported when present and materially affect whether parts settle cleanly.

## Static Import Report

Run the developer importer with a JSON report before loading the model in the simulator:

```powershell
python scripts/dev/urdf_import.py path\to\robot.urdf --report urdf-report.json --strict > robot.wbt.snippet
```

The report includes:

- root-link selection
- per-link counts (`supported_visual_count`, `supported_collision_count`), `has_inertia_matrix` flag, and any unresolved mesh paths
- ⚠️ **mesh DECODABILITY, not just existence** (`mesh_status_counts`, and per link `undecodable_meshes_visual` / `undecodable_meshes_collision` / the `unverified_*` pair). Until 2026-09-11 the only mesh question asked was "is there a file at the resolved path?", and on the public urdfeus gallery (718 models, `github.com/iory/urdfeus` @ `a5d094b`, 8,048 `<mesh>` references) that reported **0 problems** while the engine refused all 4,024 of its `.glb` files — every one is `KHR_draco_mesh_compression`, which OmniSim's assimp build cannot decode — and gave the affected links a placeholder sphere collider. The three states are kept apart: `missing` (no file), `undecodable` (a file that provably cannot be read: a required glTF extension the reader does not implement, an empty file, a truncated GLB/STL, a `.gltf` whose `.bin` sidecar is absent), and `unverified` (a required extension nobody has checked). `ok` means "nothing provably wrong", not "assimp will succeed" — a full answer needs a real decoder, which an offline checker does not have. The engine now raises an **ERROR** when a declared collision mesh reaches the solver with no triangles (`OMNISIM_STRICT_COLLISION_MESH=0` downgrades it), so this report is the cheap way to find out first.
- per-joint `axis_joint_frame` and `axis_parent_frame`
- imported limits, damping, and friction
- per-sensor kind, target link, and update rate

`--strict` exits non-zero when the report contains warnings. That makes it suitable for agent loops and CI gates. The Jackal URDF (`projects/robots/clearpath/jackal_description/urdf/jackal.urdf`) is the reference URDF that exercises all four import paths — primitive collisions, mesh visuals, full inertia tensors, and legacy plugin-form sensors — and should always pass `--strict`.

## Runtime Joint Logging

When loading through native `URDFRobot { ... }` expansion, enable importer logging:

```powershell
$env:OMNISIM_URDF_DEBUG = "1"
%OMNISIM_HOME%\msys64\mingw64\bin\omnisim-bin.exe path\to\world.omniworld --stdout --stderr
```

The parser logs:

- robot root-link selection
- per-joint origin xyz/rpy
- raw axis from the URDF joint frame
- transformed axis emitted to the Webots joint
- imported limits, damping, friction, and max velocity when present

Search the log for `URDF_DEBUG`.

## Recommended Agent Workflow

1. Generate the JSON report with `scripts/dev/urdf_import.py --report --strict`.
2. Resolve report warnings that change physics behavior first: missing collision primitives, missing limits, missing damping, and unsupported geometry.
3. Load the model through `URDFRobot { ... }` with `OMNISIM_URDF_DEBUG=1`.
4. Compare each suspicious joint's `axis_joint_frame` and `axis_parent_frame`. If the joint origin has rotation, they should usually differ.
5. If the model still does not settle, inspect the imported collision primitives and inertial blocks before adjusting controller code.

## Current Limits

- `<transmission>` blocks are ignored — motors come from the joint type. Mechanical reduction must be applied in the controller.
- ROS `xacro` is not expanded as a full template language. However, a common partial-expansion pattern — URDFs that ship with literal `${variable}` placeholders that were meant to be substituted by an outer xacro include — is handled by stripping any unexpanded `${...}` pattern at parse time, with a one-time warning. TurtleBot3's `turtlebot3_burger.urdf`, `turtlebot3_waffle.urdf`, and `turtlebot3_waffle_pi.urdf` rely on this: their joint and link names literally contain `${namespace}` because they were designed for inclusion under a namespaced xacro parent.
- COLLADA (`.dae`) meshes: two crash causes are fixed, so a `.dae` that still kills the engine is a new bug, and worth reporting with the file. (1) A file whose geometry several nodes instance (Blender exports do this) overran `OmMesh`'s arrays until 2026-10-09: Leo Rover's `Chassis.dae`, 27 geometries in 35 instances, crashed every load with an access violation and no log line. Pin: `tests/test_mesh_collada_instanced_geometry.py`. (2) A SketchUp-authored file whose submeshes carry vertices and no normals; such submeshes are skipped with a warning. ⚠️ **The worked example this bullet used to give is gone**: TurtleBot3 waffle's `meshes/sensors/r200.dae` was removed on 2026-08-24 for licence reasons (it modelled an **Intel** RealSense R200, and ROBOTIS' Apache-2.0 cannot convey rights in Intel's design), so there is no longer a reproducer in the tree. The engine-side guard it prompted stays — `OmMesh.cpp` null-checks `mesh->mNormals` and skips such submeshes with a warning — and it was never specific to that file. (3) **Placement, fixed 2026-10-15:** `OmMesh` (every `Mesh`, so every mesh collider) and `OmCadShape` composed a node's transform leaf-first (`node * parent * ... * root`, inherited from Webots) instead of `root * ... * node`. A Collada file always has two transforms in that chain — the importer's up-axis rotation on the root, and the node of any geometry instanced more than once, which `aiProcess_OptimizeGraph` keeps — so instanced geometry under a translated node landed in the wrong place, with no warning. Rover Robotics' `mega_wheels.dae` came out 26.5 mm off its axle (the MEGA bounced ±2.7 cm on its wheels alone and, as published, stalled at half speed and tilted 3.75°); `mega_payload.dae` up to 0.96 m out of place. None of the 82 Collada files in this repository moves. `OMNISIM_MESH_NODE_TRANSFORM_FIX=0` restores the old product. Pins: `tests/test_mesh_collada_node_transform_order.py` (reads the collider back from the solver model) and `tests/test_mesh_node_transform_order_source.py`. (4) **Shading, fixed 2026-10-23:** the same two walks mapped normals with the full 4×4 (translation added, no inverse-transpose), so instanced geometry under a translated or non-uniformly scaled node shaded wrong — a sphere 20 m off its node rendered as a flat disc. Visual only. `OMNISIM_MESH_NORMAL_MATRIX_FIX=0` restores it. Pins: `tests/test_mesh_normal_matrix.py`, `tests/test_mesh_normal_matrix_render.py`. (5) **`CadShape` read only the root node, fixed 2026-10-23:** its node walk never queued children, so a file whose geometry several nodes instance rendered nothing as a `CadShape` (a `Mesh` of the same file was fine). Same pins.
- Collision-mesh scale (`<mesh scale="...">` inside `<collision>`) is dropped because Webots' `boundingObject` does not honour `Transform.scale`. Visual-mesh scale is honoured.
- Sensor noise / drift parameters from `<gazebo><plugin>` blocks are not propagated — only the device type and the link it attaches to. Tune noise on the Webots side once imported if it matters for your demo.
- Multi-shape collision is supported via `Group`, but Webots auto-computes inertia from the *first* shape only unless an explicit `Physics.inertiaMatrix` is supplied.
- A driven joint with no `<limit>` (common on `continuous` wheels) imports with the `RotationalMotor` defaults: `maxTorque` **10 N·m** and `maxVelocity` **10 rad/s** (the importer writes `maxTorque` only from an `effort` attribute and `maxVelocity` only from a `velocity` one; the engine log's per-joint `[OmNewtonBackend] hinge joint ... effort=10 velLim=10` line shows it). Gazebo's DiffDrive sets wheel velocity with no torque cap, so a heavy robot that drives in Gazebo can stall here: give the joint a `<limit effort=... velocity=...>`.
- **The root link's own `<collision>` collides by default since 2026-10-17 -- unless it would hold the wheels off the floor.** The root link's collision (or, when the URDF root is an empty frame on one fixed joint such as `base_footprint`, the promoted child's) becomes the Robot's OWN `boundingObject`. Until 2026-10-17 `WorldInfo.newtonRobotColliders FALSE` (the default) kept it off the solver whenever the robot had colliding links below it, and the log told you to "declare a boundingObject" the Robot already declared. Measured on Preferred Robotics' Kachaka (chassis box underside exactly on the tyre contact plane, no caster): the published file tipped onto its nose (pitch 79°) instead of dragging the box as plain MuJoCo does. Now the engine registers it and keeps it colliding unless its lowest point, at the spawn pose, is more than 2 mm below the level the robot stands on: the lowest point shared by at least two branches below the root (each child link and its subtree: a wheel, a leg, a caster), so an arm hanging below the tyres in its zero pose does not count (an envelope that reaches under the wheels would lift every one of them); a quieted one is reported with the gap. Kachaka now drags at 0.464 of command by default (identical to the `TRUE` run), its turn makes 0.182, and with the box raised 5 mm plus casters 0.998 / 0.983. Links fixed below the root (caster, lidar, bumper) collided already. `newtonRobotColliders TRUE` still registers the root collider unconditionally; `OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER=0` restores the old skip (value-parsed). Rover Robotics' MEGA and MAX 150 and the TurtleBot3 Burger are this shape. Pins: `tests/test_newton_robot_root_collider.py` (the rule, engine-free) and `tests/test_newton_urdf_fixed_links.py` (the solver model).

## Links with `<inertial>` but no `<collision>` do not collide (since 2026-10-06)

The importer gives such a link a 1 cm sphere, `boundingObject DEF URDF_INERTIA_PLACEHOLDER Sphere { radius 0.01 }`, because the engine needs some boundingObject on a body (a body with none falls back to a 12 cm sphere). Until 2026-10-06 that sphere **collided**: every collision-less link carried an invisible 1 cm marble at its origin. URDF semantics, and MuJoCo's own URDF import, give such a link no contact at all. Measured before the fix: LimX HU_D04's arm stopped against a box at −44° that plain MuJoCo's hand passes into (−67.6°); TRON1 stood on the marbles under its feet with 12.03 N·m at the knee (MuJoCo 0.54); the DJI Mavic 2 Pro rested tilted on its propeller and gimbal marbles — the very contact public issue #10 removed from its URDF. After: −67.57°, 0.5425519 N·m, and the Mavic rests on its own body box.

The sphere is still registered (its body keeps its mass and inertia), with collision switched off; a quieted sphere also no longer counts as a descendant collider when the engine decides whether a Robot root uses its own `boundingObject`. The load log says how many were quieted. `OMNISIM_NEWTON_URDF_PLACEHOLDER_COLLIDES=1` (value-parsed) restores the old contact. A link that should touch things needs a `<collision>`. Pin: `tests/test_newton_urdf_inertia_placeholder.py`.

## An empty link on a moving joint carries the links fixed below it (since 2026-10-06)

A link that is the child of a revolute/continuous/prismatic joint and declares neither `<inertial>` nor `<collision>`, with mass or collision welded below it by fixed joints — Kinova Gen3 lite's `end_effector_link`, with the gripper and `tool_frame` underneath — used to import with no Physics. It was a kinematic frame, so its joint had no dynamic endpoint and the fixed subtree registered as a separate FREE body: on a probe arm a 1 kg gripper fell away to z = −11 m; on the Gen3 lite the gripper and a 0.5 kg payload vanished from the joint torques (joint_2 8.675 N·m either way; plain MuJoCo 9.996 / 13.712), and `run-headless` still printed PASS. The importer now gives such a link the 1 g synthetic body it already gives a collision-only link, so the fixed subtree merges into it as MuJoCo's URDF fusing does; the arm then moves like the same arm with the gripper's mass folded in, within the extra gram. `OMNISIM_URDF_EMPTY_LINK_BODY=0` restores the old import. Pin: `tests/test_urdf_empty_link_carries_fixed_subtree.py`.

## A moving joint on an empty fixed frame loads (since 2026-10-06)

`body -fixed-> mount (no <inertial>) -continuous-> wheel` (Husarion Lynx) or `torso -fixed-> arm_center -revolute-> arm` (Dexmate Vega) used to FATAL with `parent body '<frame>' ... never registered a Newton body`. The joint now attaches to the body that carries the frame, with its anchor and axis re-expressed in that body's frame: the motion is bitwise identical to the same robot with the frame folded in. `OMNISIM_NEWTON_JOINT_PARENT_FRAME_WALK=0` restores the FATAL. Still open: a joint whose whole ancestor chain owns no body. Pin: `tests/test_newton_joint_parent_empty_frame.py`.

## A start pose on a joint whose range reaches past ±π is read correctly (since 2026-10-07)

The importer writes a revolute joint's child link already posed at its initial angle — the `<rest>` value, else the midpoint of a range that excludes 0 — and tells the joint it starts there (`HingeJointParameters.position`). From 2026-09-20 (`b2857a300`, which stopped clamping such a range and skipped its stops instead) to 2026-10-07 the `position` line was written only for ranges that fit inside (−π, π), while the link was posed for every range. A range reaching past ±π at one end therefore spawned the link at the rest angle with the joint reading **0** there: every `setPosition()` landed `rest` radians off. Measured before the fix on UFACTORY's xArm6 with `<rest>` on joint3 (−3.927..0.192) and joint5 (−1.693..3.14159): the sensors read 0.0000 / 0.0000 at the seeded −2.1389 / 0.4585 pose, and a controller drove joint3 to a physical −245.6°, past its own −225° stop. It also hit every shipped quadruped whose thigh range passes π with a `<rest>`: Unitree Go2 and B2 and the OmniQuad, 28 joints. Holding its standing pose, the Go2 slumped to a 0.184 m body height with its thighs physically at 1.58 rad; after the fix it stands level at 0.310 m. After the fix the sensor reads the seeded angle on the first step, and sensor and physical angle agree. No hatch: the old behaviour was an inconsistent state. Pin: `tests/test_urdf_rest_wide_range.py`.

## A `<rest>` on a full-turn or continuous joint is applied (since 2026-10-23)

`<rest>` (an OmniSim extension) is read first, on every revolute and continuous joint. Until 2026-10-23 the importer checked the range first and returned for a joint whose limits cover a full turn (lower ≤ −(π − 0.01) and upper ≥ π − 0.01) — it gets no stops and no seeded midpoint — before it looked at `<rest>`, so ±3.14, ±3.1416 or ±2π joints started at **0** with nothing logged, while ±3.13 started at the rest angle (Ti5 T170 shoulders, 2026-10-19). A `continuous` joint, a revolute with incomplete limits and a range lying wholly past ±π lost their `<rest>` the same way. A `<rest>` outside the joint's own limits is applied and warned about by name. Prismatic joints do not read `<rest>`: they start at 0, or at the nearer stop when the travel excludes 0. Pins: `tests/test_urdf_rest_wide_range.py`, `tests/dev/test_urdf_rest_full_turn.py`.

Two log fixes found on the same run: every importer warning used to appear **twice** (the world pre-scan for undeclared PROTOs expanded each `URDFRobot` as well as the load), and an `effort="0"` joint's "passive" warning used to be followed by a "SYNTHETIC position gain" warning saying OmniSim drives it. An effort-0 joint is passive — its zero effort limit reaches the solver — and the engine now says so at INFO level.

## A prismatic joint's motor reports its travel (since 2026-10-07)

The 2026-09-20 fix that made a revolute joint's motor carry its URDF range (`minPosition`/`maxPosition`) left prismatic joints out: their `LinearMotor` read `getMinPosition() = getMaxPosition() = 0` while the joint's stops enforced the range, so a controller reading the motor to find a finger's travel was told it could not move. Seen on the Niryo Ned2 jaws (−0.01..0.01 m) and the Seeed reBot fingers. The importer now writes the travel into the `LinearMotor` too. The solver limit is unchanged: the engine reads the motor range first and the stops second, and both carry the same values, so the per-joint registration lines are identical before and after. Across the 14 shipped worlds with prismatic grippers, the 11 on the CPU solver are bitwise identical over 15 s; the three cloth worlds run on `mujoco_warp` and differ as much between two runs of the same engine as between engines. What does change: a `setPosition()` past the travel is now clamped to it, with a warning, exactly as for revolute joints. Pin: `tests/test_urdf_prismatic_motor_range.py`.

## Inertia import (ON by default since 2026-09-10)

### `OMNISIM_URDF_USE_INERTIA` — emit `Physics.inertiaMatrix` from URDF `<inertia>` tensors **(default ON)**

What it does: passes `<inertia ixx ixy ixz iyy iyz izz>` straight into `Physics.inertiaMatrix` (2-line MFVector3), together with the `<inertial><origin>` as `Physics.centerOfMass`, for any link with a physically admissible tensor.

⚠️ **This used to be OFF by default, and the consequence was that a URDF's `<inertia>` NEVER reached the solver.** Measured 2026-09-10 on a two-link cart-pole: a pole declaring `iyy = 0.0666667` about its COM was integrated at `0.00334` — the Newton runtime's Husky-tuned mass preset `m*(0.0094, 0.0167, 0.0094)` — and *multiplying the URDF tensor by 4.5× produced a byte-identical `mjModel`*. The tensor was not merely ignored; it was masked by a number with no relation to the robot. The OFF default was an ODE-era workaround for a `dMassSetParameters` crash, and ODE was deleted on 2026-08-08 (`bdc02139`), so the reason had already outlived the code. **`OMNISIM_URDF_USE_INERTIA=0` restores the bounding-object-derived import** (value-parsed) for a bisect.

Two guards remain, and both now WARN instead of dropping a tensor silently:

- **Small-tensor clamp, re-derived at `1e-9`** (it was `1e-4`). The old value was the ODE crash threshold; MuJoCo's own floor is `mjMINVAL` (1e-15), so `1e-4` was a second silent mask — a 100 g gripper finger's real tensor is ~1e-5 and was being thrown away. `1e-9` is a numerical-sanity floor, not a crash guard.
- **Triangle inequality on the principal moments (`a + b >= c`)**, mirroring `principal_moments()` in `scripts/dev/urdf_import.py`. A tensor can be positive definite and still describe no rigid body; MuJoCo's compiler REJECTS such a body, so — now that the tensor actually reaches the solver — an unchecked one would turn a sloppy URDF into a hard load failure. Such links fall back to bounding-object inertia with a named warning.

Where the tensor lands is reported once per load: `[OmNewtonBackend] inertia provenance: N declared, M from geometry, K from the mass preset`. A non-zero `preset` count names bodies whose rotational inertia no line of the world declares and no geometry implies.

Verified end-to-end: `python -m omnisim run-headless projects/samples/demos/worlds/physics/newton_husky_smoke_test.omniworld --duration 15` drives the Husky 12.53 m against 12.51 m with the tensors reverted (0.17%).

## Opt-in features

### `OMNISIM_URDF_USE_SENSORS=1` — emit OmniSim devices from `<gazebo><sensor>` and `<plugin>` blocks **(works for short runs; long-run crash is in the engine, not the importer)**

What it does: maps `<gazebo reference="LINK"><sensor type="imu/gps/camera/ray">` and legacy `<plugin filename="libhector_gazebo_ros_*.so">` patterns into `InertialUnit`/`Gyro`/`Accelerometer`/`GPS`/`Camera`/`Lidar` nodes. Devices are wrapped in a per-cluster carrier `Solid` (the pattern from `projects/samples/devices/worlds/imu.omniworld`) which is emitted inline as a child of the sensor's URDF link. Carrier has a tiny `Box` bounding object + `Physics`; SolidMerger absorbs it into the parent body.

What works:

- Static `urdf_import.py --report` lists all sensors with their kind / link / update_rate.
- The runtime importer emits the carrier Solids cleanly.
- World load + first ~5 seconds of fast-mode simulation pass; controllers start; sensors are registered and queryable.

The delayed-crash isolation finding (this is the key conclusion):

- Around 5-10 seconds into a longer run, OmniSim crashes with `ACCESS_VIOLATION` (0xC0000005) regardless of what URDF you import or how the carrier is emitted.
- **The stock `projects/samples/devices/worlds/imu.omniworld` sample world — Cyberbotics' own hand-authored example with `Accelerometer`/`Gyro`/`Compass`/`InertialUnit` siblings inside a `Solid` — crashes at the same ~5-8 second mark with the same exit code**, both in `--mode=fast` and `--mode=realtime`. Reproduce with `python scripts/dev/headless_runner.py projects/samples/devices/worlds/imu.omniworld --duration 25`.
- Bisects ruled out as causes: URDFRobot expansion, override injection, carrier mass/size, in-link vs root emission, controller startup, floor contact (suspending the robot 5 m in the air doesn't help), specific sensor type (IMU-only or GPS-only both reproduce).


What's safe to do today: use the sensor gate for short integration runs (`--duration 3-5`), to verify the imported devices register with the expected names + transforms before a controller pipeline goes upstream. Avoid long-running simulations until the bug is patched.

### Further isolation (2026-05-18 session)

Even tighter minimal repro — no URDF, no Physics, no controller:

```
Robot { children [ Accelerometer {} ] controller "<none>" }
```

Crashes with the same exit code at `t ≈ 2s` wall in `--mode=fast --no-rendering`. The crash time tracks **step count**, not wall clock — simpler worlds run more steps per wall second and hit the crash faster. The same world in larger scenes (e.g. `accelerometer.wbt` with its full geometry) crashes later in wall time because each step costs more, but the crash step count is comparable.

Additional negatives ruled out this session:

- `OMNISIM_WITH_NEWTON=OFF` rebuild still crashes. **Newton is not the cause** — earlier suspicion that the postPhysicsStep parent-walk for Newton-backed-ancestor detection was dereferencing stale pointers turned out to be wrong; short-circuiting that walk delayed the crash (per-step cost dropped) but did not eliminate it.
- Plain `Robot { children [ Solid {} ] }` runs 10s+ clean. **The bug is specifically in the OmSolidDevice path**, not in nested-Solid handling.
- `PositionSensor` (extends `OmJointDevice`) and `Motor`/`LinearMotor` worlds run clean. The crash is exclusive to subclasses of `OmSolidDevice` (Accelerometer, Camera, GPS, IMU, Gyro, Compass, TouchSensor, Lidar, Display, Connector, Emitter, Receiver, LED, Speaker, Pen, Radar, RangeFinder, VacuumGripper, etc.). *(This list also named `Radio` when it was written; the `Radio` node has since been retired — there is no `Radio.wrl` and no `OmRadio` class, so it is not a device you can author.)*
- Removing `Background {}` doesn't help. `--no-rendering` doesn't help.
- `controller "<none>"` and `controller "<extern>"` both crash with the same signature, so it isn't in the controller-comms dispatch path.

Where to look next:

- Code that runs per-step for `OmSolidDevice` but **not** for plain `OmSolid` or `OmJointDevice`. The most likely suspects are in `OmRobot::dispatchAnswer` / `writeAnswer` (foreach device → `d->writeAnswer(stream)` runs even when controller is `<none>` since the robot is still serializing state), the physics-object registration done at `OmSolid::createOdeObjects` for a device-Solid that has no physics body (⚠ 2026-08-08: that hook still exists and still runs — it kept its legacy name — but the **ODE space/geom registration it used to perform is gone with the backend (`bdc02139`) and is now a no-op stub**, so the live suspect is what replaced it: the solid-merger + bounding-object walk it still drives, and the Newton registration performed at world build / `finalizeWorld()` in [`OmNewtonBackend`](../../src/omnisim/physics/OmNewtonBackend.cpp). Re-run the minimal repro before trusting any of this suspect list — it was assembled against an engine with two backends), and any sensor-update side path triggered by `OmSimulationCluster::step` for the dual-inheritance OmDevice+OmSolid intersection.
- A debug-symbols OmniSim build + Windows debugger (windbg / Visual Studio) is the fastest next move — the crash is reproducible in seconds with the minimal world above, so a single attach-and-step session should pinpoint the faulting frame.

A device-smoke harness for re-validating after a fix is already in place: `python scripts/dev/device_smoke.py` walks every world in `projects/samples/devices/worlds/` headlessly and records PASS/FAIL/TIMEOUT per world to `device_smoke_results.json`. Pre-fix baseline: 8/45 PASS (motor/brake/encoders/supervisor — none of them exercise the OmSolidDevice path).

### How the gates behave

Both gates default off. Setting the env var to anything other than `0`, `false`, or `off` enables the feature. The `urdf_import.py --report` Python tool always reports what the C++ importer *would* emit if the gates were on, so reports remain useful for planning. The `OMNISIM_URDF_DEBUG=1` runtime log indicates which gates are active by emitting the `inertiaMatrix [...]` and sensor device lines (or omitting them).

## Headless launch (no visible OmniSim window)

OmniSim requires an OpenGL context even with `--no-rendering`, so `QT_QPA_PLATFORM=offscreen` triggers a fatal "could not initialize the rendering system" error. The working pattern is to launch the window normally then hide it via Win32 `ShowWindow(SW_HIDE)`:

```powershell
Add-Type @"using System; using System.Runtime.InteropServices; public class WinHide { [DllImport(\"user32.dll\")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow); public const int SW_HIDE = 0; }"@ -PassThru | Out-Null
$proc = Start-Process -FilePath "msys64\mingw64\bin\omnisim-bin.exe" -ArgumentList "<world.omniworld>","--mode=fast","--batch","--minimize" -PassThru
for ($i = 0; $i -lt 10; $i++) {
  Start-Sleep -Milliseconds 500
  $bin = Get-Process -Name omnisim-bin -ErrorAction SilentlyContinue
  if ($bin -and $bin.MainWindowHandle -ne [IntPtr]::Zero) {
    [WinHide]::ShowWindow($bin.MainWindowHandle, [WinHide]::SW_HIDE) | Out-Null
    break
  }
}
```

Use this for any CI / diagnostic loop where the GUI window should not appear.

## Fixed: TurtleBot3 chassis didn't translate

This was the "wheels spin at commanded speed, chassis stays at exactly (0,0,0)" bug. Bisected to: when the URDF root is a pure-frame link with no visuals / collisions / inertial (e.g. `base_footprint` in all three TB3 URDFs), the importer would synthesize a 0.001 kg Physics + 1 mm sphere bounding object on the Robot's root. That synthetic body's tiny ODE inertia tensor (~1e-10) appeared to leave the merged root body unable to receive friction-induced acceleration from the wheels.

**Fix** (in `emitRobot`): when the URDF root link is empty (no visuals, no collisions, no inertial) AND has a single fixed-joint outgoing edge, walk past it and promote the child link to be the Robot's root. ⚠️ The skipped fixed joints' offset used to be DROPPED here, which moved every child joint of the promoted link by it (TB3 wheel axles at z 0.023 in the Robot frame instead of 0.033; the robot frame then rode 10 mm above its `translation`, the ROSbot 42.5 mm, the ROSbot XL 48 mm). Since 2026-10-03 the offset is KEPT: the Robot frame stays the URDF root's (`base_footprint`, on the floor) and the promoted link's visuals, collisions, inertial and child joints are emitted at the offset. `OMNISIM_URDF_REROOT_KEEP_OFFSET=0` restores the drop. The synthetic-Physics mass and bounding-sphere size were also bumped from 0.001 / 0.001 m to 0.1 / 0.01 m for cases where the empty-root rewrite doesn't apply, as belt-and-braces against the small-body ODE issue.

Verified: `python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.omniworld --duration 30` drives all three TB3 variants — burger 11+ m, waffle 6+ m, waffle_pi 4+ m, no crashes.

The diagnostic harness (`tb3_drive_straight` controller — `turtlebot3_fall_test.wbt` was removed when it stopped earning its keep; reconstruct a minimal one if a "robot loaded but won't drive" regression returns) commands both wheels at a fixed velocity and logs commanded vs measured wheel rotation vs chassis position to `C:\tmp\husky_trace\<name>_straight.log`.

## Running the Jackal demo on Windows

Python controllers (including `husky_random` which the Jackal demo uses) require `python.exe` to be reachable on PATH. On most Windows boxes the only `python` on PATH is the Microsoft Store stub at `%LOCALAPPDATA%\Microsoft\WindowsApps\python.exe`, which prints "install from Store" and exits. Symptom: the world loads, the robot stays still, and `omnisim_log.txt` contains `WARNING: husky_random: failed to start:`. To fix, prepend a real Python install to PATH before launching OmniSim:

```powershell
$env:PATH = "C:\Users\<you>\AppData\Local\Programs\Python\Python314;$env:PATH"
msys64\mingw64\bin\omnisim-bin.exe projects\samples\demos\worlds\showcase\jackal_drive.omniworld --mode=realtime
```

Per-controller fix (alternative): edit `projects/default/controllers/husky_random/runtime.ini` to set `COMMAND = "C:/path/to/python.exe"` instead of the bare `python`.
