# Deep Robotics robot packages — provenance and engineering notes

Everything under `projects/robots/deep_robotics/` except `make_urdfs.py`, this
file, the `omnisim.yaml` manifests, `controllers/` and `worlds/` is
redistributed from
**[DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model)**
under its BSD-3-Clause licence, © 2024 DeepRoboticsLab. The licence text is
[`LICENSE.upstream`](LICENSE.upstream). Redistribution here implies no
endorsement by, or affiliation with, Deep Robotics.

| Field | Value |
|---|---|
| Upstream commit | `549eb9ac08ab0674ad388202f0a276a1b140833b` ("Add DR02 Pro fixed-joint URDF for AMP training", 2026-09-08) |
| Imported | 2026-09-08 |
| Taken | the `urdf/` tree of every robot (URDF + binary STL). The MJCF and USD trees, the high-resolution Google Drive meshes and the README renders were NOT taken |
| Generator | [`make_urdfs.py`](make_urdfs.py) — deterministic; re-run it against a fresh sparse clone to refresh |

## What is in the tree

| Package | Robot | Upstream file | DoF | Mass (URDF) | Notes |
|---|---|---|---|---|---|
| `lite3/` | Lite3 quadruped | `Lite3/urdf/Lite3.urdf` | 12 | 11.9 kg | joints `FL_HipX_joint` / `FL_HipY_joint` / `FL_Knee_joint` (FL, FR, HL, HR) |
| `x30/` | X30 quadruped | `X30/urdf/X30.urdf` | 12 | 55.8 kg | same joint names as the Lite3; four fixed lidar mount links |
| `m20/` | M20 wheeled-legged quadruped | `M20/urdf/M20.urdf` | 12 + 4 wheels | 34.5 kg | joints `fl_hipx_joint` / `fl_hipy_joint` / `fl_knee_joint` / `fl_wheel_joint` (continuous) |
| `m20s/` | M20S (higher-torque M20) | `M20S/urdf/M20S.urdf` | 12 + 4 wheels | 34.5 kg | identical geometry and meshes to the M20; different effort / velocity limits. **Points at `m20/meshes/`** |
| `m20_piper/` | M20 + AgileX Piper arm | `M20_Piper/urdf/M20_Piper.urdf` | 12 + 4 + 6 + 2 | 39.2 kg | arm `joint1`..`joint6`, gripper `joint7` / `joint8` (prismatic). Leg meshes **point at `m20/meshes/`**; only the arm meshes live here |
| `dr02/` | DR02 Standard humanoid | `DR02/urdf/standard/DR02-std.urdf` → `urdf/dr02_std.urdf` | 21 | 67.6 kg | meshes in `meshes/std/` |
| `dr02/` | DR02 Pro humanoid | `DR02/urdf/pro/DR02-pro.urdf` → `urdf/dr02_pro.urdf` | 33 | 78.1 kg | meshes in `meshes/pro/`; adds 3-DoF waist, 2-DoF neck, 3-DoF wrists |
| `dr02/` | DR02 Pro, wrists / neck / waist-xy fixed | `DR02/urdf/pro/DR02-pro_fix_joints.urdf` → `urdf/dr02_pro_fixed_wrists.urdf` | 21 | 78.1 kg | upstream's reduced model "for AMP training"; shares `meshes/pro/` |

The M20-family leg meshes are byte-identical across the three upstream
packages (`make_urdfs.py` checks every md5 before pointing at the shared copy),
which is why `m20s/` and `m20_piper/` carry no leg meshes of their own.

`worlds/` holds one stand world per robot plus `deep_robotics_showroom.omniworld`
(all seven side by side); `controllers/deep_robotics_stand/` is the settle
controller every one of those worlds runs. The chat demos live with the other
chat worlds under `projects/samples/demos/worlds/chat/omnilink_<robot>.omniworld`
and are driven by `omnilink_quadruped_bridge` through its
`_quadruped_configs.py` entries.

## What was changed, and why

Every change is applied by `make_urdfs.py`; the upstream files are not edited
by hand. The joints, limits, masses and collision shapes are upstream's except
where listed here.

1. **Mesh references** `./meshes/<f>` → `package://<pkg>/meshes/<f>` so the
   OmniSim importer resolves them by walking up from the URDF directory (the
   Go2 layout).
2. **Standing rest pose** (quadrupeds only). An OmniSim `<rest>` child on each
   leg joint. Without it the Lite3 and X30 would be posed with the knee OUTSIDE
   its own range (lower limit 0.524 / 0.349 rad at q=0) and the M20 family
   straight-legged on its wheels. The hip-pitch and knee axes are `(0, -1, 0)`
   in every Deep Robotics URDF, so a negative hip-pitch swings the thigh
   backward and a positive knee angle flexes the shank forward:

   | Robot | hip-roll | hip-pitch | knee | Leg (thigh / shank) | Hip height above foot centre |
   |---|---|---|---|---|---|
   | Lite3 | 0 | −1.0 rad | 1.8 rad | 0.200 / 0.210 m | 0.254 m (foot 1.8 cm behind the hip) |
   | X30 | 0 | −0.9 rad | 1.8 rad | 0.300 / 0.310 m | 0.379 m (foot 0.8 cm ahead) |
   | M20 / M20S / M20 Piper, front legs | 0 | −0.8 rad | 1.6 rad | 0.250 / 0.250 m | 0.348 m to the wheel axle (wheel r 0.09) |
   | M20 / M20S / M20 Piper, rear legs | 0 | +0.8 rad | −1.6 rad | 0.250 / 0.250 m | same, folded the other way |

   The M20 rear legs fold toward the body centre because upstream's rear joint
   ranges are the mirror image of the front ones (`fl_hipy` [−2.583, 2.286] vs
   `hl_hipy` [−2.286, 2.583], likewise the knees). The DR02 humanoids stand
   straight at q=0 and need no rest pose. Measured stance heights under the
   world recipe below: Lite3 0.262 m, X30 0.395 m, M20 0.428 m, DR02 0.901 m
   (1–2 cm below the geometric figures: contact and servo compliance).
3. **Foot inertials removed, mass folded into the shank** (Lite3, X30).
   Upstream declares the foot links with a real mass (0.02 / 0.06 kg) and an
   all-zero (Lite3) or 1e-12 (X30) inertia tensor. Kept, the importer makes
   each foot a 0.02 kg body welded to the shank whose sphere sits exactly on
   the sphere injected in step 4; that pair is not self-collision-filtered
   (only revolute-connected links are) and the permanent overlap shoves the
   leg — the Lite3 skated backward off its feet and flipped. Without the
   inertial the foot is an inert frame, as on the Go2, and the shank's mass
   goes up by the foot's so the total is unchanged.
4. **Foot contact on the shank** (Lite3, X30). The foot sphere is injected onto
   the shank link at the foot offset (z −0.21012 / −0.31 m), FIRST in the
   link's collision list: an inert foot frame carries no collider, the shank's
   own cylinders end short of the foot tip, and only the first collider of a
   link is registered unless `WorldInfo.newtonCompoundColliders` is TRUE. Same
   fix as the Go2 package.
5. **M20 Piper typo.** `fl_hipx_joint` has `upper="0。611"` upstream — a
   full-width ideographic period (U+3002) — which no URDF parser reads. Every
   other M20 URDF has `0.611`; corrected.

## Not changed

- Joint effort / velocity limits, masses (bar the foot fold above), inertias,
  collision primitives and mesh geometry.
- The `<mujoco>` compiler block each upstream URDF carries (ignored by the
  importer).
- Robot names (`Lite3`, `X30`, `M20`, `M20S`, `M20_Piper`, `DR02-B2-S`,
  `DR02-pro`).

## The terrain crawl (`worlds/lite3_terrain`, `worlds/x30_terrain`)

`controllers/deep_robotics_crawl` is the Unitree B2 crawl model of
`projects/policies/control/gait/b2_crawl_gait.py` (stance foot slides back at
−vx, quintic swing arc, creep order FL → RR → FR → RL at duty 0.85, stride
ramped in from a standing start) re-parameterised for the Deep Robotics leg
geometry, with every IK angle negated because these URDFs put the hip-roll
axis on (−1,0,0) and the pitch axes on (0,−1,0) — the mirror of Unitree's.
`python deep_robotics_crawl.py --selftest` runs the IK output through a
forward kinematics built from the URDF chain (worst foot error 1e-16 m). A
heading hold steers with the model's yaw sweep, using the robot's own pose as
an IMU and odometry would; it changes foot targets only. Nothing writes the
body pose. On a world with an ElevationGrid the gait is terrain-AWARE (next
section); it is a statically stable scripted gait either way, not learned
locomotion.

Measured 2026-09-08 on the 10 m strip (hills 2 → 6 cm, 0.8 cm ripple): the
Lite3 crossed it at 0.045 m/s (14.3 m in 320 s), centreline within 12 cm (20 cm under the map-aware gait),
minimum up-vector z 0.97; the X30 at 0.07–0.10 m/s, within 4 cm, minimum
0.99, and stops at x = 11.5 m holding its stance. Without the heading hold the
Lite3 veered off the strip sideways after 1.3 m. The commanded speed is
0.10 / 0.14 m/s; the shortfall is stance-foot slip.

## The extreme course (`worlds/x30_extreme_terrain`)

A 16 m × 4 m ElevationGrid (0.2 m cells): x 0–4 m rolling hills growing
from 6 to 14 cm, 4–8 m a rubble field of 0.4 m plateaus at 0 / 2 / 4 cm on
top of 6 cm hills, 8–11 m a 12° ramp, 11–12 m a 0.64 m plateau, 12–15 m a
12° ramp down, a 1 cm ripple everywhere. The X30 starts 1.5 m before it and
stops at x = 16.5 m.

`TerrainCrawl` in the controller makes the gait terrain-aware. The grid is
read ONCE through the Supervisor (`getFromDef TERRAIN_GEOM`, its parent
`TERRAIN` for the offset) and stands in for the elevation map a real robot
builds from depth sensing; nothing senses contact. Per step: stance feet
are placed on the map height; the body rides `body_height` above the MEAN
of the four feet's ground heights (referencing the map under the body
centre jumped at every plateau edge); a sustained grade, read 0.8 m ahead
and behind along the heading and gated at 6 %, pitches the body to 0.7 of
the slope through a 0.01 low-pass; each swing picks the flattest of three
footholds along the stride (0, ±5 cm; scored by map roughness plus
0.15·|offset|) and lifts over the highest ground between lift-off and
touchdown plus `--step-height`; descents slow the gait clock (up to 65 %).
A measured-tilt feedback term exists and ships at gain 0: correctly signed
gains of 0.3–0.6 rocked the robot over in the rubble, so the body follows
the map, not the IMU.

Measured 2026-09-08 (X30, `--step-height 0.12`): crossed the whole course
including the descent — 18.07 m in 320 s (0.056 m/s), minimum up-vector z
0.91, centreline within 25 cm, z 1.02 on the plateau, stops at x = 16.57
holding its stance. Negative results, so nobody re-runs them: rubble
plateaus of 0 / 4 / 8 cm and 0 / 3 / 6 cm tip the X30 at x ≈ 4–5 m (the
0 / 4 / 8 field was crossed once by an earlier configuration that then
failed the descent); independent 0.2 m cells up to 12 cm make a bed of
pillars with 30–40° facets that it slides off; a lateral (±8 cm y)
foothold search narrowed the support polygon and made the rubble worse
than a stride-only search; and a half-scale copy of the course (3–7 cm
hills, 1–2 cm rubble, 6° ramps) throws the Lite3 sideways in one second at
the crest of its 7 cm hill (x ≈ 3.8 m, a contact ejection, not a stumble),
so no Lite3 extreme world ships.

## How the worlds make them stand (measured 2026-09-08, Lite3, 8 ms step)

Two engine behaviours shaped the worlds and the settle controller. Both were
established with a supervisor trace that logs, every step from the first, the
body pose, every joint's measured angle and target, and the engine's contact
points (`OMNISIM_PROBE_OUT` on `deep_robotics_stand`).

**The joints used to start the first physics step at q = 0, not at the rest
pose — fixed in the engine the same day.** At t = 8 ms every Lite3 leg read
hip-pitch −0.015 and knee 0.524 (its lower stop) while every motor's TARGET
already read −1.0 / 1.8: the `<rest>` value reached the motors, not the
initial joint state. With no controller the servos yanked twelve joints from
straight to crouched in ~0.2 s while the body was still at spawn height; the
feet swung down at ~2 m/s, punched through a 4 cm floor box, and the robot
landed on its thighs or its back. Three things disagreed about what
`HingeJointParameters.position` means: the URDF importer wrote the endPoint
at the URDF zero pose and set `position` to the rest angle; Webots' joint
bookkeeping treats the authored endPoint pose as the pose AT `position`; and
the Newton bridge registered the joint frame at the current pose with the
coordinate at 0. The fix (`OmUrdfImporter.cpp`, `OmBasicJoint.cpp`,
`omnisim_newton_runtime.py`) makes all three agree with Webots: the importer
writes the endPoint already posed at the rest angle, the bridge defines the
joint frame at the child's zero pose and seeds newton's joint coordinate with
`position`, and the first forward kinematics lands on the authored pose. The
same trace now reads hip-pitch −1.000 / knee 1.800 at t = 8 ms, the robot
spawns in its crouch and lands on four feet. `OMNISIM_NEWTON_SPAWN_AT_POSITION=0`
restores the old registration (verified: the first-step angles come back).
Every world therefore spawns at the STANCE height — Lite3 0.29 m, X30
0.43 m, M20 family 0.45 m, DR02 0.92 m — and `deep_robotics_stand` is a hold
(its spawn→rest ramp only does work with the hatch off). The Go2 / B2 /
OmniQuad packages use the same `<rest>` mechanism and now spawn posed too.

**A stiff position servo on a light leg link is numerically unstable at 8 ms.**
The engine's default servo gain is 10 × the joint's effort limit (ke 240 /
360 N·m/rad on the Lite3) and, with the URDF inertia tensors ignored by
default, a shank's inertia comes from its registered collider — a 2 cm foot
sphere, 3e-5 kg·m² against the URDF's 9e-4. Once settled, the Lite3 hopped:
body height oscillated 0.24–0.45 m with all four feet leaving the floor, the
pitch rocking grew, and it flipped onto its back within 4–7 s. It did so on a
box floor and on a Plane, at friction 1 and 2, at contact stiffness 2500 /
8000 / 20000 N/m, at 16 ms, with servo damping zeroed, and with 0.02 kg·m²
of joint armature. Honouring the URDF inertias (`OMNISIM_URDF_USE_INERTIA=1`)
made it milder; nothing stopped it. What holds is two WorldInfo fields
together: `newtonSubsteps 8` (1 ms solver steps) and
`newtonCompoundColliders TRUE` (every collider registered AND the body's
inertia derived from all of them), with `newtonGroundMu 2` so the sphere feet
plant instead of skating. Under that recipe every robot settles level and
stays put (x drift ≤ 1 cm over 9 s; the M20's four wheels and the DR02's two
box feet included). `newtonSubsteps 4` still flipped; `newtonSubsteps 8`
without compound colliders left the Lite3 tilted on a thigh. Every world
under `worlds/` and every `omnilink_<robot>.omniworld` chat world carries
the recipe.

The OmniQuad chat demo pins its floating base with the supervisor for the
same reason ("the stiff-legged stance topples under Newton"); the Deep
Robotics robots do not need the pin under the recipe, and their bridge
configs set `body_lock: False`.

A negative result worth keeping (2026-09-08): the textbook explicit-spring
bound does NOT predict this. A per-world report of kp·dt²/(4·I) with I the
mass-matrix diagonal at the joint read 1.6× for the flipping Lite3 at 8 ms,
0.025× under the recipe, and 2.9× (OmniArm 6), 5.2× (UR5e) and 7.6× (Go2 at
16 ms) for worlds that hold their pose — so a warning built on that ratio was
written, measured and removed. What separates the recipe from the failure is
not captured by that number; treat the recipe as empirical until someone
finds the mechanism.
