# Newton → complete ODE replacement — the coverage plan

> **[STATUS UPDATE 2026-07-09 — the user-visible outcome of this plan has SHIPPED.]**
> Newton is now the **engine default**: `physicsBackend "auto"` resolves to Newton
> wherever its Python runtime is present, and a stock `make release` bundles the
> Newton runtime by default (`BUNDLE_NEWTON ?= 1`) — see AGENTS.md "Engine defaults
> (v4)". ODE remains the permanent one-switch fallback (`OMNISIM_LEGACY=1` /
> `OMNISIM_FORCE_ODE=1`) and the verification oracle, exactly as the non-negotiables
> below require. This document remains as the working record of the coverage arm;
> its in-body progress/coverage numbers are kept as written (a 2026-06-23 note in
> default-flip-plan.md reaffirms the fidelity-coverage numbers).

**Status:** living plan · drafted 2026-06-07 · last worked 2026-06-08
**Progress (~60% of the arm, honest):** instrument COMPLETE (W0 meter + W5.4 robot/prop faithful split) ·
shapes ~90% (plane/mesh/cylinder done, heightfield deferred) · joints 25% (hinge2 done) · **control W3 ~50%
— body-space force injection (W3.1: propeller + supervisor, verified) and velocity sets (W3.2, verified) both
LANDED**; joint-space force go/no-go likely GO (`Control.joint_f` exists) · **W4 native contacts ~80% — both
go/no-gos GO; readback (W4.1) + C++ accessor (W4.2a) + native contacts feeding the damage-tracker supervisor
API, native REPLACES the ODE source, VERIFIED on a single box (0→4) AND a multi-body rover (descendants +
per-solid + map, double-count bug found & fixed)**. **Three go/no-gos resolved this session — force injection,
native contacts, joint force — all GO: no upstream wall remains, only execution.** Honest robot-fidelity:
**5/8 corpus robots FAITHFUL** (misses attributed: prop chaos, a URDF parse bug, an uncontrolled drone — not
contact bugs). NOT started: W4.3 flip (native default for Newton + drop the ODE keepalive — gated on the real
battlebot damage-tracker check), W3 joint-space force, W2 ball/fixed joints, W6 legged, W7 flip. ODE stays.
**Goal:** drive Newton from today's ~35–40% world coverage to ~100% — every shipped world runs *faithfully*
on Newton, and Newton becomes the resolved default for all of them. This builds **on top of** the
[architectural baseline](architectural-baseline.md) (the dispatch structure is already done) and the
[engine-migration plan](engine-migration-plan.md) §13 (the P0–P8 physics arm); it is the "completeness"
layer those two explicitly scoped out.

> **ODE is not deleted.** Migration non-negotiables #1/#2 keep ODE compiled, reachable, and byte-identical
> forever — as the one-switch fallback (`OMNISIM_LEGACY=1`) **and** the verification oracle this plan
> measures against. "Complete replacement" therefore means: **the capability gate's ODE-fallback rate
> → ~0 on the world corpus, with ODE retained as the permanent reference** — not "remove ODE."

**The success metric (one number):** a standing *coverage meter* — the % of a fixed world corpus that
(a) resolves to Newton through the capability gate **and** (b) matches ODE within the physics-oracle
tolerance. Today ≈ 35–40%. Target ≥ 99%, the residual being worlds where ODE is *deliberately* kept
(documented, not accidental).

---

## 1. The honest shape of the problem

Newton is a GPU **reduced-coordinate (Featherstone)** solver; ODE is a CPU **maximal-coordinate** solver.
The gap to 100% is not one thing — it is five classes with very different difficulty, and conflating them
is how this work gets mis-estimated:

| Class | What it is | Difficulty | Concrete items |
|---|---|---|---|
| **A. Coverage overrides** | shapes + joints the gate rejects | bounded engineering | plane, mesh, heightfield, native cylinder; ball / hinge2 / explicit-fixed joints |
| **B. Model-mismatch ops** | per-step writes ODE has that Newton's model resists | design + maybe upstream | force/torque injection, mid-step velocity sets, joint-param sets, per-body damping |
| **C. Observability** | damage + ray sensors currently ride ODE's collision pass | architectural | native Newton contact readback → damage + sensors |
| **D. Dynamic fidelity** | contact stability, legged deploy, determinism | partly research | G1 falls @1.55 s, high-closing-speed contact divergence |
| **E. Upstream** | Newton/Warp is NVIDIA's | track / request / contribute | body-index-N cliff, contact readback, exotic joints |

The instinct "it's just shapes + joints" is **wrong**: class A is the easy part. The real spine of the
effort is **B** (per-step control injection, which Featherstone reduced coordinates make awkward) and
**C** (cutting the ODE collision umbilical). Sequence accordingly: bank the cheap A wins to move the meter
and keep momentum, but budget the bulk of the calendar for B + C, and treat D as a measured research lane.

---

## 2. The instrument — capability gate + coverage meter (build FIRST)

The capability gate is both the **control point** and the **measurement point**, so it anchors everything.

**Where it lives:** [WbSolid.cpp](../../src/omnisim/nodes/WbSolid.cpp) `articulationNewtonCapable()` /
`effectivePhysicsBackendName()` (~lines 2390–2560). It is **per-articulation**, resolved at the
articulation root, two-tier:
- **Tier A (correctness):** any joint whose `nodeType()` is not `WB_NODE_HINGE_JOINT` /
  `WB_NODE_SLIDER_JOINT` → ODE. (Deliberately `nodeType()`, not `dynamic_cast`, because `WbBallJoint` /
  `WbHinge2Joint` derive from `WbHingeJoint`.)
- **Tier B (fidelity):** any mesh collision geometry in the bounding object → ODE.
- **Scope:** ONE unsupported joint or mesh *anywhere* in the articulation forces the *whole* articulation
  to ODE. So closing a gap can unlock a disproportionate number of worlds (a single mesh prop today drags
  an otherwise-Newton robot onto ODE).

Every workstream below is the same shape: **close a gate branch → prove parity on the oracle → watch the
meter climb.** So step zero is the meter:

- **W0.1 — World corpus.** A fixed, tagged corpus: the shipped `projects/**/worlds/*.wbt` plus targeted
  fixtures, each tagged by the feature it exercises (mesh, plane, ball-joint, force-controller, ray-sensor,
  legged, …). This is the denominator of "100%."
- **W0.2 — Coverage meter.** `scripts/dev/newton_coverage.py`: for each corpus world, report the gate
  verdict (Newton / ODE **and the reason**) and, when Newton-eligible, the
  [physics_oracle](../../scripts/dev/physics_oracle.py) divergence vs ODE. Output: a coverage %, a per-world
  table, and a histogram of *why* worlds fall back (mesh vs joint vs op). **This dashboard runs after every
  workstream** and is the single source of truth for "where are we."

*Acceptance for the phase:* met — `OMNISIM_PROBE_GATE` (a post-finalize sweep in `WbWorld::finalize()`
recording every top-level articulation's resolved backend + gate verdict) + `scripts/dev/newton_coverage.py`
print the baseline below.

> **Climb so far (8 robot worlds):** baseline **5/8 worlds · 124/129 articulations · 4 mesh + 2 joint
> gaps** → after **W2 (Hinge2)** + **W1 (mesh)** → **8/8 worlds (100%) · 135/135 articulations (100%) · 0
> gaps**. Every gate gap in the robot corpus is closed. (Eligibility, not yet faithful-vs-ODE — see W5.4.)

### First reading — baseline (8 robot worlds, 2026-06-07)

| metric | value |
|---|---|
| worlds entirely Newton-eligible | **5 / 8** (spot, mavic, rosbot, + the two cobot-arm worlds) |
| mixed (some articulation ODE-gated) | 3 (panda, rosbot_xl, ur_arms) |
| top-level articulations Newton-**eligible** | **124 / 129 (~96%)** |
| capability-gate gaps | **mesh ×3 worlds** (spot, rosbot_xl, ur_arms) · **joint ×1** (panda's platform carts) |

**Two honest readings of this number:**
1. **The gap histogram is the actionable signal, and it confirms the plan's ordering:** MESH is the
   dominant blocker (3 of the 4 gap-worlds) → **W1.3 (mesh narrow-phase) is the single highest-leverage
   first target**; the lone joint gap (panda's carts, a non-Hinge/Slider joint) is W2.
2. **"Eligible" ≠ "faithful" — ~96% MUST NOT be read as "Newton already runs 96% of robots well."** The
   gate is permissive: it only blocks mesh + non-Hinge/Slider joints, so a hinge/slider robot is "eligible"
   regardless of whether Newton *simulates it faithfully*. Worse, `spot` reads as "3/3 eligible" only
   because it is **force-pinned to Newton despite mesh colliders** (gate flags `mesh` → AABB-approximated,
   degraded fidelity). Real coverage — eligible **and** matching ODE within the oracle tolerance — is the
   ~35–40% figure and needs the W5.4 faithful-match layer on top of this eligibility meter (a follow-up;
   this v1 measures eligibility + the gate gaps, the signal W1/W2 move).

---

## 3. Workstreams (close the gaps)

### W1 — Collision shapes (Tier-B + the cylinder fidelity bug)
Newton builds sphere / box / capsule today; **cylinder is faked as a sphere** (a real fidelity bug for
wheels, [WbSolid.cpp](../../src/omnisim/nodes/WbSolid.cpp) ~2788–2791), and **plane / mesh / heightfield are
absent**.
- **W1.1 Plane** (static half-space ground). ✅ **LANDED 2026-06-07** — the faithful-check (W5.4) finding:
  panda's props fell through its floor because the floor's collision is a **Plane** and only `add_shape_mesh`
  was wired. Wired `addShapePlane` → native `add_shape_plane` (local normal +Z, the `WbPlane` convention;
  the body's transform orients it); node-side detects a `WbPlane` boundingObject. **Verified:** panda's
  early divergence **0.887m → 0.334m** (statics) → **0.269m** (statics + substeps=4), and crucially
  `full == early` now — props are CAUGHT and settle (no longer falling through); ur_arms **0.498 → 0.062m**
  early. Takes effect when the floor is a Newton static body, so `faithful_check.py` now runs the Newton
  side with `OMNISIM_NEWTON_STATICS=1` + `SUBSTEPS=4` (the intended static-world config). *Residual: panda's
  ~0.27m steady settling OFFSET is contact-stiffness fidelity (XPBD softer than ODE) — W5, not a plane bug.*
- **W1.2 Native cylinder narrow-phase. ✅ LANDED 2026-06-08 (capsule fit; metric-neutral, honest).** Native
  Newton cylinder narrow-phase locks wheels against the ground (probe 7), so the cylinder boundingObject is
  now handed to Newton as a **same-radius/half-height CAPSULE** instead of the old point-contact sphere:
  `add_shape_cylinder` builds an `add_shape_capsule` rotated −90° about X to map Newton's local-Z capsule axis
  onto the Webots cylinder's body-local-Y axis (verified in isolation: Z→(0,1,0)); WbSolid's cylinder branch
  calls `addShapeCylinder(idx, r, hh, offset)`; revert lever `OMNISIM_NEWTON_CYLINDER_AS_SPHERE=1` (matches
  `OMNISIM_NEWTON_MESH_TO_ODE`). **Honest finding from the A/B (battlebox_husky_proving, the Newton-pinned
  cylinder-wheel fixture): sphere 0.036 m vs capsule 0.037 m early divergence — within noise.** The capsule is
  the geometrically-correct shape (line contact, correct width) and avoids the cylinder-lock by construction,
  but it does **not** move the faithful metric, because that early-window root metric is quasi-static and both
  shapes preserve the same rolling radius — the sphere was never the thing hurting this number. So this RETIRES
  the "cylinder faked as a sphere" stub with the correct geometry and a clean no-regression, *not* a fidelity
  gain; the husky's residual ~37 mm drift (present for both shapes) is W5 contact-stiffness, not a shape bug.
- **W1.3 Mesh / trimesh narrow-phase — THE single biggest unlock. ✅ LANDED 2026-06-07.** GO/NO-GO
  resolved **GO, no upstream block**: Newton has native `newton.Mesh(verts, indices)` + `add_shape_mesh`
  (no convex decomposition needed) — the old "Newton only AABB-approximates" was OmniSim never wiring it.
  Implemented: `WbNewtonBackend::addShapeMesh` (marshals the flat vertex + triangle-index arrays as PyLists)
  → Python `add_shape_mesh` building a `newton.Mesh(compute_inertia=False, is_solid=True)`; the WbSolid
  AABB-box fallback in `attachNewtonShapeFromBoundingObject` REPLACED with real mesh registration (verts +
  indices from `WbTriangleMesh::coordinatesData()`/`indicesData()`); Tier-B mesh gate narrowed (mesh allowed
  by default, `OMNISIM_NEWTON_MESH_TO_ODE=1` is the per-shape revert lever). **Verified:** `add_shape_mesh`
  API in isolation; ur_arms mesh forced-to-Newton stable 18 s; the robot-corpus meter went **5/8 → 8/8
  worlds (100%) eligible, 124/129 → 135/135 articulations, 4 mesh+2 joint gaps → 0** (with W2); spot + panda
  step stably on default Newton mesh. *Honest: this is ELIGIBILITY (the gate now admits mesh + it's stable),
  NOT yet faithful-vs-ODE — mesh-contact oracle parity is W5.4. Mesh-vs-mesh / very-high-poly stress + the
  shape-Pose-rotation offset are follow-ups.*
- **W1.4 Heightfield** (terrain worlds) — `add_shape_heightfield` exists natively too (same pattern as mesh).
  *Confirmed exercised: `UnevenTerrain.proto` and `husky_rocks_traverse.wbt` both `boundingObject USE` an
  `ElevationGrid`.* **Consciously DEFERRED 2026-06-08** in favour of the higher-leverage W5 meter-split: it is
  a single terrain world, a bigger task (height-data extraction + a new native shape + the static-collider
  path), and after W1.2 showed shape-correctness is metric-neutral, the faithful number moves on contact
  fidelity, not on adding one more eligible shape. Pick it up when a terrain world is prioritized.
- *Each:* narrow the Tier-B mesh gate for the now-supported case + oracle-verify within tolerance.

### W2 — Joint types (Tier-A)
Newton builds revolute + prismatic (with velocity/position targets, limits, effort caps) and, since W2,
hinge2/universal + ball (both passive). Still missing: **explicit fixed** (today fixed solids are
ODE-merged), AMotor/LMotor, and *motorised* ball.
- **W2.1 Explicit fixed joint** under Newton (don't lean on ODE merge).
- **W2.2 Ball joint** (3-DoF) — common in legged/soft rigs. ✅ **LANDED 2026-06-09.** Native Newton
  `add_joint_ball` (`JointType.BALL` — a quaternion-based spherical constraint, gimbal-free, NOT a d6 Euler
  triple); `WbNewtonBackend::addJointBall` + the embedded-Python `add_joint_ball` queue → `_add_revolute_to_builder`
  `kind=="ball"` finalize branch; registered in `WbBasicJoint::flushPendingNewtonRegistrations` in a dedicated
  branch BEFORE the Hinge2 *and* Hinge casts (`WbBallJoint` is-a `WbHinge2Joint` is-a `WbHingeJoint`, so either
  cast would capture it and register only 1–2 of its 3 DoF — `nodeType()` discriminates exactly); gate Tier-A
  allow-list widened (`WbSolid::solidSubtreeNewtonCapable`). **PASSIVE**: a *motorised* ball still needs MuJoCo
  (ball target pos/vel is XPBD-unsupported upstream) — a follow-up; the passive 3-DoF constraint holds under
  XPBD + MuJoCo alike. **Verified** (the local binary already carried the diff — no rebuild): (1) **API** in
  isolation — `ModelBuilder.add_joint_ball(parent, child, parent_xform, child_xform, collision_filter_parent=…)`
  + `JointType.BALL` confirmed on the installed newton 1.2.0; (2) **coverage meter** — `oracle_mixedjoint.wbt`
  3/3 + `ball_joint_vs_hinge_joints.wbt` 9/9 Newton-eligible, **0 joint gaps** (panda 77/77, spot 3/3 unchanged
  → no regression from the new branch); (3) **behavioral** — `physics_oracle.py --gate --require-newton` on
  `oracle_mixedjoint.wbt` (a Hinge→Ball chain): INV1/INV2/INV3 PASS (finite/settled/reproducible) and the
  default run diverges from ODE by 0.158 m, proving Newton actually *constructed and ran* the ball articulation
  (not a silent ODE fallback) — and that the new flush-branch ordering left the existing hinge registration
  intact. *Follow-up: motorised ball joint when a world needs it; a behavioral ball-vs-hinge faithfulness A/B.*
- **W2.3 Hinge2 / universal** (2-DoF) — steered+rolling wheels (cars). ✅ **LANDED 2026-06-07** (it was
  the only joint gap in the corpus — panda's platform carts). Native Newton `add_joint_d6` with two free
  angular `JointDofConfig` axes (no phantom body); `WbNewtonBackend::addJointHinge2` + the Python
  `add_joint_hinge2` build branch; registered in `WbBasicJoint::flushPendingNewtonRegistrations` (a
  dedicated branch BEFORE the WbHingeJoint cast, since WbHinge2Joint is-a WbHingeJoint); gate Tier-A
  allow-list widened. **Verified:** the meter's joint-gap count went 2→0 (panda's carts now pass Tier-A),
  and the exact `add_joint_d6`+`JointDofConfig` call is API-verified in isolation. The carts then flipped
  to a **mesh** gap (their wheels' mesh colliders) — the loop revealing the next gap (W1). *Follow-up: a
  hinge2-on-primitives fixture for the behavioral/oracle proof (panda can't give it — co-mesh-blocked);
  motorised hinge2 (driven wheels) when a world needs it.*
- **W2.4 AMotor/LMotor** — map to revolute/prismatic PD, or document as covered.
- *Each:* widen the Tier-A `nodeType` allow-list + oracle-verify. Parallelizable with W1.

### W3 — Per-step control ops (the model-mismatch spine)
Newton overrides ~11 of ~45 `WbPhysicsBackend` virtuals (reads + joint targets + reset) and inherits the
rest as `-1`. The notable gaps are the **write side a controller may need mid-step**:
- **W3.1 Force/torque injection** (`addBodyForce/Torque`) — the headline. **✅ GO + CORE LANDED 2026-06-08.**
  **Go/no-go resolved GO, NOT class-E blocked:** Newton's `state.body_f` IS a writable per-body external
  wrench (`spatial_vectorf` `[Fx,Fy,Fz, Tx,Ty,Tz]`, **world frame**), zeroed by `clear_forces()` each (sub)step
  and integrated by both XPBD and MuJoCo — so the reduced-coordinate model does *not* resist external forces
  (verified in isolation: a 1 kg body hovers when injected +mg; slot sweep confirmed force=slots 0-2,
  torque=3-5; a rotated body proved world frame). Implemented: `kNewtonRuntimeSource.add_body_force` (per-tick
  accumulator) → `step()` writes the sum into `state.body_f` after each `clear_forces()` and clears it after
  the tick (ODE addBodyForce semantics); C++ `WbNewtonBackend::addBodyForce`; `WbSolid::applyExternalForceNewton`
  (force-at-world-pos + torque → COM wrench via the body's pose-readback origin); `WbPropeller` routes thrust
  to Newton when the body is Newton-backed (its ODE body is disabled, so the old ODE path was a silent no-op).
  Revert lever `OMNISIM_NEWTON_NO_EXT_FORCE=1`; debug `OMNISIM_DEBUG_FORCE=<file>`. **Verified** three ways:
  (1) isolation (hover/slot/frame, above); (2) the live-binary routing diagnostic — `applyExternalForceNewton`
  fires for `propeller.wbt`'s helicopters with `newtonIdx` 0/1/2; (3) the **decisive** supervisor fixture
  (W3.1 supervisor entry below): a +20 N push lifts a 1 kg Newton box to the analytic height, zero lift
  without injection. **HONEST CORRECTION to commit c9a28347:** that commit cited `propeller.wbt` "force OFF →
  drift 0.102 m, ON → FAITHFUL 0.000 m" — the 0.000 was a single ANOMALOUS run. Re-measured 6× (3 ON / 3 OFF):
  **ON == OFF == 0.102 m drift every time** — that demo's propellers thrust only ~2 mN (far too small to move
  the faithful number, which is a husky-like contact/settling offset), so `propeller.wbt` is NOT a valid
  force-injection A/B. The routing still fires (diagnostic), but the supervisor fixture is the real proof.
  *Note: the corpus `mavic_2_pro.wbt` is `controller "<none>"` (URDF, no Propeller nodes) so its freefall is
  genuinely uncontrolled — NOT a force-injection case, correcting the earlier guess. The SUPERVISOR force API (`wb_supervisor_node_add_force` + `_with_offset` + `add_torque`) now routes
  through the same helper (was warn-and-skip for Newton) — **verified 2026-06-08** with a supervisor-pushes-a-
  Newton-box fixture: +20 N up on a 1 kg box lifts it 0.50→5.44 m in 1.0 s WITH injection (a near-exact match
  to the analytic ½·a·t², so the magnitude is delivered exactly) and **not at all** without it
  (`OMNISIM_NEWTON_NO_EXT_FORCE=1`). Remaining W3.1: `addJointHingeTorque` / `addJointSliderForce` for
  force-controlled joints (joint-space, vs the body-space wrench done here). **Joint-force go/no-go (2026-06-08,
  likely GO):** `newton.Control.joint_f` exists (shape = `joint_dof_count`) — a generalized joint-force input;
  MuJoCo showed motion when written. A clean 1-DoF drive test was muddied by Newton's floating-base DoF layout
  (a single-revolute model reports 7 DoF), so the implementation needs the backend's existing DoF mapping
  (`joint_qd_start` / `slot_to_real_idx`) to address the right DoF + a proper fixture to verify; a guaranteed
  fallback is a body-torque about the joint axis via the already-verified `body_f` path. **RESOLVED 2026-06-08
  — `control.joint_f` does NOT work under XPBD (the default solver); REVERTED.** Wired `set_joint_force`
  (`control.joint_f` at the joint's `qd_start` DoF, zero-and-rewrite each tick) + `WbNewtonBackend::setJointForce`
  + a `motor->userControl()` branch in `WbBasicJoint`'s Newton motor loop, then VERIFIED in-binary on a
  hinge-arm torque fixture (`wb_motor_set_torque(0.5)`, gravity off): **ODE drives the joint to 4.18 rad;
  Newton+joint_f reaches only −0.14 rad** (≈ the no-code baseline of 0.12) — i.e. `joint_f` is essentially
  inert for revolutes under XPBD (consistent with the isolation probe: MuJoCo moved, XPBD didn't). So the
  go/no-go is **NO via `joint_f`/XPBD**, not the earlier "likely GO". (The first "unverified" attempt's
  fixture "load failure" was a false alarm — a 35 s deadline too short for Newton init; the world loads fine
  with ≥60 s.) **Path forward = the body-torque fallback** (apply `T·axis_world` to the child + reaction to the
  parent via the verified `body_f`; world axis = `upperPose()->matrix().sub3x3MatrixDot(axis())`, mind the
  reverse-joint flip). NOTE a fidelity subtlety to check: ODE's 4.18 rad vs a naive `T/I_hinge` estimate (~1.2
  rad) implies a motor-model/inertia nuance to match. Low priority -- torque-control motors are uncommon and
  none exist in the corpus.*
- **W3.2 Mid-step velocity sets** (`setBodyLinearVel/AngularVel`). **✅ LANDED + VERIFIED 2026-06-08.** Newton
  `state.body_qd` is `[vx,vy,vz, wx,wy,wz]` (world frame; verified: slots 0-2 linear, 3-5 angular) and writable
  — `set_body_vel` does a read-modify-write of the linear or angular half (persistent state, unlike the per-
  tick force accumulator). `WbNewtonBackend::setBodyVel` + `WbSolid::setNewtonBodyVel`; `WbSolid::setLinear/
  AngularVelocity` route to Newton when the Solid is Newton-backed (else the ODE set is lost on the disabled
  proxy body). Reached by `wb_supervisor_node_set_velocity`. **Verified:** a supervisor forcing a Newton box
  to (2,0,0) m/s each step moved it +X 1.984 m in 1.008 s (= the commanded 2 m/s); without routing it would
  fall straight (X≈0).
- **W3.3 Joint-param sets** (FMax / lo-hi stops / CFM / ERP) — map to Newton limits/gains; some (CFM/ERP)
  have **no Featherstone equivalent** → document the deliberate divergence.
- **W3.4 Body damping, mass read, auto-disable, joint/body enable-disable** — the long tail of small ops.
- *Honest call:* some of these will not map 1:1. The deliverable per op is either an override **or** a
  documented "Newton models this differently; here's the closest faithful behavior, here's the gate."

### W4 — Native Newton contacts (retire the ODE observability bridge)
Today a Newton-backed Solid keeps a **disabled ODE body + geom** purely so ODE's one collision pass still
feeds the **damage tracker + ray sensors** (see [physics-p8-statics-design.md](physics-p8-statics-design.md)
§2). A truly ODE-free world needs Newton to supply that.
- **W4.1** Newton contact readback → a contact-point/event API on the backend. **GO/NO-GO resolved GO
  2026-06-08 (not class-E):** Newton's `Contacts` object (from `model.contacts()`, populated by
  `model.collide(state, contacts)` — already called every substep) exposes the full pass natively:
  `rigid_contact_count`, `rigid_contact_point0`/`point1` (world points), `rigid_contact_normal`,
  `rigid_contact_shape0`/`shape1` (the shape pair), and even **`rigid_contact_force`** (per-contact vec3).
  Verified in isolation: a box on a plane reports count=4 (the 4 bottom corners), normal (0,0,1), shape pair
  (plane, box). So the readback exists; the remaining work is wiring (a backend accessor that snapshots these
  after collide) + a Newton-shape-index → Solid map (the backend assigns shape indices in `add_shape_*`, so it
  can keep the reverse map) — NOT a missing capability. **Readback LANDED + verified 2026-06-08:**
  `get_contacts()` snapshots the live contacts as a flat `[bodyA, bodyB, point(3), normal(3), |force|]` list
  per contact (shapes→bodies via cached `model.shape_body`; degenerate zero-normal slots filtered); gated dump
  `OMNISIM_DEBUG_CONTACTS=<file>`. Verified in the binary — a box resting on the static floor reports its 4
  bottom corners (normal (0,0,1), box=body 1), once vs the static plane (body −1, Newton's world convention)
  and once vs the floor Solid's Newton body 0 (the floor carries two collider representations under STATICS —
  a W4.2 reverse-map detail). `|force|`=0 under XPBD (positional solve) — W4.2 damage magnitude will come from
  closing velocity. *Remaining W4: the C++ accessor + Newton-body→Solid reverse map + wiring the damage tracker
  and ray/range sensors (W4.2); ray sensors may need Newton raycasting, a separate check from rigid contacts.*
- **W4.2** Wire the damage tracker + ray/range sensors to Newton contacts. **Integration point found
  2026-06-08:** the damage tracker polls `wb_supervisor_node_get_contact_points` → `C_SUPERVISOR_NODE_GET_CONTACT_POINTS`
  → **`WbSolid::extractContactPoints()`**, which iterates `world->odeContacts()`. **CORRECTED 2026-06-08 by
  measurement:** the earlier guess that the bridge "already feeds the damage tracker under Newton" is WRONG for
  *pure-Newton* bodies — a box-on-floor fixture polled `getContactPoints` and got **ODE_pts=0 vs native_pts=8**
  (the bridge's *disabled* ODE proxy body doesn't collide, so `odeContacts()` is empty for it). So native
  contacts are **necessary, not just a cleaner replacement** — and retiring the bridge loses nothing for
  Newton bodies. **Done so far:** (a) ✅ C++ `getContacts(std::vector<WbNewtonContact>&)` accessor + struct +
  the gated `OMNISIM_NEWTON_CONTACTS_CMP` comparison harness in `extractContactPoints` — verified in-binary
  (native = the box's 4 corners × 2 floor reps, correct points/body-pairs). (b) ✅ on-demand Newton-body→Solid
  map (`WbWorld::findSolids()` per poll — no risky persistent state) + (c) ✅ `extractContactPoints` now ADDS
  native contacts for a Newton-backed Solid (opt-in `OMNISIM_NEWTON_NATIVE_CONTACTS`, additive + gated so the
  default and ODE worlds stay byte-identical), mirroring the ODE two-list logic with native data + deduping the
  dual floor registration. **VERIFIED** on a Newton box (supervisor `getContactPoints` 0 → 4) AND on a
  MULTI-BODY robot (chassis + hinge-child rover): the root's `includeDescendants` list covers BOTH bodies (2
  distinct contact nodes [chassis, child]) — descendants, per-solid attribution, and the body→Solid map all
  work on a real articulation. (d) ✅ depth made honest (= 0; `point0/point1` are support points, not
  witnesses — XPBD exposes no clean penetration). **CRITICAL FIX from the multi-body test:** a Newton Robot's
  disabled ODE proxies STILL collide (rover ODE_pts=8≠0), so ADDING native double-counted (8+5=13). Native is
  now the **SOLE source** for a Newton-backed Solid (skip the ODE accumulation when `useNative`) → rover
  NATIVE ON = 5 (no double-count), box unchanged at 4. **Remaining:** the actual battlebot damage tracker
  (bot-on-bot per-solid distinction — mechanism verified via the rover, not yet a 2-Newton-bot fixture); then
  flip native-contacts to the default for Newton worlds + drop the ODE keepalive (W4.3). Ray/range sensors
  stay SEPARATE (raycast, not rigid contact) — own Newton-raycast go/no-go.
- **W4.3** Drop the ODE-body keepalive for pure-Newton worlds (only after W4.2 parity is proven — this is what
  finally takes ODE *out of the per-step loop* for a Newton world; ODE stays compiled + the fallback/oracle).
- Architectural, and gated on W1 (Newton must actually *detect* the collisions first). This is what lets a
  world run with **zero ODE in the loop**.

### W5 — Solver robustness, fidelity, and the parity bar
- **W5.1 Auto solver selection.** XPBD (fast, default) vs MuJoCo (robust friction — *required* for pinch
  grasps; XPBD structurally can't). Pick per-world from features (grasping/load-bearing-friction → MuJoCo)
  instead of hand-setting `WorldInfo.newtonSolver`.
- **W5.2 Contact stability.** Auto-tune substeps (XPBD diverges at high closing speed on 1 substep; some
  worlds need 4). Make the safe default self-selecting.
- **W5.3 Determinism + reproducibility.** Re-establish the claims currently resting on unre-run `_scratch`
  logs (per [rl-current-state.md](rl-current-state.md)); fold into the determinism gate.
- **W5.4 The parity bar / faithful-match layer. ✅ FIRST CUT LANDED 2026-06-07.** The eligibility meter says
  which backend *resolves*; this says whether an eligible world actually *tracks* ODE. `OMNISIM_PROBE_TRAJ`
  (a per-step articulation-root world-position dump in `WbSolid::postPhysicsStep`, both solvers sampled at
  the same point so the 1-step lag cancels) + `scripts/dev/faithful_check.py` run a world Newton vs
  forced-ODE and compare per-root divergence over time (the verdict keys on the EARLY 300 ms window, before
  solver chaos amplifies). **First reading (robot worlds): rosbot 5 mm + the cobot arm 0 mm FAITHFUL; panda 887 mm +
  ur_arms 498 mm DIVERGE.** So **100% ELIGIBLE is ~50% FAITHFUL** — exactly the distinction this layer exists
  to expose, and the *real* coverage number. The divergers point at concrete gaps: **panda's props fall
  through its PLANE floor** (only `add_shape_mesh` is wired, not `add_shape_plane` — W1.1 is the obvious
  next), and ur_arms' mesh arm needs a look. *Follow-ups: tune the tolerance per motion class, fold the
  faithful verdict into newton_coverage.py, drive the corpus faithful as each gap closes.*
- **W5.4 update — the robot/prop split. ✅ LANDED 2026-06-08.** The "first reading" above was *misleading*:
  the meter reported the single WORST root, so one chaotically-rolling free **prop** dragged a whole world to
  "DIVERGE" even though the **robot** tracked ODE exactly. The per-root breakdown of panda made this
  undeniable — *every* divergent root was a loose workshop object (paint bucket 0.270 m, nuts/bolts/wrench/
  hammer ~0.11 m) while the panda arm, tables, carts, walls were all **0.000 m**. A free prop settling/rolling
  to a different-but-equally-valid rest pose under a different solver is a **fidelity FACT, not a Newton bug**
  — and it is **untunable**: a measured substep sweep on panda made it *worse*, not better (substeps 4→8→16
  drove the props 0.27 m → 7 m → 16 m as XPBD ejected them; xpbd-iters and the MuJoCo solver were both inert).
  So the meter now **classifies each root** (`WbSolid::postPhysicsStep` tags the probe line `A` for a
  `WbRobot`, `R` for a free-prop/static root — *not* "has a joint", which mis-tags an articulated prop like a
  hinged-handle bucket) and `faithful_check.py` reports a **ROBOT headline** with the free-prop bucket as
  context. **Honest robot-fidelity reading (8 robot worlds + the cylinder husky, 2026-06-08): 5/8 FAITHFUL**
  — rosbot 3 mm, rosbot_xl 5 mm, the two cobot-arm worlds 0 mm, **panda 0 mm** (the old "887 mm" was *entirely*
  the paint bucket); ur_arms drift 62 mm (arm diverges late, chaotic), husky drift 37 mm (contact-stiffness,
  shape-independent — same for sphere or capsule). The 3 non-faithful are now correctly *attributed*, not
  contact bugs: **mavic** DIVERGE 3.98 m is an **uncontrolled drone in freefall/tumble** (no controller),
  **spot** is `NO DATA` (the known `spot.urdf <rest>` parse collapse → use `spot.classic.urdf`). *Honest
  limitation: the probe dumps ROOTS only, so a fixed-base arm holding still reads "faithful" by base-stability;
  per-link arm tracking is a deeper future measurement. Remaining true contact-fidelity gap = husky/ur_arms
  small drift (W5.1/W5.2 solver-select + substep auto-tune), NOT panda.*

### W6 — Legged / dynamic fidelity + RL deploy (the research lane)
The hardest, least-bounded class. Tracked in [rl-current-state.md](rl-current-state.md): G1 deploys but
falls @1.55 s (inherent inverted-pendulum instability → needs *train-in-the-deploy-solver*, foundation
built, trainer unwritten); Atlas never deployed; learned residual ≈ a passenger on Spot. Sequence **last**,
run as a measured research track, and **do not let it block** the A–C coverage climb (a world can run
*faithfully* on Newton without a *good controller* — these are different bars).

**Spot deploy collapse — characterized 2026-06-08 (regression check, easy causes ruled out).** A headless
run of `spot_residual_deploy_newton.wbt` (classic URDF) shows Spot **spawns standing (z=0.70) and collapses to
z≈0.11 within ~3 s** under Newton, drifting backward — while **under forced-ODE it walks +1.43 m in 8 s and
holds z≈0.67** (matching the documented gait). So the controller/gait/world are fine; the failure is
Newton-specific dynamic fidelity. Ruled out: (a) NOT a regression from the W1/W4/W5 work — those are gated/
inert for Spot (0 cylinders → W1.2 inert; native contacts opt-in-off; probe read-only), and the native-config
run uses effectively-baseline physics yet still collapses; (b) NOT torque control — Spot uses position control
(`setControlPID Kp=20` + `set_pose`), so the dead-`joint_f` finding doesn't apply; (c) NOT position stiffness —
`OMNISIM_NEWTON_TARGET_KE`=500 and 2000 both still collapse; (d) NOT solver-specific — collapses under XPBD
*and* MuJoCo. So the legs buckle under load despite tracking the gait, for a reason deeper than gains/solver —
a genuine W6 dynamic-fidelity gap (likely the position→velocity actuator bridge or foot-contact dynamics under
load). The RL doc's "deploy without falling" headline is **ODE-accurate but stale for the Newton path**.
**LOCALIZED 2026-06-08 → it's the CONTROL BRIDGE, not contact.** A per-link dump (`OMNISIM_PROBE_TRAJ_ALL`,
all solids not just roots) at t≈4 s shows the WHOLE leg chain collapsed near the floor under Newton (hip 0.18,
upper_leg 0.25, lower_leg 0.06, body 0.11) vs extended under ODE (lower_leg 0.34, body 0.67) — and the **feet
do NOT penetrate** (lowest leg point 0.034 m > floor 0). So the legs simply aren't tracking the commanded
standing pose under Newton; they buckle. Since `OMNISIM_NEWTON_TARGET_KE`=500/2000 didn't fix it, it's not
merely missing position stiffness — the position TARGET isn't holding (suspect: `joint_target_pos` not driving
the leg DoFs under load). **REFINED 2026-06-08 → it is BODY ROLL / lateral stability, not joint tracking.**
A per-joint dump (`OMNISIM_DEBUG_JOINTS`, target_pos vs joint_q vs ke) settled it: the position **targets
reach correctly** (target_pos = the crouch: hip −0.6, others ±0.3), and with adequate gain the **joints DO
track** — at `ke=8000` the per-joint error drops to ~0.06–0.11 rad (vs ~0.2–0.4 rad at the live default
`ke=800`/`kd=500`). Yet Spot **still collapses to z≈0.11 at ke=8000** — i.e. the legs are at (nearly) the
commanded angles but the **body falls/tips anyway**. So it is NOT joint-position control: it is floating-base
roll/lateral instability (the code's own comment, lines ~748: "roll climbs 0.1→2.3 rad and the chassis sinks
0.61→0.23 m"). Everything tunable was ruled out: contact penetration (feet at 0.034 > 0), gain (`ke` 800→8000),
damping (`kd` 30/500/2000), the "proven" `ke=1500/kd=30` (collapses now too — it has DRIFTED, the live default
is `ke=800/kd=500`), and the per-joint effort cap (`OMNISIM_NEWTON_NO_EFFORT_LIMIT=1` doesn't help). **Net:
the W6 Spot gap is lateral/roll dynamic fidelity** — leg forces + foot-contact friction don't supply enough
lateral restoring torque to keep the chassis upright under Newton, where ODE does. That's genuine dynamic
fidelity (the hard lane), NOT a quick control fix — though restoring the drifted `ke=1500/kd=30` default is a
worthwhile cleanup regardless. Diagnostic levers left in place (all gated/inert): `OMNISIM_PROBE_TRAJ_ALL`
(all-solids dump), `OMNISIM_DEBUG_JOINTS` (per-joint target/actual/ke), `OMNISIM_NEWTON_NO_EFFORT_LIMIT`.
**REFINED AGAIN 2026-06-11 → it is the CONTROL BRIDGE, not MuJoCo's physics ("dynamic fidelity" was the wrong
verdict).** Decisive A/B: the SAME Spot model (the May-22 Newton-dumped MJCF `C:\tmp\spot_newton_fixed.xml`,
kp=250/kv=60 actuators) with the SAME residual policy run in STANDALONE mujoco_warp
(`gpu_mjwarp_residual_trainer.py --eval`, 64 envs) **balances indefinitely — 512/512 steps survived, zero
falls** — while the identical model+gains+engine through the OmniSim deploy bridge falls at ~1.2 s. The fall
signature (chassis trace) is a **growing fore-aft pitch oscillation** at ~2.5 Hz (0.14 → 0.43 → 0.50 rad over
three cycles from a clean seeded crouch at bz=0.60) — an energy-injecting limit cycle in the per-tick
bridge↔solver interaction, not a contact/friction deficit. A 12-variant knob sweep (2026-06-11, model-only
walker, `projects/policies/research/tools/spot_newton_knob_sweep.py`) all fail identically: substeps 4, statics, base-guard on/off,
seed-rebuild, kd 60/500, ke 250/1500, mjwarp + CPU mj_step + XPBD, `MESH_TO_ODE=1`, joint-clamp off. Library
drift ruled out (newton 1.2.0 / warp 1.13.0 / mujoco 3.8.1 all installed 5/17–18, before the verified 5/24
walk). Note G1 walks through the same bridge (different gains kp100/kd5, MJWARP), so it is a Spot-shaped
bridge bug (suspects: interleaved [pos,vel] ctrl writes for the 12-joint quadruped, the per-tick joint-limit
clamp interaction, or the 2026-06-11 `c57645af` sync-elision rewrite — UNTESTED pre-c57645af, the collapse
predates it but the binary under test included it). Next step: per-tick diff of bridge ctrl/state writes vs a
standalone mjwarp run from the same initial state.
**RESOLVED 2026-06-11 (same day) → it was SELF-COLLISION, introduced by W1.** The smoking gun was the live
mjModel introspection dump (new gated lever `OMNISIM_NEWTON_DUMP_MJMODEL=<path>`; note world finalize happens
~5 s after launch, so headless probe runs must be ≥6 s or the dump never fires — this also explains earlier
"SAVE_MJCF silently writes nothing" reports): MuJoCo's contacts at step 1 were `(chassis, upper_leg)×4`.
W1's native mesh registration (1adbcb69, Jun 7 — the day before the collapse was observed) turned Spot's 9
URDF collision MESHES into real MuJoCo geoms for the first time, and Spot's chassis hull overlaps all four
upper-leg hulls at the standing crouch. The resulting permanent internal wrench shoved the robot backward and
tipped it — and it poisoned every knob in the earlier "ruled out" list (the ke=8000 test collapsed *because
of the wedge*, not despite gains). ODE/Webots semantics never have intra-robot contacts (Robot.selfCollision
defaults FALSE); the fix (in `78e555c4`'s tree) makes the bridge match: at finalize, every intra-robot shape
pair (robot = connected component of the articulated-joint graph) is added to
`builder.shape_collision_filter_pairs`, which BOTH contact paths honor (newton broad-phase directly,
SolverMuJoCo via its graph-coloring into contype/conaffinity → verified `nexclude=78` for Spot's 13 bodies).
Opt-out: `OMNISIM_NEWTON_SELF_COLLISION=1`. Verified after the fix: Spot STANDS 100% upright / no falls
(`OMNISIM_NEWTON_TARGET_KE=500 KD=60..200`, the old 250 is too soft now — May's "working 250" leaned on the
pre-W1 AABB-box colliders), and gait-stepping holds 30 s upright at `KE=500 KD=200 SUBSTEPS=8`.
**⚠️ Scope:** this "stands / no falls" verification is a *babysat bench* (self-collision filter ON, hand-set
`KE=500 KD=200 SUBSTEPS=8`) — it confirms the self-collision *fix*, NOT a durable deploy. The canonical
[rl-current-state.md](rl-current-state.md) still records the **Spot RL deploy as COLLAPSING under Newton**
(z≈0.70 → ≈0.11 in ~3 s, lateral/roll dynamic-fidelity gap); that doc is authoritative when the two disagree.
Regressions
checked: G1 walk A/B'd with the filter on/off (+23.8 m/fall@58.5 s vs +26.1 m/fall@64.8 s — within run
variance; trainer mean-first-fall is 36.5 s), AnyPick 12/12 emptied+sorted PASS, joint-limit stress test
PASS. Remaining W6 work is RE-TUNING, not a bug: every pre-Jun-7 Spot walking policy/gait was implicitly
trained against the phantom box colliders and must be retrained on the honest geometry (the MJCF dump +
`gpu_mjwarp_residual_trainer.py` pipeline is ready; trainer joint classifier fixed for the widened ±1.5
hip_x range).
**RE-FRAMED 2026-06-23 → much of the observed "Newton deploy collapse" was a SILENT ODE FALLBACK, not a
Newton dynamic-fidelity wall.** Root cause (fixed in `6a459f84`, [WbNewtonBackend.cpp](../../src/omnisim/physics/WbNewtonBackend.cpp)):
under a headless DEVNULL-stdout launch, warp's startup banner wrote to a `None` `sys.stdout`, so the Newton
FFI smoke (`newton.ModelBuilder()`) raised `'NoneType' object has no attribute 'write'`, the engine **silently
fell back to ODE**, and Newton-tuned worlds collapsed — looking exactly like a Newton failure while actually
running ODE. The fix adds a writable-stdio guard before the warp import, so **Newton now reliably ENGAGES
where the runtime is present**. With Newton actually initialized, **Spot/Go2/B2 WALK on Newton-MuJoCo**
(Spot +30 m, Go2 +66 m, B2 +95 m, 0 falls). The real "Newton drove it" signal is the
`[WbNewtonBackend] world finalised (solver=...)` log line — NOT `imports OK`. The W1 self-collision fix above
is still real and relevant; the point is that the *deploy collapses* people saw were OFTEN the fallback, not
purely a Newton dynamic-fidelity gap. To catch a silent fallback going forward, set `OMNISIM_REQUIRE_NEWTON`
(opt-in fail-loud guard, `cfb11d06`): `WbLog::fatal` + non-zero exit when Newton init fails, instead of
silently degrading to ODE. *(Note: the [rl-current-state.md](rl-current-state.md) Spot-deploy headline should
be re-checked under a guaranteed-Newton run before being trusted as a Newton result.)*

### W7 — The flip + ODE's permanent role
When the meter clears the bar (coverage ≥ 99% within tolerance), make Newton the resolved default for the
covered set (the gate routes to ODE only on explicit `physicsBackend "ode"` or genuine residual edge cases),
and **document ODE's permanent role**: the byte-identical fallback + the oracle every Newton change is still
measured against. This is the physics analogue of the render Phase-ζ flip — gated on the parity bar, not the
calendar.

---

## 4. Sequencing (dependency- and value-ordered)

Each phase is independently shippable, leaves the tree green, and ODE stays the default until W7.
*[Correction 2026-07-09: the default has since flipped — `physicsBackend "auto"` now resolves to
Newton where the runtime is present, with ODE as the permanent fallback; see the status banner at
the top of this doc.]*

1. **Phase N-MEASURE — W0.** Corpus + coverage meter. *Unblocks measurement of everything; cheap; do first.*
2. **Phase N-SHAPES — W1.** Plane → native cylinder → **mesh (the big unlock)** → heightfield. Biggest meter
   jumps live here.
3. **Phase N-JOINTS — W2.** Fixed → ball → hinge2 → motors. Parallelizable with N-SHAPES (different gate tier).
4. **Phase N-CONTROL — W3.** The force-injection design spike + go/no-go first, then the rest. The spine.
5. **Phase N-CONTACTS — W4.** Native contacts → retire the ODE bridge. Needs N-SHAPES (Newton must collide).
6. **Phase N-FIDELITY — W5.** Solver auto-select + stability + the parity bar. Runs continuously; one focused pass.
7. **Phase N-DYNAMIC — W6.** Legged/RL deploy. Research lane, last, non-blocking.
8. **Phase N-FLIP — W7.** Default everything Newton-covered; ODE permanent fallback/oracle.

**Estimate:** N-MEASURE + the cheap half of N-SHAPES/N-JOINTS is weeks. **Mesh (W1.3), force-injection
(W3.1), and native contacts (W4) are the multi-month core** — and each carries a real risk that Newton's
model or the upstream library, not our effort, is the limiter (the explicit go/no-go points). N-DYNAMIC is
open-ended research. So: **the coverage meter reaches the 80s relatively fast; the last ~15–20% is the long,
genuinely-hard tail** — exactly why this is a separate plan from the baseline.

---

## 5. Verification (reuse what exists)

- **Per-change parity:** [physics_oracle.py](../../scripts/dev/physics_oracle.py) (ODE vs Newton per-body
  trajectory diff) + the [dual-backend oracle](../../scripts/dev/dual_backend_oracle.py) — already standing
  in the pre-push hook (`--gate --require-newton`). Every gate-widening must keep these green.
- **Coverage tracking:** the W0.2 meter, run after every workstream; its % is the headline progress number.
- **No-regression:** ODE stays byte-identical throughout (the forced-ODE smoke gate); `OMNISIM_LEGACY=1`
  reversibility (already proven) stays green.
- **Catch silent fallback:** wire `OMNISIM_REQUIRE_NEWTON` (the `cfb11d06` fail-loud guard) into the
  faithful-check (W5.4) / deploy harness so a silent ODE fallback fails loudly instead of being mis-read as a
  Newton result — without it, a runtime/init regression (e.g. the `6a459f84` warp-banner-vs-DEVNULL-stdout
  bug) reads as "Newton collapses" while ODE is quietly driving. The real engaged signal is the
  `[WbNewtonBackend] world finalised (solver=...)` log line, not `imports OK`.
- **Acceptance per workstream:** the closed gate branch's worlds resolve to Newton AND land within the W5.4
  parity tolerance on the oracle, with the meter's fallback histogram showing that reason retire.

---

## 6. Honest risks & the "is 100% actually reachable?" call

- **Class B may not fully close.** Mid-step CFM/ERP and arbitrary force injection are maximal-coordinate
  idioms; Featherstone Newton may never reproduce them bit-for-bit. The faithful-enough bar (W5.4), not
  byte-identity, is the target — and a small set of force-physics worlds may *legitimately* stay on ODE.
- **Upstream gates us (class E).** Mesh narrow-phase, contact readback, the body-index-N cliff, and exotic
  joints depend on NVIDIA Newton/Warp. Part of this plan is tracking/requesting/contributing upstream — not
  all of it is ours to land on our schedule.
- **D is research.** Legged deploy fidelity has no guaranteed completion date.
- **Therefore the honest target** is *not* a literal 100% with ODE deleted. It is: **the gate's
  ODE-fallback rate driven to ~0 on the corpus, Newton the default everywhere it's faithful, and a small,
  documented, deliberate ODE residual** for the genuine edge cases — with ODE permanent as fallback +
  oracle. That *is* a "complete replacement" in every sense that matters operationally, stated honestly.

---

## 7. Relationship to the other plans

- [engine-migration-plan.md](engine-migration-plan.md) §13 — the P0–P8 physics arm this extends; the
  dispatcher, build flag, schema flip, and cross-backend bridge it relies on are already landed there.
- [architectural-baseline.md](architectural-baseline.md) — the now-complete *structure*; this plan is the
  capability *completeness* that baseline explicitly scoped into the §4 "out of scope" tail.
- [rl-current-state.md](rl-current-state.md) — the source of truth for W6 (legged/RL deploy).
- [dispatcher-surface-signoff.md](dispatcher-surface-signoff.md) §1 — confirms every op here is an
  *override on the existing `WbPhysicsBackend` surface*, never a new virtual: this plan is capability, not
  architecture.
