# The Newton capability frontier

*What NVIDIA Newton can do, what OmniSim reaches today, and what it would cost to close
the gap. Every verdict here is measured on this tree, not read off upstream docs.*

**Measured 2026-08-14** on machine `9722d23d12a3` (RTX 3060 Laptop 6 GiB sm_86, AMD 16-core,
Windows 11), vendored runtime **newton 1.5.0 / warp-lang 1.16.0 / mujoco 3.11.0 /
mujoco-warp 3.11.0**, engine build `9583b4b8c`..`60dc03709`.

Read this alongside:
- [docs/guide/newton-physics-backend.md](../guide/newton-physics-backend.md) — the backend as it ships today
- [docs/benchmarks/lane4-capability-matrix.md](../benchmarks/lane4-capability-matrix.md) — the **generated, executed** capability matrix; when it and this file disagree, **the matrix is the measurement**
- [docs/developer/cloth-simulation.md](cloth-simulation.md) — the first deformable, and the template every other one follows

---

## 0. The one-paragraph answer

Newton ships **9 solvers** and ~90 examples across **16 families**. OmniSim currently drives
**two** of them: `SolverMuJoCo` for all rigid physics, plus `SolverVBD` for **cloth** (landed
2026-08-14) and **volumetric soft bodies** (`f7b03f54c`, 2026-08-15). Terrain landed alongside.
The remaining families — cables, MPM granular/snow/fluid, batched IK, `ArticulationView`, the
sensor set, SDF and hydroelastic contact, differentiable simulation — are **not blocked by
Newton**. A 37-probe census found **33/37 capabilities work on CPU and 34/37 on CUDA**; the gaps
are almost entirely unwired OmniSim bridge, not missing engine support.

**The single most useful structural fact in this document** is the one that separates the two
remaining deformable families, because it decides their cost: a capability that adds only
PARTICLES (cloth, soft bodies, MPM) leaves the coupled solver's `mjc` ModelView owning every
rigid body, so the raycast / weld / `TouchSensor` readbacks stay valid and the work is a node
plus a gate. A capability that adds rigid BODIES (Cosserat rods) moves them out of `mjc` and
silently invalidates those readbacks. Check which side a new primitive falls on before
estimating it.

---

## 1. The measured census

37 probes, each building the smallest model the capability needs and stepping it. Script:
`scratchpad/probe_solvers.py` (not in-tree). **CPU 33/37, CUDA 34/37.**

⚠ **Read the calibration note before quoting any FAIL.** The first pass produced ~10
"failures" and most were the probe's fault, not Newton's — `Heightfield`'s `hx/hy` are field
half-extents not cell size; `Mesh.create_box` takes half-extents; a muscle anchored twice to
the same body early-returns; an undamped D6 slider sampled at an arbitrary instant looks like
a broken limit. **A `broken` verdict is not publishable until it has been chased to a
mechanism.** That rule is the same one OmniBench lane 4 enforces.

### Works, both devices

Rigid solvers `SolverMuJoCo` / `SolverXPBD` / `SolverFeatherstone` / `SolverSemiImplicit` /
`SolverKamino`; VBD cloth; Style3D cloth; tet-FEM soft bodies; Cosserat rods; ImplicitMPM
granular; heightfields; ellipsoids; convex hulls; Gaussian splats; ball / D6 / cable joints;
mimic constraints; IMU, contact and tiled-camera sensors; frame-transform sensor; `IKSolver`;
`ArticulationView`; `add_mjcf` and `add_usd`; and the coupled MuJoCo+VBD, MuJoCo+MPM and ADMM
solvers.

Selected evidence: contact sensor reads **19.6200 N** against a 19.62 N resting weight
(error 0.0000); IMU reads **9.8100 m/s²** at rest and 0.000 in free fall; IK residual
**0.000001 m**; `ArticulationView` write/read round-trip error **0.00e+00**.

### The real limits

| capability | verdict | mechanism |
|---|---|---|
| **SDF / hydroelastic contact** | **CUDA-only** | `builder.py:11412` raises on CPU — the texture-SDF build uses `wp.Volume.allocate_by_tiles` + `wp.Texture3D`, both CUDA-only. This is the **only** CPU/GPU capability split in the census. |
| **Cosserat rod on `SolverMuJoCo`** | **absent** | `NotImplementedError: Joint type JointType.CABLE is not supported yet` (`solver_mujoco.py:6744`). `supported_joint_types` (`:5648`) is `{FREE, BALL, PRISMATIC, REVOLUTE, D6}` and the fallthrough **raises**. Matches the solver's own docstring at `:446`. **Rods must ride VBD.** |
| **`add_muscle`** | **broken** | activation 0 vs 500 N gives **delta = 0.000e+00 exactly**, both solvers. `eval_muscle_forces()` sits behind a literal **`if False:`** in `solver_semi_implicit.py:187` **and** `solver_featherstone.py:561` — the kernel in `kernels_muscle.py` is never launched. `add_muscle` builds model data nothing consumes. |
| **DISTANCE joint** | **three-way split** | honoured on XPBD (z=1.0000 exact); **silently ignored** on SemiImplicit and Featherstone (−2.9254, i.e. free fall); refused loudly on MuJoCo (`NotImplementedError: Joint type 5`). The silent free-fall is the dangerous case. |
| **conveyor surface velocity** | **absent** | newton 1.5.0 has no surface-velocity material property at all. Both upstream conveyor examples emulate it — one spins a kinematic belt, the other reads back per-contact normal forces and applies its own tangential wrench. |

Robustness note, not a capability: newton **crashes rather than raising** on a device-mismatched
`Contacts` buffer (`Contacts(...)` without `device=` defaults to `cuda:0` while the model is on
CPU; `update_contacts` then dies natively with no Python exception).

---

## 2. What OmniSim reaches today

Checked against the vendored `newton/_src/sim/builder.py`, not upstream docs.

| | wired | not wired |
|---|---|---|
| **Shapes** | sphere, box, capsule, mesh | ellipsoid, cone, convex hull, SDF, Gaussian, site |
| | ⚠ **cylinder is substituted by a capsule** (native cylinder narrow-phase locks wheels) | |
| **Terrain** | ✅ **heightfield — landed 2026-08-14** (§3) | |
| **Planes** | implicit z=0 ground only | an authored `Plane` is **silently dropped**; a raised or tilted one is replaced by the implicit plane |
| **Joints** | revolute, prismatic, fixed, free | distance, cable; ball and D6 **register but do not actuate** |
| **Deformables** | cloth (`add_cloth_grid`), ✅ **soft bodies (`add_soft_grid`, `f7b03f54c`)** | rods, MPM particles |
| **Sensors** | — | `SensorContact`, `SensorIMU`, `SensorTiledCamera`, `SensorFrameTransform` |
| **Control** | — | `IKSolver`, `ArticulationView`, joint-impedance controllers |
| **Importers** | URDF (OmniSim's own) | `add_mjcf`, `add_usd` |
| **Other** | — | mimic constraints, muscles, actuators, sites, record/replay, sleeping islands |

Two cross-cutting limitations worth knowing:

- **`add_shape_*` passes a translation-only transform everywhere.** Geometry *rotated inside*
  a `boundingObject` is unhandled.
- **Deformable state has no readback surface.** No supervisor, controller or harness endpoint
  can describe a `Cloth` or a `GranularGroup`. OmniBench lane 4 caps `GranularGroup` at
  `PARTIAL` for exactly this reason, and cloth is verified out-of-band through
  `OMNISIM_CLOTH_TELEMETRY`. **A generic particle/field accessor would unblock two existing
  nodes at once and is a prerequisite for any capability-matrix row above PARTIAL.**

---

## 3. Landed: terrain (`60dc03709`)

An `ElevationGrid` boundingObject registered **no Newton collider at all**. It derives from
`OmGeometry`, not `OmTriangleMeshGeometry`, so `OmSolid`'s boundingObject if/else chain ran off
the end — silently, no warning, exit 0.

Measured before the fix on the lane-4 probe: a 1 kg sphere dropped on a flat grid reached
**z = −42.89 m and was still accelerating at t = 3 s**. Before 2026-08-12 the implicit z=0
ground plane masked the same defect as a plausible rest at z = 0.0996, which is why a dead
terrain collider survived so long.

Now bridged to `newton.Heightfield` + `add_shape_heightfield`. Three conversions, none of them
guessable, all documented at the call sites:

1. `hx`/`hy` are field **half-extents**, not cell spacing; OmniSim authors terrain as a cell
   size plus a sample count, so the span is `spacing * (dimension - 1)`.
2. VRML puts the grid's **(0,0) corner** at the geometry origin; newton **centres** the field.
3. `add_shape_heightfield` takes **no body argument** — a newton heightfield is always
   world-static, so the owning Solid's transform is baked in. ⚠ **An `ElevationGrid` on a
   moving body will not follow it.** That is a newton constraint, not a choice.

Verified three ways: runtime-exact (flat grid rests a sphere at **0.6499** against 0.6500; the
same grid raised 0.3 rests it at **0.9499** — a delta of exactly **0.3000**, which a heightfield
ignoring its elevation data cannot produce); lane 4 flipped `broken → WORKS`; and a demo world,
[`newton_heightfield_terrain.omniworld`](../../projects/samples/demos/worlds/physics/newton_heightfield_terrain.omniworld),
passes `--fail-on-runaway` over 84.8 s.

Two defects fixed en route: `OmElevationGrid::height(int)` returned `int` over an `OmMFDouble`,
**truncating every elevation to a whole metre** (its one existing caller is the
camera-recognition / radar occlusion point set, which therefore saw flat terrain); and a
leftover `dHeightfieldDataID` forward-declaration of a deleted ODE type.

⚠ **A heightfield is a finite patch with no skirt and no floor beneath it.** A body that leaves
the footprint falls for ever. The demo world is a rimmed bowl for exactly this reason: its
first draft used rolling hills and asserted a per-ball rest height under each drop point, and
measuring showed balls do not stay where they land on a slope (three rolled into a common
hollow) while the fourth rolled off the 12 m edge and fell to **z = −981**.

---

## 4. Proven but unwired

Each recipe below was driven through OmniSim's **own** `World` class, not a standalone newton
script, so the numbers describe what a node would actually get.

### 4a. Rope / cable — works, but has a structural blocker

Measured: a rope pinned at one end holds its anchor at **(0.000000, 0.000000, 1.500000)** after
10 s of swinging; **0.07 %** stretch under its own weight at `stretch_stiffness 1e5`; rests on
the ground at **z = 0.01981** against a predicted 0.02; drapes over a static box with 7 nodes at
**z = 0.21993–0.22395** against a predicted 0.22.

⛔ **THE BLOCKER.** A rod is a chain of **rigid capsule bodies**, so a `Rope` node must give
those bodies to the VBD entry — and that breaks the index-identity assumption at
[`omnisim_newton_runtime.py:3190`](../../src/omnisim/physics/omnisim_newton_runtime.py).
Raycast, welds, `TouchSensor`, the pose check and the contact readback all index by **parent**
model body id, which is only sound while the `mjc` `ModelView` owns every body. In the drape
test **mjc owned 2 of 26**. The guard **logs a warning; it does not raise** — so a rope-plus-robot
world would answer those sensor queries about the wrong body with only a log line. Fixing it
means remapping through `body_global_to_local`, or refusing those readbacks on a rope world.

A **rope-only** world sidesteps this entirely (VBD alone, no coupling).

### 4b. Volumetric soft body — ✅ **LANDED** (`f7b03f54c`), and it was a small job

✅ **The identity check stays intact**: a soft body adds **particles only**, so `mjc` keeps 100 %
of bodies (measured: model bodies 2, `view("mjc")` bodies 2). Unlike rope, no structural change
to `_build_cloth_coupled_solver` is needed — the single edit is generalising `has_cloth()` to
mean "has particles".

Measured: a pinned face holds **1.200000 exactly** while the free end sags **16.2 mm**; a dropped
block rests with its lowest particle at **z = 0.009182** against a 0.01 particle radius, no
tunnelling; a 15.6 kg soft block presses a 2 kg dynamic box from 0.060000 to **0.059385** and
holds — genuine two-way coupling.

⚠ **Rigid-on-soft fails**: a rigid body supported *only* by particles gains energy and is
ejected (z = 0.57–194 m). Attributed properly — it reproduces in **pure newton with OmniSim
absent**, in both coupled and non-coupled solvers, is not a timestep artifact (8× smaller dt
made it worse), and is not self-contact. It scales with `soft_contact_ke`, which points at the
particle↔rigid penalty response. **Newton-side, mechanism unidentified.** The usable asymmetry:
soft-on-rigid is stable, rigid-on-soft is not.

### 4c. MPM granular — works, identity-safe, but **GPU-only in practice**

✅ **Identity-safe, like soft bodies.** Measured at 1 and 3 bodies: `view("mjc").body_count`
equals the parent model's (1/1, 3/3) and `body_xform(parent_id)` returns the correct distinct
poses. The `mjc` entry declares every body and joint; the MPM entry declares **particles only**.
The `mpm` view's own `body_count` are *proxy* bodies the mapping **adds** on the destination
side — additive, never subtractive. `setup_collider()` is **not called at all** on the coupled
path, so it cannot break the identity either.

Also measured, and **weaker than upstream documents**: `register_custom_attributes` does *not*
have to precede the first particle. Registering after particles exist does not raise, the
attribute arrays are back-filled with defaults, and per-particle values authored later are
still correct across the seam (a cloth grid then a granular grid gives `friction[0]=default`,
`friction[-1]=0.9` as authored). **The real constraint is "before `finalize()`."** OmniSim's
scene-tree ordering is therefore a non-issue — register lazily on the first granular call.

**Sand pile**, CPU, 2 744 particles: column height 0.4153 → 0.1750 m, spread 0.2874 → 0.5786 m,
deposit flank **16.8°**, settled by ~step 80 and then exactly static (0 drift over 40 steps),
**0 particles below the floor**. Friction is genuinely plumbed: μ = 0.20 / 0.45 / 0.68 / 1.00
gives flank **0.8 / 12.0 / 20.9 / 27.8°** and runout **1.834 / 0.920 / 0.740 / 0.640 m** —
monotone in both, with the deposit angle below `atan(μ)`, which is correct for a collapse.

**Two-way rigid coupling works** via `SolverCoupledProxy`. A 30 kg box rests at **z = 0.27070**
against a 0.09989 floor-rest — held 0.17 m up by the sand — and at 120 kg the bed is depressed
under it (0.1293) while piling in the annulus around it (**0.1400 > 0.1293**). The one-way
collider variant is genuinely one-way: the sand deforms identically but the box falls to
0.09989, byte-identical to the no-sand control.

⚠ **Cost makes this a GPU feature.** Adding MPM to a rigid tick measured **0.45 → 60–67 ms/step,
about 150×**. CPU-only tops out near 3 k particles at 42–67 ms/step, i.e. **0.15–0.25× realtime** —
a slideshow. On CUDA the cost is nearly **flat in N** (a fixed per-step floor, not particle
count): 2 197 → 30.9 ms, 10 648 → 33.0, 50 653 → 35.9, 405 224 → 67.4. So the ceiling is the
~30 ms/step floor at *any* N below ~400 k, not memory or particle count.

**Recommendation for re-backing `GranularGroup`** (today an M2 skeleton on a bespoke ~1 275-line
NVRTC CUDA kernel): do it, wire coupling mechanism (a) `SolverCoupledProxy`, and **gate it on a
CUDA device, refusing loudly on CPU** rather than running at 0.2× realtime — the same "a wrong
result is worse than a lost one" rule ODE was retired under, applied to an unusable one. It
buys real rheology, two-way coupling the bespoke kernel does not have at all, readable state
(closing the lane-4 `PARTIAL`), and hands maintenance to NVIDIA.

⚠ **The honest counter-argument:** that flatness cuts both ways. `GranularGroup`'s default is
`count 1000` — exactly the regime where a fixed ~30 ms/step floor is worst value and a bespoke
single-launch kernel should win outright. Newton MPM **cannot** get below ~30 ms/step on a
sparse grid at any N on this machine. If the shipping requirement is "1 000 particles at
100 Hz", keep the bespoke kernel. If it is "sand a robot can actually interact with, at 10⁵
particles, that we do not maintain", MPM wins decisively.

Keep `grid_type="sparse"` as the default. `"fixed"` plus CUDA-graph capture is 2.6–5.0× faster
but **NaNs silently** — every particle non-finite, no error, no warning — when `grid_padding` is
too small for the material's excursion, and its cost swung 14.3 → 30.2 ms on a padding change
alone. Expert opt-in only.

**A full implementation exists and is STAGED, NOT LANDED.** The node rewrite, the new schema, the
backend/runtime blocks and a sand-pit demo were written and are preserved out-of-tree. They were
not committed because the change deletes ~1 275 lines of the existing bespoke NVRTC/CUDA kernel,
orphans seven `tests/cuda/*` files that test it, and was never compiled or run through
`omnisim-bin` — that combination is its own tranche with its own verification, not a tail-end
addition. Four things it measured are worth keeping regardless:

- **The cost differential is the honest headline.** 8 192 particles measured **351.14 ms/step** on
  CPU against **2.14 ms/step** for the identical world with the sand deleted — **164×**, or
  ~0.02× realtime at an 8 ms tick. That control arm is also the *test*: a silently-skipped MPM
  step reads exactly the no-sand number everywhere, so a demo must assert the difference between
  the two, not just the presence of particles.
- ⚠ **`rigidSubsteps` is not a tuning knob.** At 1, a 2 kg cube resting on the sand *gained*
  energy every bounce — 0.27 → 0.39 → 0.71 → 0.99 m. At 4 (upstream's value) it settles. Anything
  coupling a rigid body to MPM needs the substeps.
- ⚠ **Weld-pinned static Solids are not reliable MPM colliders.** Four static box walls around a
  sand bed changed its final spread by **0.002 m** against a no-walls control, while dynamic
  bodies in the same scene were visibly supported. Characterised, not explained — a `Plane`
  boundingObject (which routes through the implicit ground-plane substitution to a body-−1
  collider) worked where the welded boxes did not.
- **`count` cannot be honoured** and a schema that pretends otherwise lies: MPM's particle count is
  `Π(ceil(ppc·size_i/voxel)+1)`, so the author picks a *resolution* and gets whatever count falls
  out. Treating `count` as a **budget** — bisect the voxel size for the finest grid that stays
  under it — is honest and was verified against the emitter arithmetic (budget 1 000 → voxel
  0.1333 → 900 particles; 10 000 → 0.05625 → 9 801).

---

## 5. Two structural decisions worth taking before the third deformable lands

1. **`newtonSolver` is being overloaded.** One string now encodes *device* (`mujoco` vs
   `mujoco_warp`) **and** *solver composition* (`+vbd`). A second deformable family would need
   `mujoco+vbd+mpm`-style concatenation. Decide the orthogonal-fields redesign before the third
   value lands.

   ⚠ **And `"mujoco+vbd"` is currently a dead string**: it appears **zero** times in
   `omnisim_newton_runtime.py`. The coupled pair is selected solely by `has_cloth()`, so cloth
   couples under *any* solver value, and the schema, the `Cloth.wrl` comments and the
   `OmCloth.cpp` warning all name a cause that is not the mechanism.

2. **~25 `OMNISIM_CLOTH_*` env vars are doing work the schema should do** — contact margin, soft
   ke/kd/mu, VBD iterations, self-contact radius, proxy coupling. Promoting the stable ones into
   the node schema follows the precedent already set by 8 of the 16 `newton*` `WorldInfo` fields.

3. ⛔ **`solver_soft` is a load-bearing sentinel, and its name and comment both lie.** It is
   declared "SolverVBD instance, cloth worlds only" (`:200`) and read at **six** sites —
   `:5097, :5592, :5644, :6553, :6569, :6773` — every one of which means *"a second solver is
   live"*, not *"cloth"*. The critical one is `_mjc_batch_substeps_ok()` at **`:5097`**: the
   batched path drives `mj_step` **directly and never calls `self.solver.step()`**, and its only
   guard against a second solver is `solver_soft is not None`.

   **A future solver that forgets to assign it is silently dropped on every tick.** Measured
   during the MPM investigation with the field left unset: the box fell straight through the
   sand and rested **byte-identically to a no-sand control**, and the bed never moved — no
   exception, no log line, nothing. Any new second solver (MPM, a second VBD, ADMM) must assign
   `solver_soft`. Better: rename it `solver_second` and fix the line-200 comment.

---

## 6. ⚠ RETRACTED: the "free 3.3× on cloth" was not real

**An earlier revision of this file told you to declare `WorldInfo { newtonSubsteps 2 }` on every
cloth and soft world for a 3.3× speedup. That was wrong, and following it makes those worlds
about TWICE AS SLOW. It is retracted here rather than deleted, because the reasoning that
produced it is a trap worth being able to recognise.**

The premise was half right. `_can_graph` really does require `self._n_substeps % 2 == 0`
([`omnisim_newton_runtime.py`](../../src/omnisim/physics/omnisim_newton_runtime.py)), OmniSim's
default really is **1 — odd** — and the shipped cloth worlds really do not declare the field. So
the CUDA graph really has never armed on them. Every one of those facts checks out.

The error was concluding that the substep parity is what *blocks* it. It is not. On a coupled
cloth world the `mjc` entry is CPU `mj_step`, which copies state GPU→host, steps, and copies
back **every substep** — and a device-to-host memcpy **cannot be recorded into a CUDA graph at
all**. `_cloth_graph_ok()` refuses the capture up front for exactly this reason and says so in
its own docstring. Parity is a necessary condition sitting behind a structural one that is never
satisfied, so making the count even buys nothing and the doubled physics is pure cost.

Measured 2026-08-15, machine `9722d23d12a3`, engine-level on `newton_cloth_drape` — sim time
reached in a fixed 20 s of wall clock:

| `newtonSubsteps` | sim time reached |
|---|---|
| 1 (default) | **3.13 s** |
| 2 | **1.57 s** |

Exactly half: the doubled work, none of it bought back. Reproduced through the runtime on a 4³
soft body, both with and without a rigid body present, and the graph armed in neither arm:

| arm | substeps 1 | substeps 2 | |
|---|---|---|---|
| VBD-alone (no rigid body) | 46.65 ms/step | 98.75 ms/step | **0.47×** |
| coupled (one rigid body) | 66.33 ms/step | 92.06 ms/step | **0.72×** |

The original 21.3 → 6.5 ms/step observation **did not reproduce** and its configuration could
not be identified. It is a single unreplicated data point and should not be quoted.

**What is actually true:** an even substep count is worth declaring only where the graph can
arm — i.e. a world with **no CPU-`mj_step` entry** (`newtonSolver "mujoco_warp"`, or a
particle-only world with no rigid solver). On the ordinary `"mujoco+vbd"` coupled path the
device pin's win (CUDA VBD vs CPU VBD) is what you get, and the graph is not on top of it.
`OMNISIM_CLOTH_FORCE_GRAPH=1` re-arms the capture if anyone wants to re-measure the claim.

The two newest cloth worlds already declare `newtonSubsteps 4` and `10` for their own reasons
(integration accuracy on the coupled and whole-world-VBD paths respectively) — that is a
different and legitimate use of the field, and this retraction does not touch it.

---

## 7. What is NOT measured

Stated so nobody quotes silence as evidence:

- Rope-vs-rope contact; a rope attached to a robot link; the coupled path as a clean benchmark row.
- `add_soft_mesh` (the authored-tet-mesh path) — reachable via `newton.TetMesh.create_from_file`
  (`.vtk`/`.msh`/`.vtu`/`.npz`) and `create_from_usd`, but not run.
- A gripper pinching a soft block; `mujoco_warp` as the coupled `mjc` entry.
- The VBD-kernel cause of the rigid-on-soft ejection.
- The mechanism behind MPM's silent all-NaN on an under-padded `fixed` grid; per-body sand
  reaction via `collider_body_index`; any MPM behaviour under `mujoco_warp`; determinism of the
  coupled MPM path (the cloth path already records non-determinism on CUDA for the analogous
  architecture, so assume the same until shown otherwise).
- All CUDA cost rows quoted here are **n=1**, with a measured ±20% run-to-run spread (4 reps at
  10 648 MPM particles: 33.0 / 42.4 / 47.9 / 33.7 ms). Treat the magnitudes as soft and the
  orderings as robust.
- GPU cost rows for rope and soft bodies — dropped under this session's thermal constraint
  (see the laptop-thermal note in §8).
- Whether a coupled cloth+robot run is deterministic on CUDA (the cloth doc already flags this).

---

## 8. Working constraints on this machine

The dev laptop must stay **below 75 °C**. Only the **GPU** temperature is readable
(`nvidia-smi`); WMI exposes no CPU thermal zone and there is no settable power limit
(`power.limit [N/A]` on this mobile GPU), so **shedding load is the only lever**. Practical
rules: one GPU/physics agent at a time, `-j4` builds, and prefer CPU probes — the census found
CPU and CUDA agree on 36 of 37 capabilities. Cold warp kernel compilation is as hot as the
simulation itself: 117 modules and 478 s on CUDA against 97 s on CPU for the same 37 answers.
