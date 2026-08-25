# P8 — Statics on Newton: design

> ## ✅ GOAL ACHIEVED / ⚠️ WHOLLY HISTORICAL — 2026-08-08
>
> **P8's objective is met and its subject matter is gone.** Statics on Newton are
> **default-ON** since 2026-08-07, and `bdc02139` deleted `src/ode/` — so the
> cross-backend bridge this document exists to retire was retired outright, and W4.3
> ("dropping the ODE keepalive", called "the one remaining step" in the 2026-06-08 update
> below) went with it. Expect **no measurable per-step win** from that: removing the
> redundant ODE pass measured **−0.0116 ms/step with signs disagreeing** — noise
> ([step-cost](../benchmarks/step-cost-2026-08-06.md)).
>
> ⚠️ **Everything below §1 is design rationale for a machine that no longer exists, and it
> reads as present tense.** The 2026-06-08 update already said "treat the rest as design
> rationale, not current state", but that caveat is easy to miss twelve sections later, so
> to be explicit — **none of the following is true any more**: Newton owns dynamic bodies
> while ODE owns statics; Newton-body AABBs copy to ODE each step as kinematic proxies; ODE
> runs full collision detection; there are two collision pipelines to maintain; bridge
> state can drift; `physicsBackend "ode"` is a valid field value; worlds can mix Newton and
> ODE Solids; ODE computes `dContactGeom.depth`; `dWorldStepAndSpaceCollide` /
> `odeNearCallback` exist.
>
> Two exit criteria below are now **unsatisfiable and should not be pursued**: "test worlds
> that mix Newton + ODE Solids in the same scene — the bridge MUST still work for these",
> and "if it blows up on dense static scenes, **fall back to ODE for the static phase**"
> (there is nothing to fall back to; any mitigation must be Newton-side). Likewise the
> compatibility matrix rows for pure-ODE and mixed worlds, and "verify contact behaviour
> against ODE for arbitrary boundingObject shapes" — there is no reference to compare
> against.
>
> Record: [ode-retirement-campaign.md](ode-retirement-campaign.md).

> **Update 2026-06-08 — partially landed since this 2026-05-28 scoping; "No code
> lands here yet" is superseded.** The P8.1 statics-on-Newton API is built:
> `OmNewtonBackend::addStaticBody`
> ([OmNewtonBackend.hpp:124](../../src/omnisim/physics/OmNewtonBackend.hpp)),
> driven by `WorldInfo.newtonStatics` / `OMNISIM_NEWTON_STATICS`. The
> bridge-retirement this doc motivates is now tracked as **W4** in
> [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md): native Newton
> contacts (`getContacts`) are built + wired (W4.1/W4.2); dropping the ODE
> keepalive for pure-Newton worlds (W4.3) is the one remaining step. Canonical
> status: [engine-migration-plan.md §8.1](engine-migration-plan.md). Treat the
> rest of this doc as the design rationale, not current state.

**Status (2026-05-28):** scoping doc. No code lands here yet.

**One-line:** move all static colliders (arena floor, walls, obstacles)
onto the Newton solver alongside the dynamic bodies, eliminating the
cross-backend bridge that currently routes contact detection through
ODE.

This doc owns the design for engine-migration-plan.md's P8 row
([engine-migration-plan.md](engine-migration-plan.md) §13.3).

## 1 — Why P8 matters

The Newton physics arm shipped P0–P5 with a **cross-backend bridge**:

1. Newton owns dynamic bodies (husky chassis + wheels under articulation).
2. ODE owns everything else (arena floor, walls, static obstacles).
3. Each step, Newton-body AABBs copy to ODE as kinematic proxies.
4. ODE runs full collision detection (Newton-proxy ↔ ODE statics +
   Newton-proxy ↔ Newton-proxy).
5. Contacts queued back to Newton, applied as external impulses.

This is functionally correct (P5 verified 20v20 end-to-end through
this bridge), but it imposes three structural costs:

- **Double dispatch per step.** Every Newton body pays both Newton's
  solver step *and* an ODE-side AABB sync + collision dispatch.
  Measured ~1 ms/step overhead at 10 dynamic bodies; scales linearly.
- **Two collision pipelines to maintain.** ODE's narrow-phase
  (cylinder-vs-plane, mesh-vs-mesh, contact-property handling) and
  Newton's narrow-phase (sphere/box/capsule, simplified) both need to
  stay correct for the bridge to produce sane contacts. Bug fixes
  often need attention in both.
- **Bridge state can drift.** ODE proxies are kinematic copies; if
  Newton's body_q updates between the AABB copy and the contact
  apply, the contact point is slightly stale. Doesn't matter for
  husky-vs-husky, can matter for tight-tolerance manipulation.

When **all** Solids in a scene opt into Newton, the bridge is pure
overhead — ODE could be skipped entirely. P8 enables that.

## 2 — What's currently on Newton vs ODE

Reading `OmSolid::flushPendingNewtonRegistrations` (the gate that
decides "register this Solid with Newton") against the typical
husky world:

| Solid | `physicsBackend` | `physics` node | Joint parent | Currently registered with Newton? |
|---|---|---|---|---|
| Arena floor (`RectangleArena`) | inherited "ode" / explicit | NULL | none | No — `effectivePhysicsBackendName() == "ode"` short-circuit |
| Arena walls | inherited "ode" / explicit | NULL | none | No — same |
| URDFRobot wrapper | "newton" | NULL | none | Yes — wrapper placeholder body |
| Husky chassis (URDFRobot child) | inherited "newton" | yes | URDFRobot parent (no joint) | No — flagged as `isFixedChild` and skipped; ODE merges it into the wrapper body |
| Husky wheel | inherited "newton" | yes | HingeJoint | Yes — joint parent makes it a real articulated body |
| `top_plate_link` (URDF visual decoration) | inherited "newton" | NULL | none | No — `isFixedChild` + URDF "fixed joint" semantics |

So today Newton sees: ~5 dynamic bodies per husky (1 wrapper + 4
wheels). For 20v20 = 200 dynamic bodies. The remaining 200ish Solids
in a 20v20 world (floors, walls, top_plates, fixed-joint URDF
decorations) are ODE-only.

## 3 — Target after P8

| Solid class | Today | After P8 |
|---|---|---|
| Arena floor / walls / static obstacles | ODE | **Newton (static body)** |
| URDFRobot dynamic chassis | Newton | Newton (unchanged) |
| URDFRobot wheel / arm link | Newton | Newton (unchanged) |
| URDFRobot fixed-joint decoration | ODE merger | Optional Newton static, OR keep as ODE merger sub-body |
| Hand-written ODE Solid (`physicsBackend "ode"` explicit) | ODE | ODE (unchanged) |

The bridge stays in the code for the mixed-backend case (some
Solids ODE, some Newton in the same world). The bridge becomes a
**no-op** for full-Newton scenes — no Newton bodies need to be
proxied to ODE because ODE has nothing to collide against.

## 4 — API additions

### 4.1 — Python helper (`OmNewtonBackend.cpp::kNewtonRuntimeSource`)

Add a `add_static_body` method on the embedded `World` helper:

```python
def add_static_body(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Static (mass=0) rigid body. Newton's articulation system
    accepts mass=0 links as static; collision shapes attached via
    add_shape_* are tested against dynamic-body shapes in the
    solver's contact phase."""
    idx = self.builder.add_link(
        xform=wp.transform((float(x), float(y), float(z)),
                           (float(qx), float(qy), float(qz), float(qw))),
        armature=0.0,
        inertia=wp.mat33((0.0, 0.0, 0.0),  # mass=0 → inertia ignored
                         (0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0)),
        mass=0.0,
        label=f"static_body_{len(self.body_indices)}",
    )
    self.body_indices.append(idx)
    return idx
```

The existing `add_shape_box / sphere / cylinder / capsule` methods
work unchanged — they take a body index and attach geometry.

### 4.2 — C++ side (`OmNewtonBackend.cpp/.hpp`)

```cpp
// Static rigid body — same pose convention as addBody, but mass=0
// (Newton treats mass=0 links as immovable; collision still fires).
int addStaticBody(double x, double y, double z,
                  double qx, double qy, double qz, double qw);
```

Reuses existing `addShape*` for attaching geometry.

### 4.3 — Dispatch on `OmSolid::flushPendingNewtonRegistrations`

The current detection logic rejects Solids that:
- Have `physicsBackendName() == "ode"` after ancestor walk
- Are `isFixedChild` (no joint between this Solid and its Solid ancestor)

After P8, we add a NEW class: **static Newton body** = Solid that:
- Has `mBoundingObject != NULL` (it's a collider)
- Has `physics() == NULL` (no dynamic Physics)
- Has no joint parent (not an articulated body)
- Has `effectivePhysicsBackendName() != "ode"` (opted into Newton)

These get registered as **static** Newton bodies (mass=0) with their
boundingObject geometry attached via `addShape*`.

## 5 — Phased rollout

P8 lands in four phases, each a one-commit boundary:

### P8.1 — API + smoke — ✅ LANDED 2026-05-28

- ✅ `add_static_body` Python helper in `OmNewtonBackend.cpp::kNewtonRuntimeSource`
  (builder.add_link with mass=0 + zero inertia; reuses the existing
  `add_shape_*` methods to attach geometry).
- ✅ `OmNewtonBackend::addStaticBody(x, y, z, qx, qy, qz, qw)` C++
  binding + matching NEWTON=OFF stub.
- ✅ Newton-direct smoke probe, formerly at
  `scripts/xpbd_probes/probe_p8_static_body_smoke.py` — ⚠️ that whole
  directory was **deleted with XPBD** (`94f042225`), so the script no
  longer exists and this is a record of what it did, not a runnable
  reference. It
  built the load-bearing geometry directly via the underlying
  Newton library (one mass=0 box link + one dynamic sphere falling
  1 m onto it), runs 240 steps at dt=1/60 s, asserts the box stays
  pinned + the sphere comes to rest at `BOX_HZ + SPHERE_RADIUS`.

**Exit criterion (met):** `PASS  static_box stayed pinned
(drift=0.00e+00 m); sphere came to rest at z=0.2000 m (expected
~0.2000 m); late-stage z range = 0.0000 m`. The mass=0-as-static
assumption is sound; the P8.2 OmSolid dispatch can layer on top.

**Deferred from this slice:** an in-OmniSim .wbt smoke world.
Without P8.2's OmSolid dispatch there's no way for a .wbt to opt a
Solid into the static-body path — the Newton helper is only
reachable through `OmSolid::flushPendingNewtonRegistrations`. The
Newton-direct probe covers the technical-bet half of "smoke"; the
OmniSim-world half rides along with P8.2.

### P8.2 — OmSolid dispatch for static Solids (1-2 weeks)

**🟡 DISPATCH LANDED (opt-in) 2026-05-29 (`8acf9f1d`); exit criterion
still OUTSTANDING.**

Landed:
- `flushPendingNewtonRegistrations` detects the static-collider class
  (top-level Solid, has `boundingObject`, no Physics in subtree, opted
  into Newton) and registers it via `addStaticBody` + boundingObject
  shape. The shape walk (Pose-unwrap, primitive / cylinder→sphere /
  mesh→AABB) was extracted into the free helper
  `attachNewtonShapeFromBoundingObject()` and is shared verbatim by the
  dynamic and static paths.
- `World.finalize()` routes static bodies' free joints into a SEPARATE
  `statics` articulation (mirrors `probe_p8_static_body_smoke.py`: the
  zero-mass free DOFs must not integrate with the dynamic `webots_world`
  articulation). Each articulation's joints are created contiguously
  (Newton rejects non-contiguous joint-index ranges).
- New `OmSolid::mNewtonBodyIsStatic` flag: static bodies skip the
  per-step pose writeback AND `syncNewtonPoseFromFields` (incl. the
  `resetJointsToDefaults` that caused the chassis-freeze) — a pinned
  root collider must never touch the shared articulation.
- **Gated opt-in via `OMNISIM_NEWTON_STATICS`.** Default-off keeps
  statics ODE-side, so husky / head-on / determinism paths are
  byte-unchanged. Promote to default-on once the exit criterion below
  is met.
- New smoke world
  [`newton_static_collider_smoke.omniworld`](../../projects/samples/demos/worlds/physics/newton_static_collider_smoke.omniworld)
  (the in-OmniSim half deferred from P8.1): with the knob ON a dynamic
  Newton box rests at z=0.6 on a mass=0 static box (top 0.5 + half 0.1),
  static box pinned at z=0.25; with the knob OFF the faller passes
  through to the ground plane at z=0.1.

**Exit criterion — practical parity MET 2026-05-29** (both blockers
resolved; `newton_husky_head_on.omniworld`, the arena is a no-op static here
since RectangleArena's outer Solid has no boundingObject, so STATICS
on/off doesn't change this world):
1. ✅ **Multi-husky head-on XPBD stability** — `OMNISIM_NEWTON_SUBSTEPS`
   (`3ba5e079`). The un-frozen huskies' 50/30 rad/s head-on NaN'd XPBD at
   step 1; isolated empirically to SPEED (clearance spawn still NaN'd;
   low speed stable), fixed by sub-stepping the solver (default 1 =
   unchanged; substeps=4 finite).
2. ✅ **P6 contact-debounce** — `OMNISIM_DAMAGE_VEL_SMOOTH` (`dadd50ae`).
   The 57k-vs-149 was Newton-velocity jitter in the tracker's synthetic
   `mass*|Δv|` impulse proxy; an EMA low-pass on per-body velocity brings
   the 50/30 head-on (30 s, substeps=4, vs=5) to **57,161 → 58** events
   vs ODE-raw 39 — within the `damage_events_diff` 10× tolerance.

**Caveat:** parity is bounded by the synthetic-impulse hack (the contact
API exposes no real per-contact impulse/depth). The smoothing knob is a
Newton-side compensation, default-off so ODE damage games are unchanged.
The clean backend-symmetric fix is an engine-side contact-impulse/depth
API (ODE already computes `dContactGeom.depth` but discards it before the
controller stream) — future work, tracked in §13.3 P6.

### P8.3 — Bridge skip on full-Newton scenes — ⚠️ PREMISE INVALID (re-scoped 2026-05-29)

**Finding (code audit 2026-05-29):** the "AABB-copy + route-contacts-
back-as-impulse bridge" this section assumed **does not exist in the
code**. There is no per-step AABB-copy loop and no impulse-route-back
anywhere in `OmSimulationCluster.cpp`. The actual mechanism:

- A Newton-backed Solid keeps its ODE `dBody` + geom, but the body is
  **artificially disabled** (`OmSolidMerger::setBodyArtificiallyDisabled(true)`
  in `flushPendingNewtonRegistrations`), which also flips the geom's
  `enableForContactPoint=true`. `dWorldStep` **skips disabled bodies**, so
  ODE integration of Newton bodies is already ~free.
- Newton drives the visible pose by writing the Solid translation/rotation
  *fields* in `postPhysicsStep`; it never calls `dBodySetPosition`, so the
  disabled ODE body sits frozen at spawn and ODE never integrates it.
- The only real per-step ODE work is the single fused
  `dWorldStepAndSpaceCollide(... odeNearCallback ...)` in
  `OmSimulationCluster::step()`, which does (a) collision broad+narrow
  phase, (b) integration (~free here), (c) sensor-ray update.

**Why a "skip" is largely vacuous AND unsafe:** the collision pass (a) is
**load-bearing** — it fills `odeContacts` via `appendOdeContact`, which is
what `getContactPoints` returns to the **damage tracker**, and it is also
how **every ray sensor** (DistanceSensor, TouchSensor, Camera, Radar,
Receiver, LightSensor) detects. Skipping `dSpaceCollide` on a full-Newton
scene would silently **zero damage events and blind all ray sensors**.
Integration (b) is already skipped for disabled bodies, so there is no
meaningful per-step cost left to remove. The expected ">=5% ms/step on
20v20" win is therefore not available without breaking contacts/sensors.

**Re-scope:** P8.3 is **shelved as written.** A genuinely safe optimization
exists only for the narrow case of a full-Newton scene with **no
ODE-collision consumers** (no contact-point reads, no damage tracker, no
ray sensors) — there the whole `dWorldStepAndSpaceCollide` could be
replaced by just its maintenance tail (`swapBuffer`, `dJointGroupEmpty`,
dirty-sensor clear — which MUST still run). That is a situational win
behind an explicit opt-in flag, not the headline architectural lever the
plan implied. Recommend leaving the bridge as-is until profiling on a real
full-Newton workload shows the collision pass is actually the bottleneck.

(Original text, now known incorrect: detect full-Newton, skip the per-step
AABB-copy / contact-route, ~1 ms/step saved.)

### P8.4 — Mixed-backend regression suite (1 week)

- Test worlds that mix Newton + ODE Solids in the same scene.
- The bridge MUST still work for these (Newton dynamic ↔ ODE static).
- Existing 4v4 damage world becomes the canonical mixed-backend test.

**Exit criterion:** all existing Newton + ODE worlds continue to load
+ run headless 30s with 0 errors.

## 6 — Risks

| Risk | What could break | Mitigation |
|---|---|---|
| Newton's contact pipeline doesn't match ODE's for arbitrary geometry | Visual/physics divergence on legacy worlds | P8.2 exit criterion: damage-event-count parity on the existing 4v4 damage world. Forces direct comparison |
| Static-body count explodes (e.g. forest world with 1000 trees as separate Solids) | Newton broadphase O(N²) on first-step contact pair generation | Newton has a spatial hash for broadphase. P8.3 monitors ms/step; if it blows up on dense static scenes, fall back to ODE for static phase |
| URDF "fixed-joint decoration" semantics drift | OmniQuad's hip_motor / shoulder_motor links currently merge into the leg; if P8 registers them as separate Newton statics they become collidable, which may add unwanted self-collisions | P8.2 keeps the `isFixedChild` check intact for inside-robot statics; only top-level scene statics (arena floor/walls/obstacles) become Newton statics |
| Determinism: ODE's contact iteration order vs Newton's | Newton-canonical and bridged-backend produce different RL training observations | Acceptable: training is already non-deterministic across runs (RNG seeds, numerical precision); the goal is "stable enough that PPO converges," not bit-identical replays |

## 7 — Out of scope for P8

- **GPU broadphase for statics.** Static-body collision pair generation
  on the CPU is fine for typical 10-100 static Solids per world. GPU
  broadphase is Granular Tier 3's concern (§13.7), not P8's.
- **Static-collision-shape baking.** Newton's `ModelBuilder` already
  caches static shapes at finalize; no further work needed here.
- **Mesh collision for statics.** URDF static meshes (OmniQuad's terrain
  mesh, e.g.) fall back to AABB box approximation — same as the
  dynamic-mesh path today.

## 8 — Test plan

| Test | Pre-P8 status | Expected post-P8 behaviour |
|---|---|---|
| `newton_husky_smoke_test.omniworld` | works | works (no behavioural change; ground was already on Newton as `addGroundPlane`) |
| `newton_husky_head_on_damage.omniworld` (4v4) | works through bridge | works without bridge; damage events parity within rounding |
| `newton_husky_20v20.omniworld` | works through bridge | works without bridge; ms/step drops by ≥ 5% |
| `husky_head_on.omniworld` (ODE) | works (pure ODE) | works (unchanged; no Newton-backed Solids) |
| `husky_damage_arena.omniworld` (ODE) | works | works |
| Mixed (`mixed_husky_10.omniworld`: 9 Newton + 1 ODE) | works | works through bridge (the 1 ODE husky still needs proxy contacts against Newton statics, which is the bridge's job) |

## 9 — Estimated total effort

Total: **4-6 weeks** single-engineer.

- P8.1 API + smoke: 1 week
- P8.2 OmSolid dispatch: 1-2 weeks (the hard part; matching Newton
  contact behaviour against ODE for arbitrary boundingObject shapes)
- P8.3 Bridge skip: 1 week
- P8.4 Mixed-backend regression suite: 1 week
- Buffer for unknown integration issues: 1 week

This is the major architectural win in Phase β of the engine-migration
plan, and it's the largest single deferral remaining in the physics
arm before Phase D's canonical flip ships.

## 10 — Dependencies

- P5 in-OmniSim 20v20 ✅ (verified 2026-05-28 in commit `71b11f01`).
  P8 is the next major Newton-arm milestone after P5.
- Newton 1.2 stable release ✅ (installed + verified 2026-05-29; was
  1.2.0rc3 at session-start — husky trace bit-identical across the bump).
- P6 quantitative damage-suite parity harness (Phase β tool work) is
  the load-bearing test for P8.2's exit criterion.
