# Retiring ODE — the decision, the evidence campaign, and the work

> # ✅ STATUS: EXECUTED AND CLOSED — 2026-08-08
>
> **The ODE backend is gone.** `bdc02139` deleted `src/ode/` and `include/ode/` —
> **106,283 lines**. There is no second physics engine in OmniSim. Newton with
> `SolverMuJoCo` is the only backend, in a CPU (`mj_step`) and a batched-GPU
> (`mujoco_warp`) profile. `physicsBackend "ode"`, `defaultPhysicsBackend "ode"`,
> `OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY` and `OMNISIM_ALLOW_ODE_FALLBACK` no
> longer select a working backend. **There is no fallback backend** — the phrase
> "falls back to ODE", which appears throughout the document below and across this
> docs tree, described a machine that no longer exists.
>
> ### 🔴 BUT THE DELETION LEFT A SILENT NO-PHYSICS PATH — the exact failure class this campaign fought
>
> **Verified in source 2026-08-08, and it is the most important thing on this page.**
> "There is no ODE" does **not** mean "asking for ODE fails loudly". Two paths still
> resolve to the inert `OmOdeBackend` stub and run **with no physics at all, silently**:
>
> 1. **A world that still pins `physicsBackend "ode"` / `defaultPhysicsBackend "ode"`
>    LOADS FINE.** `OmSolid::effectivePhysicsBackendName` treats an explicit `"ode"` as a
>    choice that *"always wins"*, so the Solid is handed the stub. No FATAL, no ERROR, no
>    warning — the body simply never moves and nothing ever collides. ⚠ Refusing these at
>    construction **was tried and reverted** (it wrongly refused 4 of 4
>    `defaultPhysicsBackend "ode"` worlds), so this is a known, deliberate state, not an
>    oversight. ~270 worlds carried `"ode"` pins before `9db0e162` dropped the habitual
>    ones; whatever remains is in this state.
> 2. **A clone with the Newton runtime ABSENT does the same.**
>    `OmPhysicsBackendRegistry::newtonEnforced()` returns false when no Newton runtime came
>    up, and the code's own comment says it *"no longer keeps running ODE, unaffected,
>    because there is no ODE — it runs on the inert `OmOdeBackend` stub, i.e. with no
>    physics"*, caught **only** if the operator sets `OMNISIM_REQUIRE_NEWTON`. Making it
>    loud by default is recorded as an owner decision that has not been taken.
>
> **So distinguish two cases, because only one of them is loud.** A Newton runtime that is
> *installed but would not come up* is refused with a FATAL (the 2026-08-05 work,
> `85fa6bde`). A runtime that is simply *absent* is not. The campaign's own rule — **"a
> wrong result is worse than a lost one"**, and reachable stubs must warn — is therefore
> **not yet satisfied by the end state**. Until it is, `OMNISIM_REQUIRE_NEWTON=1` is
> mandatory in any deploy, CI or benchmark run whose result you intend to trust, and a
> `.newton.json` sidecar assertion is the only honest proof Newton drove a world.
>
> ### ⚠ Be precise about what "deleted" means — the second half is unexecuted
>
> `bdc02139` deleted the **vendored ODE library**. It did **not** delete the
> engine-side ODE integration shells, and a reader who assumes otherwise will be
> confused by the tree:
>
> - `src/omnisim/ode/` still exists in full — `OmOdeContext`, `OmOdeDebugger`,
>   `OmOdeUtilities`, `OmOdeContact.hpp`, `OmOdeGeomData.hpp`, `OmOdeTypes.hpp` —
>   as **stubs**. `OmOdeContext` constructs null handles around a mutex that now
>   guards nothing.
> - `src/omnisim/physics/OmOdeBackend.{cpp,hpp}` still exists, likewise stubbed.
> - **~23 `#ifdef OMNISIM_WITH_ODE` sites remain in `src/`**, permanently
>   compiling their `#else` branch.
> - `OMNISIM_WITH_ODE ?= OFF` **still exists in both Makefiles** — but only to
>   `$(error)` on a stale `=ON` invocation, with a sentence instead of a thousand
>   link errors. It is not a build choice; never offer it as one.
> - `OmOdeTypes.hpp` survives deliberately, as vestigial plain typedefs: ~44
>   headers carry those handle types through the `OmBodyHandle` /
>   `OmJointHandle` dispatcher pattern, and renaming them is pure code motion
>   with review risk and zero behaviour change. Optional hygiene, never a
>   deletion blocker.
>
> So **collapsing those ifdefs, deleting the stubbed module and retiring the flag
> is real remaining work, not bookkeeping.** None of it can simulate anything —
> the *backend* is unambiguously gone — but a grep for `WbOde*` or
> `OMNISIM_WITH_ODE` still returns hits, and anyone reading those hits as live
> code will draw the wrong conclusion.
>
> **Everything below this banner is the campaign record, written while ODE still
> shipped.** It is preserved deliberately: it holds the measurements, the
> rationale and the six-blocker analysis that justified the work. Read it as
> history. Its *instructions* are dead, its *plans* were either executed or
> overtaken, and §9's recommendation was **overruled by the owner** (see below).
> Where a section is still live, it carries its own banner.
>
> ### What actually happened
>
> The document opens by arguing that "retire ODE" names two projects differing by
> ~34× — solver retirement (310 lines behind an existing seam) and kernel removal
> (~10,663 lines behind no seam at all) — and that the tree had never chosen. **The
> owner chose kernel removal**, and it was engineered, not deferred. All six
> blockers closed:
>
> | # | blocker (§0) | outcome | commits |
> |---|---|---|---|
> | 1 | Raycasting | **ported native + default-ON**; all five consumers answer from `mj_ray` | `4040805f`, `cb73a90e`, `9240be24` |
> | 2 | Fixed / weld joints | **native** (MuJoCo equality weld), default-ON | `fea7bc4d`, `6eb35675` |
> | 3 | Buoyancy (`Fluid`) | **DELETED with the feature** — not ported | `f0574cbe` |
> | 4 | Kinematic collision | **native** (MuJoCo mocap bodies), default-ON | `31d75482`, `6eb35675` |
> | 5 | Force-type `TouchSensor` | **native** (`cfrc_int` readback), default-ON — ⚠ but see the defect list | `fea7bc4d`, `6eb35675` |
> | 6 | Physics-plugin C ABI | **DELETED, not replaced** — a deliberate breaking change | `a0ed801c`, `e50d45fd` |
>
> And the recursion §0 flags — Newton reading ODE's `dMass` for its own inertia
> tensors — was broken by an ODE-free transcription of the mass integrator
> (`fe35be64`), including a Mirtich port for meshes. Supporting work: the solver
> default flipped to MuJoCo (`7b431e81`), XPBD was then **removed entirely**
> (`94f04222`), bumpers went native (`36b37510`), the world corpus dropped its
> habitual ODE pins (`9db0e162`), `OMNISIM_WITH_ODE=OFF` was built and booted
> (`3e30485b`, `0db3a274`), and `WorldInfo.coordinateSystem` finally reached the
> solver (`c77cbe98`).
>
> **Flagship gate on the ODE-free binary:** G1 `box_delivery` `ok=1 segs=14/14
> minz=0.720 dur=155.1` — the balance-harness walk still works with no ODE in the
> process. (Harness disclosure applies as always: weight-bearing rig, not a
> free-standing walk.)
>
> ### ⚠ The one lesson worth carrying out of this campaign
>
> **A load-only sweep cannot see wrong physics, and this campaign has the
> cleanest proof of it anyone is likely to get.** §4.5 below scores `tests/api`
> at **140/140 PASS**. All 140 of those worlds are `NUE` (Y-up). Newton was
> being built `up_axis=Z` regardless, so gravity — projected onto the up vector —
> came out **exactly zero**, and the implicit ground plane stood up as a vertical
> wall. *Nothing in any of those 140 worlds ever fell.* A ball released at y=3
> read y=3.000 at step 15360. They loaded, they stepped, they logged nothing bad,
> and they scored a perfect 140/140 while simulating a world with no gravity.
>
> Post-fix, the honest `tests/api` score is **128/139**. That is the number to
> quote. Any doc in this tree leaning on a load-only PASS as *physics* evidence
> is making the same mistake — `run-headless --fail-on-runaway` exists precisely
> because a bare PASS is a log verdict, never a physics verdict.
>
> ### ⚠ Known broken on the ODE-free binary — recorded, not hidden
>
> These are open defects, not campaign residue. Each was found by the deletion:
>
> - **Motorised `BallJoint` and `Hinge2Joint` do not actuate.** Registration and
>   motor enrollment are verified wired; the defect is in the runtime **d6 joint
>   build at finalize**. The gate default is **OFF**. Population is small and
>   pedagogical (§7 Tier-1 item 5 sized it at 20 worlds, 16 in `tests/`, no demo,
>   flagship, policy or benchmark world) — but "deprioritised" is no longer the
>   right word now that ODE cannot serve them.
> - **Force-type `TouchSensor` can read 0 N with no force source at all.** Read the
>   shape of this precisely (source-verified 2026-08-08), because "force sensors
>   are broken" is too coarse: `OMNISIM_NEWTON_TOUCH_FORCE` is **value-parsed and
>   DEFAULT ON**, and a sensor that *was* un-folded into its own Newton body does
>   answer from the `cfrc_int` mount wrench. A sensor that was **not** un-folded has
>   **no force source whatsoever** — the ODE mount-joint feedback it used to fall
>   back to is gone — and `mValues[0]` is a lookup of `0.0`. To its credit the
>   engine now warns once, naming the structural reason and saying *"do not read
>   this 0 as a measured force."* This is an OFF-mode regression that survived
>   review *because ODE was still compiled in and silently answering*.
>   ⚠ **Source doc-drift, reported not fixed:** the comment at
>   `OmTouchSensor.cpp:34` still says `DEFAULT OFF` while the code twelve lines
>   below returns `true` when unset. The code is the truth; the header comment was
>   not updated when the default flipped.
> - **Contact sound and the contact-points GUI overlay produce nothing.** Both
>   read `odeContacts()`. These have been dead since Newton became the default and
>   **predate the deletion** — the deletion only made them undeniable.
> - **The smoke lane now asserts NO DYNAMICS ANYWHERE** — and that is the honest
>   summary, stated in `tests/smoke/run_smoke.py` itself. Of the three worlds that
>   used to be ODE-pinned:
>   - `template_deterministic` **genuinely PASSES on Newton** (the DistanceSensor
>     raycast default going ON in `6eb35675` fixed it — its four sensors had been
>     reading a constant, which collapsed the PROTO-determinism verdict). This is
>     the world whose sensors sat exactly where the wrong-axis ground plane stood
>     up as a wall.
>   - `accelerometer` is **skipped, red**. Its `skip_reason` blames Newton
>     hardcoding `up_axis=Z` and never reading `WorldInfo.coordinateSystem`.
>     ⚠ **That reason is inconsistent with `c77cbe98`, which plumbed exactly that
>     field** — so either the fix does not reach the accelerometer path or the
>     reason is stale. Worth re-measuring; it is written as a re-measurement dated
>     2026-08-08, after the fix.
>   - `contact_points` is **skipped**, and its note is the second measured
>     instance of this campaign's central lesson (see below).
>
>   The replacement physics assertion has not landed: the candidates checked
>   (`damping`, `hinge_joint_damping`, `rolling_friction`,
>   `floating_point_precision`) all fail for an unrelated reason — their C
>   controllers are not built in this clone, which surfaces only as a 30 s
>   *"results file has not been written"* timeout.
>
> - ✅ **`run_smoke.py` no longer forces ODE** — the campaign's own to-do list
>   flagged this as an open gap and it is closed. It now runs **one group on the
>   engine default**, and it **scrubs `OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY` from
>   the environment with a loud warning**, so a stale export in a developer's shell
>   cannot decide the lane's verdict.
>
> ### 🔴 A SECOND, INDEPENDENT MEASUREMENT OF "A GREEN VERDICT THAT CERTIFIES NOTHING"
>
> The `coordinateSystem` case (140/140 with zero gravity) is not a one-off. Measured
> 2026-08-08 on `tests/physics/worlds/contact_points.omniworld`, machine
> machine `9722d23d12a3` (RTX 3060 laptop), via the in-binary `OMNISIM_PROBE_TRAJ` probe
> over 3000 ms:
>
> - **Default backend** (sidecar `{backend: newton, solver: "MuJoCo (cpu/mj_step,
>   default)", finalised: true}`): the Cone rolls down the ramp and lands —
>   `-5.951940 -4.392650 1.060230` → `-5.976457 -5.292823 0.007408`. The test
>   **FAILS** (its `-rotation[3] > 2.44` verdict is ODE-contact-tuned).
> - **Under `OMNISIM_FORCE_ODE=1`**: no `.newton.json` sidecar is written and the Cone
>   reads `-5.951940 -4.392650 1.060230` at t=3000 ms — **bit-identical to t=0, zero
>   motion in three seconds.** The test **PASSES**, purely because the *authored*
>   rotation (`-2.4871`) already satisfies the threshold before anything happens.
>
> **The ODE pin turned a red physics test green by freezing the scene.** This is the
> silent-no-physics path in the top banner, caught in the act, and it is why the
> pinned group was deleted rather than retargeted. The rule it earns, in the smoke
> lane's own words: **"a green verdict from a frozen scene is worse than a red
> one."** A world that cannot pass on the default backend is marked `skip: true`
> with a measured reason naming the engine-side gap — never coaxed green with a pin
> or a retuned golden.
>
> ### Companion docs — status after the campaign
>
> All four are now **historical**; none should be followed as a plan:
> [`v6-readiness.md`](v6-readiness.md),
> [`newton-ode-replacement-plan.md`](newton-ode-replacement-plan.md) (W0–W7),
> [`default-flip-plan.md`](default-flip-plan.md) (the safety harness for a flip
> that has since happened *and* been superseded by deletion),
> [`../benchmarks/step-cost-2026-08-06.md`](../benchmarks/step-cost-2026-08-06.md)
> (its ODE arm can never be re-run).
>
> **The second IN-ENGINE arm is gone with the backend — the oracle is not.**
> ⚠️ Lane 1's oracle is **analytic ground truth** and it is unaffected; bare MuJoCo
> and PyBullet still run as external arms. OmniBench lane 1 used ODE as its second
> *integration* and its generator emitted an `ode_pin`, so what is gone is the
> ability to diff one `.wbt` through two integrations — which is how every Newton
> integration defect here was found. And note the result that gets misquoted: ODE
> outscored `omnisim-newton`, not MuJoCo; bare MuJoCo scored fine on the same
> scenes and the four defects were ours and are fixed ([correctness-scope.md](../benchmarks/correctness-scope.md)). The frozen reference
> values survive in [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json)
> and that file is now the only ODE artefact in the tree. See
> [`../benchmarks/lane1-validity-2026-08-07.md`](../benchmarks/lane1-validity-2026-08-07.md)
> for what that lane was and was not measuring.

---

**Original status line, preserved:** *a plan, not a decision.* Written 2026-08-07
from a seven-agent read of the tree at `8e5f022f`. Every claim below cites a file
or a measurement. Where something is unknown this says so rather than estimating.

---

## 0. The finding that shapes everything

**"Retire ODE" names two projects that differ by ~34× in size, and the tree has
never chosen between them.**

| | solver retirement | kernel removal |
|---|---|---|
| what it is | `OmOdeBackend.cpp` — **310 lines, 46 call sites**, behind the existing `OmPhysicsBackend` abstraction | the collision / ray-casting / sensing kernel — **~10,663 lines across 22 files**, behind **no abstraction at all** |
| the seam | exists | **does not exist**: `OmPhysicsBackend.hpp` is 341 lines of body/joint verbs with *zero* geom, space, ray, collide or contact API |
| ODE stays in tree? | yes (~106k lines) | no |
| blockers | none structural | **six, with no Newton primitive to port onto** |
| size | weeks | different order of magnitude |

The six kernel blockers, each with no Newton equivalent today:

1. **Raycasting** — `DistanceSensor` (`OmDistanceSensor.cpp:433,454,547`), camera
   recognition + Radar occlusion (`OmObjectDetection.cpp:70-114`), `LightSensor`
   (`OmLightSensor.cpp:64-100`), `Receiver` line-of-sight (`OmReceiver.cpp:50-79`).
   `OmNewtonBackend` has **no raycast API whatsoever**.
2. **Fixed / weld joints** — `Connector` (`OmConnector.cpp:459-583`),
   `VacuumGripper` (`OmVacuumGripper.cpp:129-188`). Newton has revolute /
   prismatic / hinge2 / ball; no weld.
3. **Buoyancy** — `Fluid` / `ImmersionProperties`, implemented in a **fork-only**
   ODE subsystem (`src/ode/ode/src/fluid_dynamics/`, 19 files, 4,688 lines) that
   is not even upstream ODE.
4. **Kinematic collision** — physics-less robots (`OmSimulationCluster.cpp:162-192`).
   By definition these bodies never enter a solver.
5. **Force-type TouchSensor** — `dJointFeedback` (`OmTouchSensor.cpp:241-265`).
6. **The physics-plugin C ABI** — `include/plugins/physics.h:19` is
   `#include <ode/ode.h>`. `webots_physics_collide(dGeomID, dGeomID)` *is* a
   narrow-phase callback. This is a documented public API with ~10 reference
   pages; it cannot be reimplemented on Newton because its type vocabulary is
   ODE's. Any replacement is a **new** API and a breaking-change event.

And one recursion that has to be broken either way: **Newton reads ODE's mass
integrator for its own inertia tensors** — `OmSolid.cpp:3392-3396` feeds
`s->odeMass()->I` straight into `newton->addBody(...)`.

> ✅ **Blocker 1 (raycast) COMPLETE AND DEFAULT-ON 2026-08-07** — service
> `4040805f`, consumers 2–4 `cb73a90e`, default flip `9240be24` (with the
> shared joint-descending exclusion walk + the LASER transparency re-cast).
> All five consumers answer from `mj_ray` on the live mjModel;
> `OMNISIM_NEWTON_RAYCAST=0` reverts; INFRA_RED stays on the ODE pass
> structurally (needs the collided object's color). Corpus: tests/api sweep
> effectively 140/140 on the flipped default (134 + 6 FFI-flake retries),
> above the 132/140 baseline.
> ⚠ **That corpus figure is the poisoned one** — all 140 `tests/api` worlds are
> `NUE` and were running with gravity at exactly zero at the time (see the
> `coordinateSystem` banner below). It evidences that the raycast flip **broke
> nothing**, which is what it was for; it is *not* evidence that the rays are
> physically correct. The sensor parity tests are that evidence.
>
> ✅ **Blocker 2 (welds) LANDED 2026-08-07 (`fea7bc4d`)** — MuJoCo equality
> weld slot per Connector/VacuumGripper, engage/release/rupture readback
> from solved efc rows; `OMNISIM_NEWTON_WELDS` (value-parsed, default OFF —
> flip after soak). Two probed corrections vs the design: eq_data anchor
> encoding (zero-anchor lever-arms) and pre-refresh snapshot readback (the
> end-of-tick mj_step1 refresh rebuilds efc unsolved).
>
> ✅ **Blocker 4 (kinematic) LANDED 2026-08-07 (`31d75482`)** — physics-less
> movers register as MuJoCo mocap bodies (statics already are; pose writes
> go directly to `mj_data.mocap_pos/mocap_quat`, wxyz);
> `OMNISIM_NEWTON_KINEMATIC` (value-parsed, default OFF — flip after soak).
>
> ✅ **Blocker 5 (force TouchSensor) LANDED 2026-08-07 (`fea7bc4d`)** —
> un-fold into an own body + fixed joint, negated `cfrc_int` readback (981 N
> mount semantics, not the 1962 N contact sum); `OMNISIM_NEWTON_TOUCH_FORCE`
> (value-parsed, default OFF — flip after soak).
>
> Remaining blockers: **3 (Fluid/buoyancy)** — dies with the feature per the
> owner decision — and **6 (physics-plugin C ABI)** — deleted, not ported.
>
> ✅ **`OMNISIM_WITH_ODE=OFF` WORKS 2026-08-08** — skeleton `3e30485b`
> (flag, matrix guard, closed-core stubs), long tail `0db3a274` (~39 TUs,
> insertion-only): the OFF configuration **compiles, links and boots**
> (empty.wbt clean log, Newton runtime up); OFF+NEWTON=OFF is refused at
> make time. ⚠ The flag does not invalidate objects and `make clean`
> no-ops in the standard invocation — purge `build/release` `*.o` when
> switching flavors. Execution-ready deletion manifests (plugin ABI,
> Fluid, src/ode) live in `_scratch/deletion_manifests.md`, including the
> pre-deletion gaps they surfaced: BUMPER TouchSensors and contact
> sound/GUI have no native contact feed yet, Hinge2/Ball joints have no
> Newton registration (frozen under OFF), and `run_smoke.py` still forces
> ODE (the pre-push hook needs a Newton re-golden). Two banner
> corrections from the same audit: INFRA_RED DistanceSensors and
> OmGranularGroup are ODE-free already.
>
> **Where those four pre-deletion gaps ended up:** BUMPER TouchSensors went
> **native** (`36b37510`); contact sound and the contact-points GUI overlay did
> **not** — they still read `odeContacts()` and now produce nothing; Hinge2/Ball
> registration is wired but the joints **do not actuate** (runtime d6 build at
> finalize, gate default OFF); and `run_smoke.py` **no longer forces ODE** — it runs
> one group on the engine default and *scrubs* `OMNISIM_FORCE_ODE` /
> `OMNISIM_LEGACY` from the environment with a warning. ⚠ Its re-golden only partly
> happened, though: `accelerometer` and `contact_points` are skipped with measured
> reasons, so **the smoke lane now asserts no dynamics at all.** Details in the
> defect list in the banner at the top of this file.
>
> Remaining engineering: soak-flip `OMNISIM_NEWTON_WELDS` /
> `OMNISIM_NEWTON_TOUCH_FORCE` / `OMNISIM_NEWTON_KINEMATIC`, close the
> native-feed gaps above, smoke re-golden, execute manifests 1+2, soak
> `OMNISIM_WITH_ODE ?= OFF` as the default, then delete `src/ode/`.
>
> ✅ **ALL OF THAT IS DONE.** The three gates flipped ON together with the
> bumper feed (`6eb35675`, `36b37510`), the manifests executed (`a0ed801c`
> plugin ABI, `f0574cbe` Fluid, `e50d45fd` plugin test trees), and
> `src/ode/` + `include/ode/` were deleted in `bdc02139`. `OMNISIM_WITH_ODE`
> no longer exists as a choice — there is nothing for it to switch on. The
> gaps this paragraph called "native-feed gaps" did **not** all close:
> contact sound and the contact-points GUI still read `odeContacts()` and
> now produce nothing, and Hinge2/Ball motorisation is broken rather than
> frozen. See the defect list in the banner at the top of this file.
>
> ✅ **`WorldInfo.coordinateSystem` REACHES NEWTON 2026-08-08 — the last thing
> ODE was masking for a THIRD of the corpus.** `OmNewtonBackend.cpp` built
> `newton.ModelBuilder(up_axis=newton.Axis.Z)` and the string `coordinateSystem`
> appeared **zero** times in the file, so all **210 `NUE` (Y-up) worlds — 29% of
> 719, and not one of them pinning a backend** — got both halves of their world
> model wrong: `set_gravity()` projects `WorldInfo`'s gravity vector onto
> `builder.up_vector`, and `(0,−g,0)·(0,0,1)` is **exactly 0**, so nothing fell
> (measured: ball released at y=3 reads **y=3.000 at step 15360**), while
> `add_ground_plane()` took its **normal** from the same axis and stood the
> implicit floor up as a **vertical wall** along the world's East axis (the ball
> drifted to **z=+384 m**; in `tests/protos/worlds/template_deterministic.omniworld` the
> wall sits exactly where its four DistanceSensors are and all four read
> 0.000000). ⚠️ **This is the sharpest example in the campaign of a log verdict
> certifying nothing**: §4.5 above scores `tests/api` **140/140 PASS**, and all
> 140 of those worlds are `NUE`. They loaded, they stepped, they logged nothing.
> Fix: the field is plumbed to `up_axis` by `applyCoordinateSystemToWorld()` from
> inside `ensureWorldOpen()`, **after `beginWorld()` and before
> `addGroundPlane()`** — the only window, since newton bakes the plane normal at
> add time. Verified against the bundled runtime rather than assumed: `up_axis` is
> a plain mutable `ModelBuilder` attribute, `Model.up_axis` is read by **no
> solver** (only viewer + importers), the MuJoCo converter passes the plane's full
> pos+quat and `Model.gravity` as an explicit 3-vector, and newton's own
> `newton/tests/test_physics_validation.py` runs Y-up builders with a real ground
> plane through `SolverMuJoCo` on both CPU and mujoco_warp. Gate
> `OMNISIM_NEWTON_COORD_SYSTEM` (value-parsed, **DEFAULT ON** — a bug fix, not a
> feature; `=0` pins the historical Z for bisection and warns). `ENU` is a literal
> no-op (`Axis.Z` over `Axis.Z`), pinned as such. Tests:
> `tests/test_newton_coordinate_system.py` (5 cases: ENU vs golden, NUE analytic,
> the two arms agreeing to 1 mm, and both directions of the gate).
>
> ✅ **The inertia recursion is BROKEN 2026-08-07 (`fe35be64`)** — `OmInertia`
> + `OmSolidUtilities::addInertia` (ODE-free transcription of mass.cpp incl.
> a Mirtich port for meshes) now feeds the Newton body build;
> `OMNISIM_NEWTON_NATIVE_INERTIA=0` reverts while ODE ships. Parity vs the
> dMass oracle: primitives bitwise, rotated compounds 1.4e-16, trimesh
> 4.8e-8 (`tests/test_newton_native_inertia_parity.py`); flagship gates green
> incl. G1 box_delivery `ok=1 segs=14/14 minz=0.720`. Known left-behinds, on
> purpose: `rolledUpComInertia`'s zero-tensor fixed-child fallback and the
> `OmSolidMerger`/mass-tab/`OmPose` dMass consumers die *with* the ODE body
> pipeline, not before. Two aspirational red tests sit untracked in `tests/`
> (`test_newton_friction_cone_default.py`, `test_newton_servo_without_limits.py`)
> — binary-swap A/B proves both pre-date the inertia change; they pin future
> default flips (elliptic cone + impratio, limitless-servo tracking), not
> regressions.

There is also no build switch. `Makefile:171-174` builds `src/ode` in every
target; the only flag is `OMNISIM_WITH_NEWTON=OFF`. The tree can build ODE-only
and cannot build Newton-only.

> ✅ **Resolved, then made moot.** `OMNISIM_WITH_ODE=OFF` was added (`3e30485b`
> skeleton, `0db3a274` long tail — ~39 TUs, insertion-only) so the switch existed
> before the deletion depended on it. After `bdc02139` there is nothing to
> switch: the tree builds Newton-only and *cannot* build ODE at all. The
> inversion is complete.

---

## 0.5 The finding that reorders this whole document: the default solver is wrong

> ✅ **DONE — the flip LANDED 2026-08-07 (`7b431e81`), gates green.**
> `""`/`"auto"` now resolve to SolverMuJoCo (CPU `mj_step`); `"xpbd"` and
> `OMNISIM_NEWTON_FORCE_XPBD=1` (value-parsed) are the opt-outs — ⚠ **and both were
> then REMOVED the same day (`94f04222`): XPBD is gone, so there is no opt-out and
> `newtonSolver "xpbd"` selects nothing.** OmniSim runs one solver; unpinned
> worlds' sidecars read `MuJoCo (cpu/mj_step, default)`. Verified on the
> flipped binary: lane-1 T3/T6/T7 exact values identical, the Newton test
> battery (fallback ×5, wrench ×2, readbacks ×3, pins ×3, freshness ×3), G1
> conformance 16/16, smoke suite, and the 10-Husky swarm driving 2.385 m
> unpinned. The section below is the decision record.

**Almost everything bad we "know about Newton" is about XPBD. Almost everything
good is about MuJoCo. And nobody has ever chosen XPBD.**

Verified against the Newton runtime bundled in this tree — the exact version the
engine loads, which is stronger evidence than the published docs:

- **`newton/solvers.py:128-132`, verbatim:** *"Only `SolverFeatherstone` and
  `SolverMuJoCo` operate on articulations (generalized/reduced coordinates). The
  maximal-coordinate solvers (`SolverSemiImplicit`, **`SolverXPBD`**, and
  `SolverKamino`) enforce joints as pairwise body constraints but do not use the
  articulation kinematic-tree structure."* OmniSim's default solver is one its
  own vendor documents as **not operating on articulations** — in a simulator
  whose entire content is articulated robots.
- **All 9 of Newton's `examples/robot/` use `SolverMuJoCo`. Not one uses XPBD**
  (allegro hand, ANYmal C/D, cartpole, G1, H1, panda, policy, UR10). XPBD appears
  only in `basic/`, `cloth/` and two stacking demos.
- **Joint feature matrix, XPBD vs MuJoCo:** `joint_armature` no/yes,
  `joint_effort_limit` no/yes, plus `joint_friction`, `joint_limit_ke/kd`,
  `joint_target_mode`, equality (CONNECT/WELD/JOINT) and mimic constraints — all
  absent on XPBD. Armature and effort limits are not niceties for robotics.
- **`SolverXPBD(..., enable_restitution: bool = False)`** — and `restitution` /
  `bounce` appear nowhere in `OmNewtonBackend.cpp`. **Every default world has zero
  coefficient of restitution.**
- **The friction law is self-described as approximate** —
  `xpbd/kernels.py:2252`: *"limit friction based on incremental normal force,
  **good approximation** to limiting on total force."* An approximation to
  Coulomb is exactly a creep, which is what the pinch-grasp failure is.

And the configuration is wrong on top of the choice. Newton's own XPBD example
runs `fps=100`, `sim_substeps=10` → a **1 ms** substep. Our default is
`newtonSubsteps 1` at a 16–32 ms `basicTimeStep` → a **16–32 ms** substep, i.e.
**16–32× the vendor's reference**, in exactly the regime Macklin's *Small Steps in
Physics Simulation* (SCA'19) exists to warn against. **Zero of the 519 worlds on
the default set `newtonSubsteps`.**

**Nobody chose this.** Of 725 tracked worlds, **198 declare `newtonSolver` and
every single one names a MuJoCo variant. Zero declare `"xpbd"`.** All 156 lane-1
correctness worlds pin `"mujoco"`; every Newton regression baseline was measured
on it; 141 of 155 policy worlds pin it; there is a lint
(`scripts/dev/check_deploy_solver.py`) whose entire reason for existing is that a
deploy world loaded without its run-script *"silently falls to XPBD"*. **There is
no correctness evidence for XPBD anywhere in this tree.** Yet 527 worlds take the
default — including all 14 OmniLink chat demos, i.e. the beginner surface.

**What a flip to `"mujoco"` (CPU) would dissolve:** the multi-wheel lateral lock
(documented as structural and parameter-unfixable — 1 wheel drives 6 m, a lateral
pair 0.2 m; MuJoCo and ODE drive every case), the pinch-grasp failure, joint
armature / effort limits / friction / target mode, equality and mimic
constraints, motorised ball joints, the NaN-at-closing-speed class, and probably
the inert `joint_f`. Determinism *improves*: CPU `mj_step` is bitwise at 336
contacts with ten live controllers, where XPBD is proven on one 1.6 s sphere drop.

**Is XPBD load-bearing? No.** The CUDA granular path couples to **ODE**, not
Newton (`OmGranularGroup.cpp:272,991` — it reads ODE velocities and pushes back
via `dBodyAddForceAtPos`). No cloth or soft-body nodes exist. XPBD's real
strengths in Newton — particles, cloth, soft bodies — are capabilities OmniSim
does not use.

**Two gates before flipping — ✅ BOTH ANSWERED 2026-08-07, and the flip case got
stronger:**

1. **The XPBD-vs-`mujoco`-CPU step-cost A/B — RUN, and it inverts the premise.**
   The schema comment calls XPBD "the default perf path"; measured, **CPU mujoco
   is FASTER than the XPBD default at every scale tried**, interleaved
   pair-by-pair on one machine in one session (`step_cost --solver`, RTX 3060
   laptop):

   | scene | XPBD (shipped default) | mujoco CPU | ratio |
   |---|---|---|---|
   | 5 boxes resting | 3.82 ms/step | 0.44 | **0.11×** |
   | 20 boxes | 3.94 | 1.10 | **0.28×** |
   | 100 boxes | 5.24 | 3.31 | **0.63×** |
   | `newton_husky_swarm_drive.omniworld` (10 Huskies, shipped, unpinned) | 8.16 | 5.46 | **0.67×** |

   All pairs agree in sign at every size. There is no perf cost to the flip;
   there is a perf *win*. (ODE reads 2.31 ms/step on the Husky world — still the
   fastest, consistent with everything above.)

   **And the same run answered capability with physical units.** On the shipped
   10-Husky world, planar displacement over 6 s of sim, identical commands:
   ODE **2.402 m** · mujoco **2.385 m** (matches ODE within 1%) · XPBD
   **0.968 m max, 0.906 median** — the robots move at **~40% of the speed the
   other two solvers agree on**. That is the lateral wheel-pair defect measured
   on a shipped world: not "slower to compute", but *wrong*, by 2.5×, on the
   demo surface. One caveat: n=1 session, one machine, and one XPBD launch also
   hit the intermittent FFI bring-up fault mid-experiment (correctly refused by
   the `85fa6bde` machinery — the refusal path works under fire).

2. **`newtonNjmax`/`newtonNconmax` — MOOT for the CPU target.** Per
   [`worldinfo.md:181`](../reference/worldinfo.md): *"The CPU `newtonSolver
   "mujoco"` path is not watched and does not need to be — **MuJoCo-C grows its
   `efc` arrays dynamically, so there is no cap to overflow**."* The 256-row cap
   is a `mujoco_warp` concern only; a flip to `"mujoco"` (CPU) does not touch it.
   (The cap-scaling work is still worth doing for the 14 `mujoco_warp` worlds —
   just not as a gate on this flip.)

What remains before the flip ships: the physics re-baseline itself — lane-1
T3/T6/T7 exact values under the new default, the 27 many-robot worlds
spot-checked, and the smoke suite — plus landing the value-parsing escape
hatches (F2) so `newtonSolver "xpbd"` remains an exact opt-out revert.

> ✅ **The flip shipped (`7b431e81`) with that re-baseline done, and then the escape
> hatch was removed rather than kept:** XPBD was deleted the same day (`94f04222`).
> `newtonSolver "xpbd"` and `OMNISIM_NEWTON_FORCE_XPBD` select nothing. This is an
> instance of a general rule the campaign learned — **a value-parsed revert hatch is
> only honest while the thing it reverts *to* still exists.** Retire the hatch in the
> same release notes as the substrate it reverted to.

⚠ Target **`"mujoco"` (CPU)**, not `"mujoco_warp"`: at nworld=1 warp measured
9.06× slower than CPU `mj_step` and is not run-to-run reproducible.

This is plausibly higher-value than every flip in §6 combined, and it should be
coordinated with the five uncommitted red tests rather than landed across them.
It does **not** buy correctness supremacy — Newton-under-MuJoCo still wins only
4 of 11 lane-1 metrics, and per
[`lane1-validity`](../benchmarks/lane1-validity-2026-08-07.md) that score is
itself largely mis-measured. It buys **capability and honesty**.

### What the XPBD papers say about themselves

The authors are consistent, across a decade, that this is a graphics method:

- **PBD (Müller et al. 2007):** *"In contrast to computational sciences where the
  main focus is on accuracy, the main issues here are stability, robustness and
  speed while the results should remain **visually plausible**… existing methods
  from computational sciences can not be adopted one to one. **The method we
  present falls into this category.**"*
- **XPBD (Macklin et al., MIG 2016)** — the acceptance criterion in its own
  abstract is *"visually similar results"*; CCS concepts are *"Real-time
  simulation; Interactive simulation"*. Its limitations say *"traditional
  methods may be more suitable for applications requiring **greater accuracy
  guarantees**"* and *"low iteration counts that terminate before convergence
  will result in **artificial compliance**."*
- **⚠ And the compliance machinery does not cover contacts at all.** §6, verbatim:
  *"**We assume zero compliance in contact**, meaning it is not necessary to store
  the Lagrange multiplier for the contact constraint."* So XPBD's headline
  achievement — iteration-independent stiffness — **excludes the one thing
  robotics cares about**. Contacts stay plain PBD projections, i.e.
  iteration-count dependent. Read that against Newton's `SolverXPBD` default of
  **`iterations = 2`**.
- **Flex (Macklin et al. 2014)**, on the family's friction: classic PBD friction
  *"**cannot model static friction**… piles quickly collapse"*, and even their
  improved position-level model has *"friction strength … **somewhat dependent on
  iteration count**"* — i.e. effective μ is a function of your solver budget.
- **Small Steps (Macklin et al. 2019)**, same authors: *"Iterative implicit
  methods like XPBD are stable, and efficient for moderate stiffness, however
  they **struggle to achieve high stiffness**, and also suffer from numerical
  damping."*
- **Detailed Rigid Body Simulation with XPBD (Müller et al., SCA 2020)** — the
  rigid-body formulation Newton implements. First sentence: *"Rigid body
  simulation lays at the heart of every **game engine** and plays a major role in
  computer generated **special effects in movies**."* The word "robotic" appears
  **zero** times in it; the four "robot" mentions are an inverse-kinematics demo
  and a row in a timing table. Its limitations: *"Updating constraint directions
  after each projection might cause **instabilities when simulating tall stacks
  or piles of objects**"* — the warehouse/bin-picking regime — and *"for small
  time steps, **double precision** floating point numbers are required"*, which is
  hostile to GPU batch training.

Corroborating structure: **PhysX 5 uses PBD as its *particle* solver** (fluids,
granular, cloth) with rigid bodies and articulations as separate subsystems — so
NVIDIA does not use PBD for articulated rigid bodies in its own flagship engine
either. Newton's docs call out **Kamino**, not XPBD, for *"hard frictional
contacts"*; Newton's grasping guidance exists **only** on the MuJoCo page; and
Newton's own XPBD unit tests set static-equilibrium tolerances that **grow with
contact count** — sphere `rtol=0.05`, box `rtol=0.10`, 3-cube pyramid
`rtol=0.15`. Le Lidec et al.'s T-RO 2024 contact-model survey — the field's
canonical reference — does not mention PBD or XPBD **at all** (0 occurrences).

### ⚠ The honest limit of this case

**Nobody has published a head-to-head of the SCA-2020 XPBD rigid-body solver
against MuJoCo, Drake or Bullet on robotics contact benchmarks.** This is
convergent circumstantial evidence from primary sources — the authors' own stated
domain and limitations, the vendor's own feature matrix and example set, and our
own measured wheel-lock and pinch failures — **not a measured refutation**. The one
analytic PBD-vs-MuJoCo study (Abderezaei et al., arXiv 2603.14634, 2026) benchmarks
the **particle/shape-matching** PBD lineage, *not* this solver; do not transfer its
numbers. And MuJoCo is itself an admitted approximation: it relaxes the Signorini
condition, and Todorov's own words are that *"the complementarity formulation is not
necessarily a gold standard. **All contact models presently used in rigid-body
simulations are phenomenological.**"*

So the argument for the flip is **fitness for purpose and capability**, which is
well-evidenced, not *"XPBD is wrong"*, which is not established.

---

> ⚠ **READ [`lane1-validity-2026-08-07.md`](../benchmarks/lane1-validity-2026-08-07.md)
> BEFORE ACTING ON §1.** A validity audit of the correctness lane found that
> most of "Newton loses 5 of 11" is measuring **modelling differences, not
> errors**: `SPEC.md` cites Erez/Tassa/Todorov ICRA'15 as its lineage but
> implements the method that paper argues against, and mandates a
> speed-accuracy Pareto that does not exist; the T5/T7 momentum results are a
> published, explained consequence of joint- vs Cartesian-coordinate
> representation; T1 scores a restitution coefficient **MuJoCo does not have**;
> T6 penetration is a readout of an unequalised free parameter that MuJoCo's
> docs defend as *increased* realism; two T2 metrics are broken as instruments;
> T1/T6 give Newton a **duplicate floor**; and the substeps section that
> produced "4 of 11" **has no data behind it at all**. What survives is listed
> in §9 of that doc. The contract violations below are unaffected — and they,
> not the fidelity scores, are what blocks retirement.

## 1. The prior question: Newton is not ready to be the sole *solver* either

> ⚠ **HISTORICAL — this section's question was answered by events, not by
> evidence.** Newton *is* the sole solver as of `bdc02139`. Read the six bullets
> below as the state of knowledge on 2026-08-07, and note honestly which of them
> were closed versus which were merely made unaskable:
>
> - **"Corpus faithfulness is ~35–40%"** — superseded by the full-corpus sweep in
>   §4.5 (486 worlds, 444 PASS, zero Newton-attributable failures). ⚠ **But that
>   sweep is itself load-and-step only**, and the `coordinateSystem` bug proves
>   what such a sweep cannot see. Neither number is a fidelity measurement.
> - **"The capability gate has become a rubber stamp"** — unchanged in substance,
>   and now unavoidable: with no second backend there is nothing to gate *to*. A
>   world that Newton models badly is simply modelled badly.
> - **"Newton loses 5 of 11 lane-1 correctness metrics"** — first heavily
>   qualified by [`lane1-validity-2026-08-07.md`](../benchmarks/lane1-validity-2026-08-07.md)
>   (most of the gap was modelling difference, not error), and now **unrepeatable**:
>   the lane's oracle *was* ODE. Frozen values live in
>   [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json).
> - **"The shipped default has never been scored … 527 of 725 worlds run that XPBD
>   default"** — **moot.** XPBD was removed (`94f04222`); the default is
>   `SolverMuJoCo`, which *is* the configuration lane 1 always pinned. This
>   particular unmeasured gap closed by elimination.
> - **"The agent-facing contract is already wrong on Newton"** — partly fixed.
>   Native contact readback went default-ON, so `/sim/contacts` and `/sim/grips`
>   are no longer structurally blind. `wake=1` remains a no-op on Newton that
>   still costs two steps (there is no sleep to clear), and `physicsDisableTime`
>   is now a field nothing reads.
> - **"Speed is not a reason to do this"** — **preserved as measured, and it was
>   never refuted.** ODE was the faster single-world backend at every size
>   measured. Retirement happened for capability and maintenance reasons, not
>   performance ones; nobody should read the deletion as a claim that Newton got
>   faster per step. See [step-cost](../benchmarks/step-cost-2026-08-06.md), whose
>   ODE arm can never be re-run.

Before scope, the readiness evidence. None of this is an argument against
retirement eventually; it is an argument that the retirement decision is not yet
informed.

- **Corpus faithfulness is ~35–40%** against a ≥99% bar
  (`newton-ode-replacement-plan.md:37-39`) — and that reading was taken on the
  meter's default corpus of **76 worlds out of a live 701**.
- **The capability gate has become a rubber stamp.** Tier A
  (`OmSolid.cpp:2554`) now admits *every* Webots joint type and mesh is opt-*out*
  (`:2535`). Gate-eligibility is ~100% while faithfulness is 35–40%, so every
  remaining gap is now a **silent** fidelity gap on an admitted articulation.
- **Newton loses 5 of 11 lane-1 correctness metrics** and one, T5 linear
  momentum (3.910 vs ODE's 9.78e-15), is integrator truncation — *not closeable*.
  The 4 wins do not co-exist: T1/T4/T7-angmom need `substeps=4`, which makes T2
  sliding accuracy **12.25× worse than its own substeps=1**.
- **The shipped default has never been scored.** Every lane-1 world pins
  `newtonSolver "mujoco"`, and T2/T6 additionally pin elliptic + impratio 10.
  What a user actually gets — **XPBD + pyramidal + statics off + contacts
  invisible** — is unmeasured. **527 of 725 tracked worlds run that XPBD
  default**, where `joint_f` is inert, contact impulse is 0 and depth is
  hard-coded 0 (`OmNewtonBackend.cpp:768`).
- **The agent-facing contract is already wrong on Newton, today.**
  `/sim/contacts` documents ODE's sleep semantics as the reason it returns
  empty; `physicsDisableTime` is plumbed only into ODE, so `wake=1` writes to a
  field Newton never reads, pays two real steps, and returns the same empty
  list. `/sim/grips` is 100% `getContactPoints`-derived and tells agents *"an
  empty list here **does** mean nothing is gripped"* — false on a default Newton
  world, where the engine's own comment records **1008 contacts on ODE vs 0 on
  Newton** for the same scene.
- **Speed is not a reason to do this.** ODE is the *faster* single-world
  backend at every size measured (40.7× at 1 body, ~1.6–2.2× at 50–200), and at
  200 bodies `mj_step` alone exceeds ODE's entire physics phase. See
  [step-cost-2026-08-06.md](../benchmarks/step-cost-2026-08-06.md). Newton's
  case is batching, and batching does not require ODE's removal.

---

## 2. Phase 0 — five decisions that must be written down before anything runs

These are owner decisions, not agent work, and four of them are external
commitments. **No pod time should be bought before D1 and D2.**

> ✅ **D1 DECIDED BY THE OWNER, 2026-08-07: full deletion is the end state**
> ("we are deleting ODE"), executed locally, solver-first as the path. Two
> consequences recorded the same day:
> - **XPBD is REMOVED** (`94f04222`) — with ODE going and XPBD gone, OmniSim
>   is converging on **one physics engine, SolverMuJoCo**, with CPU and
>   batched-GPU profiles.
> - **The physics-plugin worlds are NOT pinned to `"ode"`** — under deletion
>   they are removed *together with the plugin feature they test* (D5 follows
>   D1). Until the kernel work lands they simply stay red on Newton, which is
>   honest.
>
> What full deletion commits us to, so nobody rediscovers it mid-campaign:
> the six kernel blockers in §0 become the **work plan** (a Newton raycast
> service for DistanceSensor/recognition/LightSensor/Receiver, a weld joint
> for Connector/VacuumGripper, force-type TouchSensor, kinematic collision,
> a buoyancy decision — port or delete `Fluid` —, and a plugin-API
> replacement or removal); Newton must stop reading ODE's `dMass` for its
> inertia; `OMNISIM_WITH_ODE=OFF` must exist before `src/ode/` can go; and
> the D2–D4 external items (Phase W's frozen fixtures, the external no-GPU
> deliverable — now re-groundable on CPU `mj_step`, gated on A1 — and macOS)
> still need their own written answers.

> ⚠ **D2–D5 AFTER THE DELETION: three of the four are now LIVE CONSEQUENCES, not
> pending decisions.** The deletion shipped without written answers to them, so
> they are recorded here as open items rather than as blockers that were cleared:
>
> - **D2 (AgentBench Phase W) — the frozen fixtures reference a backend that no
>   longer exists.** Four hash-frozen scored fixtures pin
>   `defaultPhysicsBackend "ode"` to hold physics constant. That pin is now a
>   no-op at best. Per `preregister/FREEZE.md`, any post-freeze change **voids the
>   campaign** — so Phase W cannot be re-run as frozen *and* cannot be repaired
>   without voiding. It needs an explicit owner call: re-freeze on Newton and
>   re-run, or drop the phase from v6. Do not report a Phase W result gathered
>   after `bdc02139` as comparable to one gathered before it.
> - **D3 (the no-GPU promise) — the mechanism changed, the promise did not.**
>   OmniSim still runs without a GPU, but via CPU `SolverMuJoCo` (`mj_step`), not
>   via ODE. Every place that names ODE as *how* the CPU fallback works is now
>   wrong, including the work-package wording of an external funding commitment
>   to a non-RTX/CPU fallback (recorded in the private ops tree). The commitment is
>   still meetable — `mj_step` is a CPU path — but it is a **third-party
>   commitment whose stated mechanism silently changed**, and it deserves a
>   deliberate re-wording rather than a quiet re-interpretation.
> - **D4 (macOS) — unresolved, and now load-bearing.** The pre-deletion note read
>   *"without ODE, macOS has no physics backend at all."* That is no longer a
>   hypothetical. macOS was already untested for Newton and `scripts/packaging/`
>   had no macOS Newton bundling story; nothing in this campaign changed either.
>   Treat macOS as having **no physics backend** until someone proves otherwise.
> - **D5 (the plugin ABI break) — EXECUTED.** `include/plugins/physics.h` and the
>   in-tree plugin test worlds and controllers are deleted (`a0ed801c`,
>   `e50d45fd`), and the reference pages went with them. This is a breaking
>   change for any out-of-tree physics plugin, with no replacement API.

| # | decision | why it blocks |
|---|---|---|
| **D1** | ~~**Which retirement?**~~ **DECIDED: full deletion (see banner)** | Sets the scope by ~34×. Every other decision depends on it. |
| **D2** | **Phase W** — run as frozen / bump + re-freeze + re-run / drop from v6 | AgentBench Phase W holds physics constant by pinning `defaultPhysicsBackend "ode"` in four hash-frozen scored fixtures (`B1_overlap_audit`, `B2_subject_in_frame`, `C1_parse_error_fix`, `C2_fall_through_floor`). Retiring ODE removes the shared solver family its fairness rests on. `preregister/FREEZE.md:259-267` — any post-freeze change **voids the campaign**. |
| **D3** | **The no-GPU promise** | README states it in ~6 places and **names ODE as the mechanism** (`README.md:79,116,238`). It also matches a **work-package deliverable in an external funding commitment** to a non-RTX/CPU fallback (the application itself is in the private ops tree, not in the public snapshot). This is a commitment to a third party. It can only survive retirement if the CPU `newtonSolver "mujoco"` path is **proven** first — see W-A1. |
| **D4** | **macOS** | macOS is untested for Newton and `scripts/packaging/` has no macOS Newton bundling story. Without ODE, **macOS has no physics backend at all.** |
| **D5** | **The plugin ABI break** | `include/plugins/physics.h` is a shipped public C API. Removing it is a breaking change for every out-of-tree plugin plus 4 in-tree test worlds and ~10 doc pages. |

---

## 3. Fix-before-you-launch (cheap, and each one has already cost a real experiment)

These are prerequisites for the campaign, not part of it. All small.

> **Status:** **F2 was fixed** — `OMNISIM_NEWTON_STATICS` /
> `OMNISIM_NEWTON_NATIVE_CONTACTS` and the gates added afterwards all
> **parse their value**, so `=0` means OFF as documented. Every gate this
> campaign introduced follows that convention. **F1 and F5 are moot** (no pod
> was ever used — see §4). **F3/F4** were never needed at the scale they were
> written for, because the coverage/faithfulness meters were superseded by the
> per-world readiness sweep in §4.5 rather than re-armed. ⚠ That substitution
> replaced a *fidelity* meter with a *load-and-step* one, which is the gap §9
> now records.

| # | item | evidence |
|---|---|---|
| **F1** | **The cloud-pod delete watchdog used by the campaign launch scripts is VERIFIED BROKEN**, and the launcher only warns and continues, so **a campaign launched on it can bill indefinitely.** The failure mechanism, the working replacement and the launch invariants are recorded in the private ops tree (not in the public snapshot). | private ops tree |
| **F2** | **`OMNISIM_NEWTON_STATICS=0` and `OMNISIM_NEWTON_NATIVE_CONTACTS=0` turn the features ON.** Both are presence-gated (`qEnvironmentVariableIsSet` / `qEnvironmentVariableIsEmpty`), so the documented escape hatch does the opposite of what it says. The native-contacts comment block also contradicts itself — `OmSolid.cpp:4117` says "⚠ DEFAULT ON as of 2026-08-03", `:4147` says the flip "was tried and REVERTED". The code is the second. **Fix to value-parsing before any flip experiment, or every A/B arm is inverted.** | `OmSolid.cpp:3192,4129` |
| **F3** | **`newton_coverage.py` and `faithful_check.py` cannot be sharded as written**: neither sets `OMNISIM_LOG_PATH`, and both key probe output on `os.path.basename(world)` — **32 basenames collide** across the corpus (`empty.wbt`, `accelerometer.wbt`, `gps.wbt`, …). Patch to a path hash + per-shard scratch. | — |
| **F4** | **Re-arm the coverage meter.** It reports *eligibility*, which the rubber-stamp gate has made meaningless. It must report **faithfulness** (the W5.4 criterion `faithful_check.py` already implements) or nothing in Wave A is scoreable. | `default-flip-plan.md:56-57` |
| **F5** | `tests/benchmarks/optim_bench.py multi-instance` does not apply the Linux runtime env or `xvfb-run`; it needs a wrapper before it runs on a pod. | — |

---

## 4. The venue — LOCAL (owner decision, 2026-08-07)

> ⚠ **SUPERSEDED BY OWNER DECISION: the campaign runs locally, not on a pod.**
> Two things changed since the pod section below was written. First, the
> solver-default flip (`7b431e81`) landed with its full gate battery run
> **locally in under an hour** — lane-1 exact values, the Newton test suite,
> smoke, and the Husky capability check — which demonstrated the per-item cost
> is laptop-sized. Second, the adversarial review cut the campaign's biggest
> pod consumers (A3/A4 over the full corpus, A7 cross-machine, the B3
> re-baseline), so what remains fits a 16-core RTX 3060 laptop running
> overnight at worst.
>
> **Local execution rules** (this machine has measured ±28% run-to-run spread
> and ~60% thermal drift across a 90-minute session):
> - every A/B **interleaved**, never sequential arms;
> - one engine at a time for anything timed; K≤4 for untimed sweeps;
> - every result through the differenced-window method or the trajectory
>   probe, never a single-window average;
> - every number carries the `env_fingerprint` machine id as always.
>
> **What cannot be answered locally** and stays parked until someone opts back
> into a pod: **A1's honest form** (a genuinely GPU-less box —
> `CUDA_VISIBLE_DEVICES=""` is a weak proxy because warp still sees the
> driver; run the proxy, label it a proxy) and **A7 cross-machine determinism**
> (already deferred — the answer is known to be "differs"). Neither blocks
> solver-retirement readiness.
>
> The pod section below is kept as the recipe for when scale is wanted again.

**Primary (SUPERSEDED — see above): a rented RTX 4090 cloud pod on a persistent network volume carrying the
prebuilt install (provider, volume and image details are in the private ops tree).**

Why: it is the venue this pipeline is already built for and proven on — the
volume carries the repo, Qt 6.5.3, `bin/omnisim-bin` and (critically) the **warp
kernel cache**, so a warm pod is training-ready in ~5–6 min instead of a 907 s
cold build. Network volumes attach **only** to Secure Cloud pods in the same
datacenter. Pods report **96 vCPU / 251 GB**, and the private ops tree records
that vCPU/RAM are never the bottleneck (load 6 of 96 cores) — which is
exactly the resource this campaign is bound by, so it is free.

**Two satellites:**

- **A CPU-only pod (cheapest available)** for W-A1, the GPU-less honesty test.
  This is the one measurement that *cannot* be taken on a GPU box, and it is the
  gate on D3. `use_mujoco_cpu` only makes the **solver** CPU — the warp Model
  still takes the default device — so `CUDA_VISIBLE_DEVICES=""` is a weaker
  proxy than real absent hardware. Run both.
- **A second 4090 in a different datacenter** (its own network volume) for the
  cross-machine determinism arm, which by definition needs a different machine.

**Confirm before planning spend:** that the provider API credential is still
valid, and that both volumes still exist — they bill ~$2.80/mo each and may
have been reaped since 2026-07-25. The credential location is recorded in the
private ops tree, never here.

**Non-negotiables** (recorded in full in the private ops tree):
arm the watchdog **on the pod** as the first command after SSH (F1); write all
results under `/workspace` (a pod can and did vanish mid-run); **TERMINATE, never
stop** (a stopped pod bills container disk at *double* the running rate, releases
the GPU without reserving it, and may resume with zero GPUs); confirm
`GET /v1/pods` returns `[]` at session end; know the exit condition before
creating the pod.

**Estimated total: ~$12–18** across the whole of Wave A and Wave B. This campaign
is cheap; the expensive resource is decision latency, not GPU time.

---

## 4.45 NEW (2026-08-08) — a runtime-DELETED node is never removed from the MuJoCo model

Found while repairing the Radar / Camera-recognition occlusion carrier (the last
ODE ray consumer). The repair itself is done and verified — a wall now blocks a
target, proven by a control arm (`occlusion FALSE` sees both) plus a differential
(`occlusion TRUE` drops exactly the walled one) in
[`tests/test_newton_radar_occlusion_parity.py`](../../tests/test_newton_radar_occlusion_parity.py).
Making occlusion *live* immediately exposed something underneath it.

**`tests/api/worlds/camera_recognition.omniworld` now gets FURTHER and still fails**,
and the two facts together are the finding:

| assertion | before the repair | after |
|---|---|---|
| 8 objects with `occlusion 0` | pass | pass |
| **7 objects with `occlusion 2`** | **FAIL — saw 8** (no ray was ever cast) | **pass** |
| 1 object visible once the occluders are *removed* | not reached | **FAIL — sees 0** |

The last row is not an occlusion bug. `wb_supervisor_node_remove_node()` deletes
the Solid from the scene graph, but **there is no remove/unregister path from a
deleted Solid to the Newton/MuJoCo model at all** — grep
`src/omnisim/physics/OmNewtonBackend.hpp` for a `removeBody` and there is none.
The deleted geometry stays in the model, so `mj_ray` still hits it and the target
it used to hide is still reported occluded.

**Why this is bigger than occlusion, and why it surfaced only now.** Rays are
merely the first consumer honest enough to notice. The same stale geometry is
presumably still a *collider*, which would mean a deleted wall still stops a
robot and a deleted floor still holds a body up. That was structurally
unobservable before **2026-08-07**, when static colliders were off by default
(`newtonStatics`) — a static that collided with nothing could not be caught
still colliding after deletion. Two defaults flipping (statics on, native
contacts on) plus a live raycast are what make it reachable.

### ✅ MEASURED, not inferred — the contact half is CONFIRMED (2026-08-08)

The discriminating experiment was run, and it is not rays-only. World:
[`tests/physics/worlds/gravity_rest_height.omniworld`](../../tests/physics/worlds/gravity_rest_height.omniworld)
— `FLOOR` is **elevated** (centre z=0.4, 0.2 thick, top **0.50**) precisely so the
implicit z=0 ground plane cannot stand in for it, and `FALLER` is a 0.2 m box.
Driven over the HTTP harness on `:6889`, machine `9722d23d12a3`, dt **4 ms**:

1. `FALLER` fell from its authored z=1.5 and settled at **z = 0.5999** — floor top
   + half height. **This is the within-run control: gravity and contact both
   demonstrably worked in this process, in this world, moments earlier.**
2. `POST /scene/delete {"def":"FLOOR"}` → `{"removed":[{"def":"FLOOR",...}],
   "verification":{"all_removed":true}}`, and `GET /scene/tree` no longer lists it.
3. Stepped on. `sim_time_ms` advanced 1280 → 2024 → and the engine's own per-step
   log reached **step 61440 (~246 s of sim time)**, so the world was unambiguously
   running.
4. `FALLER` never moved: **z = 0.5999 at every sample**, unchanged to four
   decimals. Falling the 0.5 m to the implicit plane would have taken ~320 ms
   (~80 steps); it had ~770× that.

The engine log is the direct proof, because it names the bodies it is stepping:

```
INFO: [OmNewtonBackend] step 61440 dt=0.004s b0=(0.000,0.000,0.400) b1=(-0.000,-0.000,0.600) ...
```

`b0` **is the deleted floor**, still registered and still at its authored pose,
61,440 steps after the Solid was removed from the scene graph.

**So a runtime-deleted static Solid keeps colliding.** A deleted wall still stops
a robot; a deleted floor still holds a body up. Nothing warns, because nothing in
the engine knows the model is stale. Scope of what was measured: one static,
non-articulated Solid, `newtonSolver` default (CPU `mj_step`). Dynamic bodies,
whole robots, and `mujoco_warp` were **not** tested — do not assume the same
shape for those without measuring, and do not assume they are fine either.

**Interim guidance** (shipped in `GET /capabilities` → `not_supported` as
`OCCLUSION_RAYS_UNANSWERED`'s note, in AGENTS.md, and in the Newton guide):
reload the world (`POST /world/load`) after removing collidable nodes rather
than trusting occlusion — or contacts — against a scene you have mutated. Note
the failure is **silent**: no warning fires, because nothing in the engine knows
the model is stale.

## 4.5 Readiness scoreboard — live, local, updated as sweeps land

`scripts/dev/newton_readiness_sweep.py` (committed `16c0332f`): every world in
its own engine process, PASS = loaded + stepped + no FATAL/ERROR + sidecar
present, failures grouped by first FATAL line. Flake discipline: the
intermittent FFI bring-up fault (~5% of cold launches) is separated from real
failures by **retrying every FFI-flagged world once** — 7 of 8 flagged worlds
across the first two sweeps passed on retry; the 8th unmasked a real failure
underneath.

> ⚠️ **THE `tests/api` ROW BELOW IS THE CAMPAIGN'S CAUTIONARY TALE — DO NOT QUOTE
> IT AS A PHYSICS RESULT.** All 140 of those worlds are `NUE` (Y-up), and until
> `c77cbe98` Newton built every world `up_axis=Z` regardless. Gravity projected
> onto the up vector came out **exactly zero** and the implicit ground plane stood
> up as a vertical wall. **Nothing in any of those 140 worlds ever fell**, and they
> still scored 140/140, because PASS here means *loaded + stepped + no
> FATAL/ERROR + sidecar present* — a log verdict with no view of the physics.
> Post-fix, the honest score is **128/139**. Quote that one.
>
> **Why the denominator moved 140 → 139** (so it does not read as a typo): the deletions
> took test worlds with them. `tests/api/worlds/emitter_receiver_physics_plugin.wbt` went
> with the plugin ABI (`a0ed801c`), leaving **139**. `tests/physics` fell **35 → 32** the
> same way — `determinism.wbt` and `spring_damping_force.wbt` (the two physics-plugin
> worlds) plus `connector_detach.wbt`. So the one "real failure" in the table below,
> `determinism.wbt`, is no longer a failure: the world does not exist.

| group | PASS (post-retry) | real failures | failure class |
|---|---|---|---|
| `tests/physics` (35 → **32**) | **34/35 (97%)** | 1 → **0** | `determinism.wbt` — the ODE plugin world, red by decision. ✅ **It was duly deleted** (`a0ed801c`), with `spring_damping_force.wbt` and `connector_detach.wbt`; the stratum is now 32 worlds and this failure class is empty |
| `tests/api` (140) | ~~**140/140 (100%)**~~ → **128/139** post-`c77cbe98` | 0 → 11 | ⚠ the 140/140 was measured with gravity at exactly zero — see the banner above |
| **combined** | ~~**174/175 (99%)**~~ — superseded | — | — |

> ✅ **The registration-gap class is CLOSED (2026-08-07, `807c2612`)** — kinematic
> chains gate to ODE loudly (warning, not FATAL; reason `"kinematic"` in the
> coverage histogram), and a jointed physics-bearing Solid folded into a
> body-less merge leader now registers itself. Earlier scoreboard for the
> record: physics 29/35, api 132/140 (161/175) on the morning's binary.

**Every real failure so far is the same thing**: a joint whose parent or child
Solid never registers a Newton body, refused loudly by newton-enforce rather
than silently going inert. The named sub-cases, mapping 1:1 onto the Tier-1
capability worklist (§7):

- **ball-joint children** — `ball_joint_reset`, `ball_joint_vs_hinge_joints`
- **hinge2 bodies** — `hinge_2_joint_damping_axis`, `motor2_velocity`,
  `motor3_velocity` (the pedagogical Hinge2/Ball worlds)
- **nested / physics-less Solids as joint bodies** — `gps_speed`
  (`solid_without_physic`), `joint_end_point_collision`,
  `supervisor_get_position_orientation_pose`,
  `supervisor_reset_physics_hinge_joint`, `supervisor_set_position_orientation`,
  `touch_sensor_kinematic`, `hidden_parameter_single`
- **physics-plugin worlds (structurally ODE)** — `determinism`,
  `spring_damping_force`: their C API *is* ODE; the honest fix is an explicit
  `physicsBackend "ode"` pin, not a port.
  > ⚠ **That instruction is dead.** There is no `"ode"` to pin. These worlds were
  > **deleted with the plugin feature** (`a0ed801c`, `e50d45fd`) — D5 followed D1.

> ✅ **THE FULL CAMPAIGN IS COMPLETE (2026-08-07, agent-swept): 486 worlds,
> and ZERO real failures are Newton-attributable.**
>
> ⚠️ **READ THIS BEFORE QUOTING THE TABLE.** This is a **load-and-step sweep**:
> PASS = loaded, stepped, no FATAL/ERROR, sidecar present. It is a strong result
> about *robustness* and says nothing about *fidelity*. One day later `c77cbe98`
> showed exactly how much it can miss — 210 of the tree's `NUE` worlds, 29% of
> 719, were running with gravity projected to **exactly zero** and an implicit
> floor rotated into a vertical wall, and this sweep scored them green. "Zero
> Newton-attributable failures" means *nothing crashed*, not *the physics is
> right*. The `tests/api 140/140` row below is the same poisoned number corrected
> to **128/139** above.
>
> | stratum | post-retry |
> |---|---|
> | tests/physics | 34/35 |
> | tests/api | 140/140 |
> | tests/protos | 47/49 |
> | tests/parser | 14/44 |
> | tests/cache + rendering + other_api | 10/19 |
> | projects/samples/demos | **74/74** |
> | projects/robots + omni_quest + generated | **33/33** |
> | projects/robot_combat | **33/33** |
> | projects/samples/{devices,geometries,rendering} | **59/59** |
> | **total** | **444/486** |
>
> Every one of the 41 real failures is orthogonal to physics: 30 are the
> parser stratum's DELIBERATE negative tests (they exist to produce parse
> errors; 26 of 30 first-error lines match `expected_results.txt` verbatim),
> 8 are offline web-cache EXTERNPROTO fetches, 2 are R2025a backward-compat
> refusals, 1 is PROTO parameter drift. **Every shipped world — demos,
> robots, combat, samples, including the only Connector and only Fluid
> worlds — loads and steps on the Newton default.** The FFI bring-up flake
> measured 3.4% over ~328 launches, all recovered on retry.
>
> Caveats recorded by the sweep: 96 passing worlds ended before finalize
> under the 12 s budget, so their backend is load-and-step-verified but not
> sidecar-certified (a longer-budget pass would certify them); the sweep's
> parallel verdict is not port-contention-proof (8 protos worlds needed a
> solo re-run after zombie engines took ports 1234–1237); and node coverage
> for the kernel work reads DistanceSensor ×42 (40 pass), Lidar ×18 (all
> pass — WREN-based, already ODE-free), TouchSensor ×12, Connector ×3,
> Fluid ×1, all passing.

## 5. Wave A — the evidence campaign (fully parallel, no interdependencies)

> ⚠️ **NEVER RUN AS SPECIFIED — DO NOT PICK THIS UP AS A WORK PLAN.** No pod was
> bought; the venue decision in §4 went local and the campaign then went straight
> to engineering. §§5, 6 and 8 (Wave A, Wave B, the 20-agent execution shape) are
> a **plan that was overtaken**, not a backlog. Several items are now
> unrunnable on their own terms because they compare against ODE: **A2** scores a
> default (XPBD) that no longer exists, **A5**'s premise is that `run_smoke.py`
> forces ODE, **A7** replays lane 1 whose oracle was ODE, and **A8** says "run …
> on both backends" — there is one backend. **A1** (the no-GPU / D3 gate) is the
> one item that is both still meaningful and still unanswered: nobody has proven
> the CPU `mj_step` path on a GPU-less machine, and D3 is a third-party
> commitment. If you resurrect anything from this section, resurrect A1.

Every item is independent. Assign one agent each; they share a pod but not
state. All results land under `/workspace/.../results/<machine_id>/<utc-date>/`.

| id | question | how | shard key | est. |
|---|---|---|---|---|
| **A1** | **Does OmniSim run without a GPU, without ODE?** The D3 gate. | CPU-only pod. Load a representative world set with `newtonSolver "mujoco"` and `OMNISIM_REQUIRE_NEWTON=1`; record the `.newton.json` sidecar. Then repeat with no `newtonSolver` pin (the real default, XPBD) and record what happens. | — | 1 pod-hr |
| **A2** | **Score the shipped DEFAULT on lane 1.** Never done. | `run_all.py --lanes 1` on a variant world set with **no** `newtonSolver`, **no** cone pin, statics off, contacts off — i.e. what a user gets. Compare against the existing pinned rows. | dt × scene | 2 h |
| **A3** | **Coverage meter over all 701 live worlds**, not 76. | `newton_coverage.py` post-F3/F4. | world path, ~11 shards | 25 min at 24-way |
| **A4** | **`faithful_check` over all 701.** The real ≥99% number. | `faithful_check.py` post-F3. 2 engine runs/world. | world path, ~11 shards | ~1.5 h at 24-way |
| **A5** | **Do the 289 group-runner worlds pass on Newton?** Unknown — only 1 of 289 pins a backend and `test_group` does **not** set `OMNISIM_FORCE_ODE`, unlike `run_smoke.py` which forces it because the goldens are ODE-tuned. ⚠ **The premise expired:** `run_smoke.py` no longer forces ODE (it scrubs the var); the underlying question — *do those 289 worlds pass on the only backend there is?* — is still open and is now the only form it can take. | `omnisim test-group` × 7. A group is one process and cannot be split. | test group (7 shards) | ~1 h |
| **A6** | **Damage regression on Newton.** ~3 of 7 ranges are impulse/penetration-derived. | `omnisim damage-regression` | scenario (7) | 10 min |
| **A7** | **Cross-machine determinism.** 56 of 180 lane-1 cells already differ between two machines, 13 at rel ≥1e-3. | lane 3a on both pods, same build sha. | pod | 1 h |
| **A8** | **Sensor behaviour under Newton.** No gate covers lidar / rangefinder / camera returns at all. | Author a probe; run the 68 DistanceSensor + 27 Receiver + 16 TouchSensor + 7 Radar + 6 LightSensor worlds on both backends. | sensor type | 1 h |

**Wave A exit:** one table saying, for the real corpus and the real default, what
fraction of OmniSim works on Newton. That table is what D1 needs and does not
have.

---

## 6. Wave B — the flip experiments (parallel across flips; each needs Wave A)

Five defaults, each measured as an **independent, interleaved A/B** with the
value-parsing hatch from F2 as the revert arm. Interleave — a sequential
comparison on a shared box produced sign-disagreeing deltas in this tree twice
this week.

| id | flip | known effect | shield status |
|---|---|---|---|
| **B1** | `newtonStatics` FALSE → TRUE | ball settles at **0.0996** on a phantom plane vs **0.6496** correct | 47 worlds already ask explicitly; **3 Go2 world-deploys + `omniquad_walk` + `h1_walk` unshielded** |
| **B2** | native contacts OFF → ON | **1008 → 0** contact blindness today | fixes `/sim/contacts` + `/sim/grips`; damage-tracker thresholds move |
| **B3** | cone pyramidal → elliptic + impratio 10 | T2 incline 26° @ μ=0.5: **181 mm** creep → **0.6 mm** | ⚠ **15/15 skills and 6/6 sequences unshielded** — 0 manifests and 0 policy worlds pin a cone |
| **B4** | friction actually reaching contacts | `newtonGroundMu 2.0` declared, box still slides 4.8 m | **an ORDER bug, not routing** — `flushPendingNewtonRegistrations()` adds shapes (copying `cfg.mu`) *before* pushing the WorldInfo contact block at `OmSolid.cpp:3665`. Fix: hoist that call above the registration loop |
| **B5** | limit-less motor ⇒ velocity wheel | commanded 0.5 rad → ODE 0.500000, Newton **0.000000** | naive fix pins husky wheels; classification must become revisable at first finite `setPosition` |

**B6 — the RL bill, and it is the real one.** `policy verify-demos` is a
**dict comparison, ~1 s, no engine** — it would stay 100% green through any of
these flips while every champion silently changed. There is currently **no gate
that runs a shipped RL champion and scores its behaviour**. Wave B must add one:
`skill_lib.py run <skill>` (~60 s × 9) + `sequence <name>` (~24 min of sim per
arm) × 3 repeats × 2 arms, to prove `OMNISIM_NEWTON_CONE=pyramidal` is an exact
revert. Note also that champions must be re-verified on **CPU `mujoco`** — it is
the only bitwise-reproducible Newton path; `mujoco_warp` is **0 bitwise of 24
pairs, 9.152 m apart at 1000 steps**.

---

## 7. Wave C — engineering, in dependency order

From the capability audit. **Tier 1 is fully parallel and changes no working
world's physics** — it is the best place to put many agents.

- **Tier 0 (blocking):** F4, re-arm the meter.
- **Tier 1 (parallel, no re-baseline)** — ⚠ **REVISED 2026-08-07 after a usage
  audit; three of the six items below were mis-sized or mis-specified in the
  first draft, and one of them would have shipped a no-op:**

  1. **Shape-walker unification + `Pose` rotation — PROMOTED to the highest-value
     item** (was "S"). It is **four** partial walkers, not two, and they diverge
     physically: the recursive one substitutes `addShapeSphere` for a cylinder
     ([`OmSolid.cpp:2831`](../../src/omnisim/nodes/OmSolid.cpp#L2831)) while the
     default one uses a capsule (`:2958`), so flipping
     `newtonCompoundColliders` silently downgrades every cylinder wheel from a
     line contact to a point contact — and the comment at `:2761` claiming they
     are "shared verbatim" is stale and false. **Both** walkers drop `Pose`
     rotation (`:2859`, `:2927` accumulate translation only). Population: 76
     worlds with a rotated `Pose` in a `boundingObject` — including the flagship
     `omniarm6_universal_pick.omniworld` and 24 lane-1 benchmark cells — plus **124
     rotated collision origins across 19 URDFs** (b2 40, go2 12, ur5e 11, h1 8…)
     reaching ~161 more worlds. Every one of those colliders is currently placed
     with the wrong orientation on Newton.
  2. **Fixed-joint child collider harvest (M)** — confirmed the largest by
     population. The static path harvests descendant colliders (`:3225-3242`);
     the dynamic path reads only `s->mBoundingObject` (`:3470-3478`) and never
     got the same fix. Mass *is* rolled up, so mass is right and geometry is
     missing — the silent-failure shape. ~221 non-ODE-pinned worlds, all the
     flagship robots.
  3. **B4 friction ordering (S)** — ⚠ **the first draft's fix description was
     WRONG and would have shipped a green diff that changes nothing.** Hoisting
     the `:3641` block above the registration loop is a no-op on the first flush,
     because `isWorldOpenForBuild()` is false until `ensureWorldOpen()` runs —
     and `ensureWorldOpen()` is called *inside* the loop (`:3162/3248/3278`).
     Worse, `ensureWorldOpen()` itself calls `addGroundPlane()`, so the implicit
     ground plane — the surface `newtonGroundMu` most often exists to tune — is
     created with the constructor's cfg. **The real fix is to push the WorldInfo
     contact block before `beginWorld()`/`addGroundPlane()`**, i.e. a restructure
     of the world-open sequence. Also note `World._contact_world` and
     `World._SHAPE_CFG` are **class**-level, so a second world load in one
     process inherits the previous world's friction — worth its own test.
  4. **Heightfield / `ElevationGrid` (M)** — keep, but the first draft's
     mechanism was wrong. `OmElevationGrid` derives from `OmGeometry`, not
     `OmTriangleMeshGeometry`, so it matches no branch and yields an empty
     `shapeDesc`; all 12 affected worlds are **static**, and the static path
     discards the return with **no fallback at all** — the r=0.12 sphere
     (`:3479`) is dynamic-path-only and unreachable here. So the terrain has
     *zero* shapes, not a sphere. Only 4 of the "16 files" use it as a collider;
     12 worlds total once `UnevenTerrain.proto` is resolved, including
     `husky_rocks_traverse.omniworld` (in DEMOS.md) and `mars.wbt` (in the quickstart).
     Note the three `generated_worlds` came from a **retired** generator —
     `omniworld` emits no `ElevationGrid` — so regenerating will not reproduce them.
  5. **Motorised Hinge2 + Ball — DEPRIORITISED.** The defect is real (both
     branches `continue` before the motor registration at
     [`OmBasicJoint.cpp:589`](../../src/omnisim/nodes/OmBasicJoint.cpp#L589), so
     they are never in the list `pushNewtonMotorTargets()` iterates). But the
     population is 20 worlds, of which 16 are `tests/` and 4 are
     `projects/samples/{devices,geometries}` — which `WORLDS.md:127` itself calls
     a "pedagogical tour, NOT regression tests". **No demo, no flagship, no
     policy world, no benchmark scene.** A load-time warning is the honest fix.

     > ⚠ **STILL BROKEN AFTER THE DELETION — and the diagnosis has moved.** The
     > `continue`-before-registration reading above is **not** the live cause:
     > registration and motor enrollment are both verified wired. The defect is
     > in the **runtime d6 joint build at finalize**, and the gate ships
     > **default OFF**. The sizing stands, but the stakes changed: while ODE
     > shipped, these worlds could be pinned to a backend that actuated them.
     > Now nothing does. Motorised `BallJoint` / `Hinge2Joint` is a real
     > capability hole, not a deprioritised nicety.
  6. **Plane-on-static dropped under MuJoCo — DROPPED from Tier 1.** The first
     draft justified it with "184 worlds pin `newtonSolver \"mujoco\"`". That
     figure is wrong (the real count is **334**, or 348 including
     `mujoco_warp`) *and* irrelevant to the defect, which needs a **raised or
     tilted** Plane. Of the 171 mujoco worlds with a Plane-backed floor:
     **0 raised, 0 tilted.** Sweeping all 901 worlds finds 13 raised and 66
     tilted, and **none of the 79 pins `newtonSolver` at all** (the 66 are
     `rotation 1 0 0 -1.5708` Y-up→Z-up conversions in legacy `NUE` test
     worlds). The mechanism is real
     ([`OmNewtonBackend.cpp:1344`](../../src/omnisim/physics/OmNewtonBackend.cpp#L1344))
     but **unreachable in the live tree**. Document it in `worldinfo.md` as a
     latent-defect guard; do not staff it.
- **Tier 2 (gated on Tier 1):** B1 statics needs the fixed-joint and Plane fixes
  first, or it registers some statics correctly and others as fallback spheres.
  B2 native contacts needs contact **magnitude** to be non-zero first, or it
  trades blindness for noise.
- **Tier 3 (gated on a re-baseline capability):** B3 cone → champion
  re-verification → which is gated on determinism scope.
- **Not closeable, document instead:** CFM/ERP have no Featherstone equivalent —
  and lane-1 T1 needs `softCFM 1e-6` to get e≈0.8 at all. T5 linear momentum.
- **Last:** W4.3, dropping the ODE keepalive. Every Newton Solid still carries a
  disabled ODE body whose **geoms stay in the space and still collide**, because
  rays, TouchSensor, kinematic collision and cross-backend contacts need them.
  This is a *correctness* dependency, not a performance one — removing the
  redundant ODE pass measured **−0.0116 ms/step, signs disagreeing**
  ([step-cost](../benchmarks/step-cost-2026-08-06.md)). **You cannot stage this
  per-node**; every geom must survive until all of §0's six blockers are ported.

  > ✅ **DONE — and this bullet's own reasoning is why it could be.** All six
  > blockers were ported or deleted first, so the keepalive's four dependents
  > (rays, force `TouchSensor`, kinematic collision, cross-backend contacts) no
  > longer needed an ODE geom. The keepalive went out with `src/ode/`
  > (`bdc02139`) as a single non-stageable step, exactly as predicted. Expect **no
  > measurable speed-up** from it: the removal measured −0.0116 ms/step with signs
  > disagreeing, i.e. indistinguishable from noise. Anyone attributing a perf
  > change to the deletion should re-measure.

---

## 8. Parallel execution shape

~20 agents, three groups, no shared mutable state:

- **Group 1 (8 agents, Wave A)** — one per A-item. Read-only against the tree;
  each writes to its own results dir. A3/A4 fan out further internally to ~11
  world-path shards each.
- **Group 2 (6 agents, Tier 1 engineering)** — one per fix. Each touches
  disjoint code; each lands with a red-first test in the style of the five
  already in the working tree.
- **Group 3 (5 agents, Wave B)** — one per flip, each owning its interleaved A/B
  and its re-baseline.

**Per-shard requirements** (all four are load-bearing; each has a recorded
failure behind it):

1. unique `OMNISIM_LOG_PATH` per child — without it K=4 loading-scaling measured
   **3.22×** the K=1 child, against 1.29× with it;
2. unique scratch prefix (F3, the 32 colliding basenames);
3. **assert the `.newton.json` sidecar**, never the exit code and never the log —
   a whole Go2 head-to-head was once run on policies that never loaded, printing
   `PASS`;
4. `xvfb-run` **per engine child** — children that bypass the common launcher
   need their own, and SIGABRT'd at platform init on both tier-C attempts of a
   previous campaign without it.

`--parallel` is now capped at 24 rather than 8 (`8e5f022f`); the binding
constraint is the controller-connect launch race and killed-engine port residue,
not the port range. **Do not parallelise `physics_oracle.py`** — its 8-attempt
backoff ladder exists precisely because concurrent launches lose that race; give
it one engine per machine.

---

## 9. Recommendation

> ⚠️ **THIS RECOMMENDATION WAS OVERRULED, AND THE OUTCOME CONTRADICTS IT.**
> Preserved verbatim because it records what the evidence supported at the time
> and because its cost estimate was the thing that turned out to be wrong.
>
> The owner chose **full deletion**, and kernel removal shipped in days rather
> than being deferred out of v6: a native raycast service, a weld joint, native
> kinematic bodies, force feedback, a native inertia integrator, `Fluid` deleted
> and the plugin ABI deleted — the six subsystems this section calls "six
> from-scratch subsystems … nothing in the tree has a design for any of them".
> Designs were written and executed (they lived in `_scratch/design_*.md`).
>
> ⚠️ **`_scratch/` IS UNPUBLISHED AND WILL NOT SURVIVE — and load-bearing knowledge is
> still only there.** At least nine committed files defer to it in comments, including
> `OmNewtonBackend.{hpp,cpp}`, `OmSolid.{cpp,hpp}`, `OmConnector`, `OmVacuumGripper`,
> `OmTouchSensor`, `OmOdeTypes.hpp`, `OmSimulationCluster.cpp`, three
> `tests/test_newton_*_parity.py`, and `projects/policies/research/backends/g1_deploy_runtime.py`.
> `OmVacuumGripper.cpp` is the worst case: it says *"Phase-1 approximations, both recorded
> in `_scratch/design_weld_touch.md`"* — deferring the actual content to a file that is
> about to vanish. **Promote before deleting `_scratch/`.** The four things that are
> genuinely unrecoverable from the code, because their source (`src/ode/ode/src/mass.cpp`,
> the collision cluster) is deleted:
>
> 1. **The dMass frame conventions and op semantics** that `OmInertia` was built to
>    reproduce, plus the Mirtich-port rationale and the reason op *ordering* must match to
>    hit 1e-12 on compounds.
> 2. **The corrected `eq_data` weld anchor encoding.** ⚠ The design doc still teaches the
>    **wrong** one (`anchor=0` + `relpos=current`), which sags whenever the weld point is
>    far from body2's origin — a 1 kg body at (3,2,1) sagged to z=0.065. The correct
>    encoding (anchor = body_a's origin in body_b's frame, `relpos=0`, `relquat = q_a⁻¹·q_b`
>    wxyz) holds to 4e-4 and exists only in a code comment.
> 3. **The ODE ray-filtering semantics** the Newton raycast service replicates, and the
>    seven-consumer contract table (which sensor needs the normal, which skips transparent
>    geometry, which casts from the emitter).
> 4. **The mocap-body decision** for kinematic bodies, its rejected alternatives, and the
>    9-/34-world census of affected worlds.
>
> Two standing defects also live only there: **density-only `Physics` nodes get 0.25 kg**
> (ODE derived ρ·V; no code path computes it now, and no oracle remains to check a fix),
> and the **COM/frame pairing fed to `add_link` is a deliberate self-consistent
> approximation** that nothing in the code marks as such — so someone will "fix" it and
> silently change every world's dynamics.
>
> **What the recommendation got right:** the *sequencing*. Every blocker did have
> to be closed before the keepalive could go, and it could not be staged
> per-node. **What it got wrong:** the size. The estimate treated "no design
> exists" as "no design is reachable".
>
> Wave A (below) was **never run as specified** — no pod was bought. The
> question it existed to answer was answered a different way, by the local
> full-corpus sweep in §4.5 (486 worlds) plus the per-blocker parity tests. ⚠ Note
> the substitution honestly: Wave A asked a **fidelity** question and the sweep
> that replaced it is **load-and-step**. The fidelity question was never actually
> answered — the `coordinateSystem` bug is the proof, and with ODE gone there is
> no oracle left to answer it against.

**Take D1 as solver-only, and do not schedule kernel removal in v6.** The kernel
is six from-scratch subsystems (a raycast service Newton does not have, a weld
joint, a buoyancy engine, kinematic collision, force feedback, and a public C
ABI replacement) behind no abstraction, and nothing in the tree has a design for
any of them.

**Then run Wave A before committing to even that.** It is ~$12–18 and about a
day of pod time, it is the cheapest part of this whole programme, and it answers
the question the four external commitments turn on: *does OmniSim actually work
on Newton, in the configuration users get, on the corpus we ship?* Right now the
honest answer is that nobody knows — the number everyone quotes (35–40%) was
measured on 11% of the corpus, in a configuration 28% of worlds select.

If Wave A comes back strong, solver retirement is weeks of Tier-1/2 work. If it
comes back weak, we will have learned that for the price of a coffee instead of
in a release.
