# Physics in OmniSim — Newton/MuJoCo, and nothing else

**OmniSim has exactly one physics backend: [Newton](https://github.com/newton-physics/newton).** ODE — the CPU rigid-body engine inherited from Webots — was **deleted on 2026-08-08** (commit `bdc02139`, removing `src/ode` and `include/ode`, 106,283 lines). There is no second solver, no CPU fallback tier, and no build flag or environment variable that brings one back.

If you have read an older version of this page, or older commits, or Webots documentation: **most of what you know about choosing a backend no longer applies.** This page is the current picture. Where a claim is a past measurement, it says so and gives the date.

## What this means in practice

- **The Newton Python runtime is a hard requirement, not a capability.** A binary that cannot import `newton` / `warp` / `mujoco` through its embedded interpreter has *no physics at all* — nothing falls, nothing drives. That is a broken install, not a degraded mode. See [Build and runtime requirements](#build-and-runtime-requirements).
- **`physicsBackend` and `defaultPhysicsBackend` still exist, and still accept `"ode"`, but `"ode"` no longer selects a working solver.** See [The backend fields today](#the-backend-fields-today).
- **The `newtonSolver` field is the decision that matters now** — CPU `mj_step` (the default; deterministic, no GPU needed) versus GPU `mujoco_warp` (for batched training only). See [Choosing a solver](#choosing-a-solver-cpu-vs-gpu).
- **Several ODE-era nodes and fields are gone or inert.** See [What went away with ODE](#what-went-away-with-ode) and [Known gaps](#known-gaps--what-does-not-work-yet).

## The backend fields today

`Solid.physicsBackend` (which cascades down a scene tree, so setting it on an outer `URDFRobot` covers the whole robot) and `WorldInfo.defaultPhysicsBackend` are **still declared in the node schema** and still accept `"ode"`. That is deliberate: an *undeclared* field is a parse ERROR that takes a headless run's exit code to 1, and worlds in this tree still carry the value.

What the value does now, read from the engine source (`OmPhysicsBackend.cpp`, `OmSolid.cpp`) — and corroborated by a measurement of the process-wide equivalent, `OMNISIM_FORCE_ODE=1`, which on 2026-08-08 left `tests/physics/worlds/contact_points.omniworld` **frozen at its authored pose for the whole run** (see [Verifying that Newton actually drove a run](#verifying-that-newton-actually-drove-a-run)):

| value | effect today |
|---|---|
| omitted / `"auto"` | resolves to Newton. **This is what you want.** |
| `"newton"` | resolves to Newton, explicitly. Harmless and self-documenting. |
| `"ode"` | `OmOdeBackend::isAvailable()` is now `false` and every one of its operations returns `-1`; `OmSolid::flushPendingNewtonRegistrations` **skips** any Solid whose effective backend resolves to `"ode"`. So the Solid is registered with no solver — **it is not simulated.** |

⚠️ **The engine does not warn when the `"ode"` choice was explicit** (it treats an explicit choice as already answered). So on an older world, "this body is frozen" or "this prop never falls" should make you check for `physicsBackend "ode"` *before* you suspect a missing `boundingObject` or a solver bug.

**Never write `"ode"` into a new world, and never suggest it as a workaround for anything.** Leave the field off, or write `"newton"`.

## Choosing a solver (CPU vs GPU)

`WorldInfo.newtonSolver` is the field that actually changes physics behaviour. The solver is `SolverMuJoCo` either way — the field selects where it runs:

| `newtonSolver` | what it is | when |
|---|---|---|
| `""` / `"auto"` / `"mujoco"` | reference **CPU** `mj_step`. Needs no GPU. Bitwise reproducible run-to-run. | **The default, and the right choice for single-world work**: demos, authoring, regression tests, single-robot deploy. |
| `"mujoco_warp"` | the same solver **batched on the GPU** via Warp. Needs a CUDA GPU and an **even** `newtonSubsteps`. | Parallel RL training (thousands of envs). Nothing else. |

Two things to know before you reach for the GPU:

- **`mujoco_warp` at `nworld=1` measured 9.06× *slower* than CPU `mj_step`** (105.3 vs 11.6 ms/step — launch-bound, and the CUDA graph that would fix it is unreachable at the default substep count). ⚠ n=1, so the magnitude is soft; the sign is not. "Use the GPU" is not the single-world answer.
- **`mujoco_warp` is not run-to-run reproducible.** Measured: 0 bitwise of 24 same-config cold pairs, ~5e-5 m apart by 120 steps and **9.152 m apart by 1000 steps**, because it claims contact slots with `wp.atomic_add` so buffer order is thread-arrival order.

> **⚠️ If your run has to be replayable, pin `newtonSolver "mujoco"`.** The CPU path
> reproduces bitwise across cold launches, verified on a 336-contact / 1344-constraint-row
> ten-robot scene with ten live controllers on machine `9722d23d12a3`. **Cross-machine**
> bitwise identity is **untested** on every path. Full scope, numbers and reproduction
> commands: [determinism-scope.md](../benchmarks/determinism-scope.md).
>
> Historical note, kept because you will meet it in older docs: ODE was also bitwise
> reproducible, and through the 2026-07-24 OmniBench campaign `omnisim-ode` was best or
> tied-best on 6 of the 7 analytic-ground-truth lane-1 scenes at dt = 4 ms, with linear
> momentum exactly zero to double precision. ⚠️ **That is a comparison of two integrations,
> not two solvers, and older docs compress it wrongly.** Bare MuJoCo scored fine on the same
> scenes with the same solver family; the four defects behind our Newton integration's
> deficit (friction cone, rolling inertia, momentum leak, spin loss) were in the plumbing
> between the scene graph and the solver, and all four were fixed in `e7b9fb11`. Those ODE
> values are now **frozen** in [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json)
> and serve as a fixed regression datum. What OmniSim no longer has is a **second in-engine
> path** to tell a plumbing bug from a solver bug — the oracle, analytic ground truth, is
> untouched. Full scope: [correctness-scope.md](../benchmarks/correctness-scope.md).

XPBD, the former default solver, was **removed on 2026-08-07** (one day after the default flipped to MuJoCo). A world still declaring `"xpbd"` gets the parser's invalid-value warning and the default. Anything you read about XPBD substep behaviour, XPBD joint targets, or `SolverXPBD` gains is history.

## What went away with ODE

**Removed outright** — these are gone from the node schema and the engine:

- **The `Fluid` and `ImmersionProperties` nodes.** Buoyancy, Archimedes' thrust and fluid drag are **not simulated**. `Solid.immersionProperties` is gone from the schema too, so a legacy world still declaring it gets a "Skipped unknown field" **ERROR**, which takes a headless run's exit code to 1 — this one does not fail quietly.
- **The ODE physics-plugin API** (`webots_physics_init` / `webots_physics_collide` / `webots_physics_step` and the rest of the callback set). `WorldInfo.physics` still parses so legacy worlds load, but any value other than `"<none>"` is ignored with one parse-time warning.

**Still parse, so old worlds load, but are not read** — do not tune with these and do not tell a user they do anything:

- `WorldInfo.CFM`, `WorldInfo.ERP` — ODE global solver parameters.
- `WorldInfo.broadphase` — an ODE broadphase selector (never fully wired even under ODE).
- `WorldInfo.physicsDisableTime`, `physicsDisableLinearThreshold`, `physicsDisableAngularThreshold` — ODE's auto-disable ("body sleep") knobs. **Newton has no body sleep**, and `OmSolidMerger::setOdeAutoDisable()` is now an empty function.
- `WorldInfo.contactProperties` / the `ContactProperties` node, including `coulombFriction`, `bounce`, `softCFM`, `softERP`. **Use `WorldInfo.newtonGroundMu` / `newtonContactKe` / `newtonContactKd` instead.** The engine emits one warning when a world declares `contactProperties` without a `newtonGroundMu`.
- `Physics.damping` and the `Damping` node — `OmSolidMerger::setOdeDamping()` is now an empty function; damping is not plumbed to Newton.

## What works on Newton today

- **Articulated rigid-body dynamics** — bodies plus `HingeJoint` / `SliderJoint`, position- and velocity-controlled motors, `setForce` / `setTorque`. The public motor API is backend-agnostic, so ordinary controllers work unchanged.
- **URDF import** — `URDFRobot` builds a Newton articulation directly.
- **Sensor reads** — Accelerometer, GPS, Gyro, InertialUnit read body state through `OmPhysicsBackend` and return real Newton dynamics.
- **Mesh collision, natively** (`add_shape_mesh`) — not an AABB approximation. The old conservative "mesh forces the articulation to ODE" routing survives only behind `OMNISIM_NEWTON_MESH_TO_ODE`, which now routes to nothing; leave it unset.
- **Static colliders** — floors, tables and walls are solid by default (since 2026-08-07). Before that flip a static collider collided with *nothing*: measured, a ball dropped onto a floor whose top is at z = 0.55 passed through and settled at **z = 0.0996** on an implicit z = 0 plane that was not in the file; with statics on it rests at **0.6496**, exactly box-top + radius.
- **Native contact readback** — `getContactPoints`, `/sim/contacts` and `/sim/grips` work by default (since 2026-08-07). Before that flip every Newton world was contact-blind: measured 1008 contacts on ODE vs **0** on Newton for the same scene.
- **External body force** — a queued wrench is written into `state.body_f` after `clear_forces()` each substep and cleared at end of tick, i.e. ODE `addBodyForce` semantics: the controller must re-apply every tick. Verified: a +20 N push lifts a 1 kg Newton box from 0.50 m to 5.44 m.
- **`WorldInfo.coordinateSystem`** — reaches the solver as of `c77cbe98`. ⚠️ Before that commit, **all 210 `NUE` worlds in the tree had gravity projected to zero and never fell**, masked by ODE still driving them. NUE and EUN worlds work now.
- **Native inertia** — `fe35be64` replaced ODE's `dMass` integrator; primitive inertia tensors are **bitwise identical** to the frozen ODE values.
- **Raycast sensors** — a native `mj_ray` service, **default on**, answers
  `DistanceSensor`, `Receiver`, `LightSensor` and, since 2026-08-08, `Radar`
  and Camera-recognition **occlusion**. The last two share `OmObjectDetection`,
  whose ray carrier was the ODE ray geom; until it was rebuilt as plain
  `{start, direction, length}` members, its ray list was permanently empty and
  every target read as unoccluded. A service that cannot answer now returns
  false — the caller keeps its previous verdicts and warns once — rather than
  reporting a clear line of sight it never tested.
  ⚠️ **Remaining gap IN THE DEFAULT: a node DELETED at runtime is never
  removed from the model** — and this is not ray-only, it is **contacts too
  (measured)**. A box resting on an elevated floor stayed at z=0.5999 for
  61,440 steps after that floor was deleted, with the engine still listing the
  deleted body at its authored pose. A deleted wall still stops a robot,
  silently. In default mode, reload the world after removing collidable nodes.
  Measured on one static Solid under the default CPU solver; robots and
  `mujoco_warp` are unmeasured. ✅ **Since 2026-09-01 (`88487d988`, W1.7)
  there is an opt-in mid-run fix: a physics rebuild** —
  `POST /sim/rebuild_physics`, or `{"physics": "rebuild"}` on
  `/scene/spawn` / `/scene/delete` — which tears down the Newton world and
  re-registers the whole scene at its current poses, so deleted nodes
  genuinely stop colliding and runtime-spawned nodes gain physics. Measured
  97–267 ms; refused (`409 REBUILD_REFUSED`) on Cloth/SoftBody/GranularBed
  worlds, and engaged `Connector`/`VacuumGripper` welds are dropped with a
  loud warning. The default behaviour is unchanged (PROTOCOL.md §7.36).

  ⚠️ **SCOPED TO THE SOLVER, and the scope is the whole point: under
  `newtonSolver "mujoco_warp"` the rays see the scene FROZEN AT ITS BUILD-TIME
  POSE.** Source, re-verified 2026-08-13: `raycast_batch` serves off
  `solver.mj_data` ([`omnisim_newton_runtime.py:964`](../../src/omnisim/physics/omnisim_newton_runtime.py))
  with no CPU/GPU gate — but newton's `SolverMuJoCo.step()` writes `mj_data`
  only on its `use_mujoco_cpu` branch; the warp branch steps `mjw_data` on the
  GPU and never touches it (vendored `newton/_src/solvers/mujoco/solver_mujoco.py:3266-3288`).
  The one `get_data_into(self.mj_data, …)` that would sync it back sits in the
  MuJoCo-viewer render path (`:7220`), which OmniSim never enters, and
  `_refresh_mj_cartesian` is explicitly CPU-only. So `mj_data` keeps the state
  it was constructed with (`_update_mjc_data` + `mj_forward`, `:5498,:5504`):
  the authored t = 0 scene.

  **Two measurements look like they contradict each other here, and they do
  not.** What differs is *which side moves* — because the ray's ORIGIN never
  comes from `mj_data` at all. `OmDistanceSensor` builds origin and direction
  from `matrix()`, the engine's own scene-graph transform, refreshed from the
  Newton state readback every tick on **both** paths
  ([`OmDistanceSensor.cpp:560-568`](../../src/omnisim/nodes/OmDistanceSensor.cpp)).

  | measurement | what moves | result |
  |---|---|---|
  | the original, recorded during raycast bring-up (**date not recorded in this tree**) | the sensor's **TARGET** | `DistanceSensor` pinned at **0.75 m forever**, while the same world on CPU tracked 0.75 → 3.9 → 1.3 |
  | ladder0 rung 6, **2026-08-12**, sidecar-verified `solver: MuJoCo (mujoco_warp)`, `device: cuda:0` | the **SENSOR**, against a static wall | **tracks to 2e-05 m** — 1253 distinct readings over 2000 samples |

  Both are correct. A static geom's t = 0 pose *is* its pose at every t, so a
  moving sensor against static geometry reads correctly off a frozen scene;
  a target that moves is read at its authored pose for ever. ⚠️ The decisive
  differential — a **moving target** under `mujoco_warp` — **has not been
  run**, so this reconciliation is a source proof plus two mutually consistent
  measurements, not a measurement of the failure mode itself. Until it is:
  do not report GPU raycast sensors as working, do not report them as frozen.
  The rule is **static targets only**, and it applies to `Receiver`,
  `LightSensor`, `Radar` and Camera-recognition occlusion for the same reason,
  none of which has been measured on the GPU path at all.

  **The architecture underneath, because it is what gets re-cited wrongly:
  OmniSim's sensors live ENTIRELY OUTSIDE the MuJoCo model.** They are served
  by body readback plus this `mj_ray` service; the engine emits no MJCF
  `<sensor>` element at all — `grep add_sensor src/` returns nothing (checked
  2026-08-13). That is also why the frozen-`mj_data` scope above cannot be
  fixed by "just letting MuJoCo compute the sensors": there is no sensor in
  the model to compute.

  ⚠️ **A companion claim — that a batched training env therefore reports
  `nsensor = 0, ncam = 0` — has circulated as though it were a measurement.
  It is not one.** Nothing in this tree has ever printed those counters:
  `OMNISIM_NEWTON_DUMP_MJMODEL` emits `nq / nv / nbody / ngeom / nu /
  nexclude` and stops. The source reading above **is** established, and it is
  the stronger statement anyway — it names the mechanism instead of a symptom
  — but the counters themselves are unmeasured, so quote the mechanism, not
  the numbers. Making them measurable is a one-line change to that dump
  (append `nsensor=%d ncam=%d nlight=%d`; all three exist on `MjModel` in the
  bundled mujoco 3.8.1), and `lane1/translation_audit.py` harvests that header
  line with a generic `(\w+)=(\d+)` sweep, so it would pick them up with **no
  parser change**.
- **Welds** — `Connector` and `VacuumGripper` attach natively, **default on**.
- **Kinematic (mocap) bodies** — a joint endpoint with no `Physics` node registers as a native kinematic collider, **default on**, and the engine animates it as before.
- **Friction grasping** — a two-finger pinch holds. It is not automatic; read [friction-grasp.md](friction-grasp.md) before trying.

## Known gaps — what does NOT work yet

Do not report any of these as working, and do not build a demo that depends on one.

- ✅ **RESOLVED 2026-08-17 — motorised `BallJoint` and `Hinge2Joint` DO actuate.** This entry said they did not, and that `OMNISIM_NEWTON_BALL_HINGE2=1` "does not work either". Both were true when written and both were measured against the **then-vendored newton 1.2.0**: the constraint registered, the motors were accepted and silently ignored, and axis 1 read 0.0000 rad when commanded 0.4. The defect was correctly attributed below us — to newton's d6 → MuJoCo actuator mapping for multi-DoF position control, a path upstream's own suite never exercised through `SolverMuJoCo`. The `b56be84a0` upgrade to **newton 1.5.0** fixed it. Re-measured on the current binary, the hinge2 arm of `tests/test_newton_ball_hinge2.py` went `xfail` → **XPASS**: both axes reach their commanded angle inside 0.05 rad, axis 1 does not drift when only axis 2 is re-commanded, and axis 2 travels its full 0.6 rad. **The gate now defaults ON**; `OMNISIM_NEWTON_BALL_HINGE2=0` is the exact-revert hatch back to the passive constraint, and the test asserts it still reverts. Worlds unblocked: the `motor2` / `motor3` / `muscle` / `gyro` samples and the api joint worlds. ⚠️ **One caveat survives:** the ball element is emitted `limited: False`, so per-axis `min/maxStop` on a `BallJointParameters` are **not enforced** — a ball joint swings past its authored stops. Hinge2 axes *are* limited.
- **`TouchSensor` reads nothing — in BOTH types, from ONE mechanism.** ⚠️ This entry used to read *"Bumper-type sensors work; force and force-3d do not"*, and the "Bumper `TouchSensor` — contact reads are native, default on" line in the works list above said the same thing. **Both were wrong**, and the pairing was actively harmful: it sent anyone blocked on the force type to the bumper as the workaround.

  Measured 2026-08-10 (OmniBench lane 4 `device.touch_bumper` / `device.touch_force`, build `df62067d7`, machine `9722d23d12a3`, CPU `mj_step`): the bumper read **`max=0` across all 750 samples**; the force type read **0.000 N** under a resting 2 kg body whose weight is **19.62 N**.

  The mechanism is neither device. **The `TouchSensor`'s own `boundingObject` never becomes a Newton collider**, so the parent body takes the contact and the sensor's geometry touches nothing. The probe separates that from a readout defect *geometrically*, which is stronger than any sensor value: its pad protrudes 10 mm below the body, so pad contact rests the prober at **z=0.66** and body contact at **0.65** — measured **0.6499**. The 0 N is therefore a *consequence*, not a second, independently fixable defect. Consistent in code: `newtonBumperTouching()` refuses outright when the sensor has no Newton body of its own rather than answering with the parent's contacts ([`OmTouchSensor.cpp:248`](../../src/omnisim/nodes/OmTouchSensor.cpp)), and the un-fold that would give it one does not happen — so `OMNISIM_NEWTON_BUMPER` (default ON) cannot rescue it.

  ⚠️ **Keep these two apart: contact detection works; the contact DEVICE does not.** Contact *queries* are a different code path and are fine — `getContactPoints`, `/sim/contacts` and `/sim/grips` all read native Newton contacts, and lane 4 scores that row WORKS on the same build. **So a gripper cannot verify a grasp with a `TouchSensor`, and a foot cannot verify ground contact with one.** Prove a grasp geometrically instead — the part is airborne and tracks the gripper ([friction-grasp.md](friction-grasp.md) asks for that anyway, for an independent reason) — and use `getContactPoints` / `/sim/contacts` for everything else.
- **A loop-closing `SolidReference` kills physics for the ENTIRE WORLD — and since 2026-08-16 a bare `run-headless` FAILS on it instead of printing PASS.** Newton/MuJoCo is a tree-articulation solver and OmniSim has no equality-constraint substitute, so a second joint arriving at a Solid that already has a joint parent is a malformed articulation, not a loop. Measured 2026-08-13 (tree `227b35c36`, binary sha256[:16] `1b82affcd3956d95`, machine `9722d23d12a3`, `newtonSolver "mujoco"`, device `cpu`): the loop joint **does** reach the builder — the engine logs `hinge joint 0 (parent=body 2, child=body 3)` and `hinge joint 1 (parent=body 2, child=body 3)` — and `SolverMuJoCo` construction then dies with `ValueError: Multiple joints lead to body 3` (newton `_src/utils/topology.py:47`, via `solver_mujoco.py:4468`). With no solver to substitute there is **no Newton world at all**: no `.newton.json` sidecar, no `[OmNewtonBackend] step` line. ⚠️ **The blast radius is the whole scene.** An unrelated 1 kg box two metres from the robot, released at z=3.0, read **exactly 3.0 after 1000 steps**; delete the loop joint and change nothing else and it settles at **0.649892 m**. ✅ **The SILENT half is fixed (2026-08-16) — the physics half is not.** The finalize raise used to go through `reportPyError()` at WARNING, so the run read `0 errors, 4254 warnings … PASS`, exit 0 — and `--fail-on-runaway` passed too, because a body frozen at its authored pose looks exactly like one still legally mid-air — leaving `--fail-on-warning` as the only lane that caught it. It now goes through `reportPyErrorFatal()` at **ERROR**, leading with the words `THIS WORLD HAS NO PHYSICS` and carrying the Python exception, so a bare `run-headless` FAILs: re-measured on the same reproducer, `1 errors, 1 warnings … FAIL`, exit 1, with the control world still PASS at exit 0. Latched to once per world (a failed finalize never closes for build, so the engine retries every tick — that retry produced the 4254 warnings). The harness classifies the line as `NEWTON_WORLD_NOT_BUILT`. ⚠️ Only the REPORTING changed: the world still gets no physics, and `--fail-on-runaway` still cannot see this class. ⚠️ **`Body N has multiple parents in this articulation` IS a real newton error, and an earlier edition of this bullet wrongly denied it.** It said the string "lives only in a source comment about a different, already-fixed eager-add ordering bug" — half right (that fixed bug is real, described at `omnisim_newton_runtime.py:2345`), and misleading: newton **raises** that exact string from its own `newton/_src/sim/builder.py` (vendored bundle, not a repo path), and a genuine loop produces it. Measured 2026-08-16 on newton 1.5.0, both wordings from one runtime: a flat two-hinge loop raised `ValueError: Multiple joints lead to body 3` (from `topological_sort`), while the shipped `coupled_motors.omniworld` raised `Body 8 has multiple parents in this articulation: 7 and 9`. **Match on either.** Full derivation, guard-by-guard: [solidreference.md](../reference/solidreference.md). `solidName "<static environment>"` is unaffected — that is a tree edge.
- **Joint-space force** (`control.joint_f`) is not wired. Body-space force works (above); a joint-force-driven world has no path today.
- **AMotor / LMotor and explicit fixed joints** are not registered by Newton, so an articulation containing one is routed to `"ode"` by the capability gate — which now means it is **not simulated**, and under Newton enforcement the world is refused with a FATAL naming the Solid and the reason. That FATAL is the good outcome: the alternative is a silently half-simulated world.
- **Contact sound** produces nothing.
- **The contact-points GUI overlay** produces nothing. An empty overlay is not evidence that nothing is touching — use `/sim/contacts` or `getContactPoints`.
- **Damage-system parity** was validated against ODE to *practical* parity (within the `damage_events_diff` 10× tolerance): a 50/30 head-on gave 58 Newton vs 39 ODE events with `newtonSubsteps 4` + `OMNISIM_DAMAGE_VEL_SMOOTH=5`. With ODE gone there is nothing left to compare against, so that tolerance is now a historical datum rather than a live gate.

## The capability gate

A Solid on the `"auto"` default resolves to Newton **only if** its whole articulation uses features Newton can register faithfully. The gate is `OmSolid::articulationNewtonCapable` in [`src/omnisim/nodes/OmSolid.cpp`](../../src/omnisim/nodes/OmSolid.cpp) — **read the allow-list in the source if the answer matters**, because it is the thing that decides and this page will drift.

The gate was designed when routing an articulation to ODE was a *safe* conservative choice. **It is not safe any more**: the ODE branch simulates nothing. That is why Newton enforcement turns a gate-triggered downgrade into a **FATAL** rather than a quiet fall-back, and why the gate's remaining disqualifiers (AMotor / LMotor / explicit fixed joints) read as engine gaps rather than as routing decisions. Two former disqualifiers have been lifted: **mesh** collision is native, and **kinematic** endpoints register as native mocap bodies.

`OMNISIM_AUTO_NO_CAPABILITY_GATE=1` forces `auto`→Newton even for a disqualified articulation. It is an A/B-test lever, and what you get is undefined behaviour rather than a fall-back.

## Tuning knobs (`WorldInfo` fields; env vars still override)

These fields fold the former launch env vars into the world file, so a demo world is self-contained.

- `newtonSubsteps N` (default `1`) — split each physics tick into N solver sub-steps. Shrinks integrator truncation (measured: flips OmniBench lane-1 T4 energy and T7 angular-momentum in Newton's favour at N=4), but is **not free** — it degraded T2 sliding accuracy 12× in the same sweep, most likely because MuJoCo's `solref` / `solimp` are not rescaled. The RL deploy stack runs N=8. On `mujoco_warp` the CUDA-graph replay needs an **even** N. (Env: `OMNISIM_NEWTON_SUBSTEPS`.)
- `newtonGroundMu` (default `0` = the built-in 1.0) — **this is the friction field**, not `ContactProperties.coulombFriction`. Raise it for legged robots (a friction probe measured sphere feet sliding ~1 m while merely standing at mu = 1.0, and planting inside 4 cm at mu = 2.0); a two-finger friction pinch needed mu = 3. Above ~6 the floor itself destabilises even at `basicTimeStep 8`. (Env: `OMNISIM_NEWTON_GROUND_MU`.)
- `newtonContactKe` / `newtonContactKd` (defaults `0` = built-in 2500 / 100) — contact stiffness and damping. **Raise `ke`** (8000 measured) so gripper fingers stop *at* an object's face instead of over-penetrating and squirting it out; lower it to soften contact for dense layers of small parts. (Env: `OMNISIM_NEWTON_CONTACT_KE` / `_KD`.)
- `newtonIterations` / `newtonLsIterations` (defaults `0` = the solver's own) — raise for hard frictional contact that must HOLD rather than creep (150 / 50 measured for a two-finger pinch). (Env: `OMNISIM_NEWTON_ITERS` / `_LS_ITERS`.)
- `newtonCone "elliptic"` + `newtonImpratio 10` (defaults `""` / `0` = MuJoCo stock pyramidal, impratio 1) — MuJoCo's stock pyramidal cone creeps near the cone boundary: OmniBench T2 measured **181 mm** of pseudo-slip below the static-friction transition at pyramidal versus **0.6 mm** at elliptic + impratio 10. The global default stays MuJoCo stock deliberately — flipping it would change contact physics for every world and break train==deploy bit-exactness of shipped champions; a flip is gated on champion re-verification. (Env: `OMNISIM_NEWTON_CONE` / `OMNISIM_NEWTON_IMPRATIO`.)
- `newtonStatics TRUE` (default: statics are **on** since 2026-08-07; the field forces them on and always wins over the revert hatch) — registers top-level static colliders as mass-0 Newton bodies. `OMNISIM_NEWTON_STATICS=0` is the exact-revert hatch and reproduces the pre-flip build. ⚠️ The field's schema comment in `resources/nodes/WorldInfo.wrl` still describes the pre-flip default; the flip is the current behaviour.
- `newtonRobotColliders TRUE` (default `FALSE`) — gives a robot-wrapper body (a `URDFRobot` chassis Solid) its own `boundingObject` as a Newton collider, so the robot **body** — not just its wheels or feet — collides with scene geometry. Off by default because a chassis envelope box that engulfs the wheel space pins the body and starves the wheels of load. (Env: `OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE`.)
- `newtonCompoundColliders TRUE` (default `FALSE`) — registers **every** collider in a compound `boundingObject` (a `Group` of offset primitives on one rigid body — a bin's floor plus four walls, an L-tube of two boxes) rather than only the first child. (Env: `OMNISIM_NEWTON_COMPOUND_COLLIDERS`.)
- `newtonNjmax N` + `newtonNconmax N` (defaults `0` / `0` = the engine's built-in 256 caps; `-1` = newton's own auto-estimate, **not recommended**) — MuJoCo constraint-row and contact buffer caps. **A fleet world has to raise these.** Measured on a 10-Husky flat-ground `mujoco_warp` world: 8 wheel–ground contacts per Husky × 4 rows each on a pyramidal cone = **32 constraint rows per robot**, so ten peak at `nefc` 320 / `ncon` 80 and blow past the 256 default from tick 4 (≈ 32 rows per wheeled robot, so 256 runs out at 9). Overflow is **silent**: `mujoco_warp` truncates the constraint vector and its only native warning is a per-tick `wp.printf` from inside the solver kernel, which floods where stdout is captured and is discarded outright on Windows (`omnisim-bin` is a GUI-subsystem binary). The engine therefore samples the counters itself on the `mujoco_warp` path and logs one `CONSTRAINT BUFFER OVERFLOW` warning; `OMNISIM_NEWTON_CONSTRAINT_STATS=<every N steps>` traces every new peak into `.build_tmp/newton_solver.log`. Prefer `0`, or an explicit positive value with headroom (512 for ten 4WD robots) — sizing exactly to the measured peak is itself a trap ([determinism-scope.md](../benchmarks/determinism-scope.md) §3: 8.81 m of divergence at njmax = 320). (Env: `OMNISIM_NEWTON_NJMAX` / `OMNISIM_NEWTON_NCONMAX`.)

The **base-divergence guard is on by default**: if Newton's base state goes non-finite or beyond `OMNISIM_NEWTON_BASE_GUARD_MAX` metres (1000), the articulation freezes at its last good pose (logged once) rather than feeding garbage to controllers. It is a strict no-op for any physically valid state. Disable with `OMNISIM_NEWTON_BASE_GUARD=0`.

⚠️ **`finalizeWorld()` costs 1.9–4.9 s, inside the physics timing bracket.** Any short-run per-step average that includes world load is badly inflated — difference two windows.

## Build and runtime requirements

Newton is compiled in by default (`OMNISIM_WITH_NEWTON ?= ON`). A plain build gives you Newton:

```bash
make -C src/omnisim release
```

⚠️ **`OMNISIM_WITH_NEWTON=OFF` no longer produces "a pure-ODE binary"** — the Makefile comment still says so, and it is stale. With `src/ode` deleted, that flag now yields a binary with **no physics implementation at all**. Do not use it. (`OMNISIM_WITH_ODE=ON` is refused outright with an error message.)

The backend needs a Python runtime (`warp-lang`, `newton`, `mujoco`, and `mujoco-warp` for the GPU path) importable by the binary's embedded interpreter. Two ways it reaches you:

- **A packaged release bundles it.** A stock `make release` runs the bundler (`BUNDLE_NEWTON ?= 1`, idempotent), vendoring a self-contained CPython plus `warp` / `newton` / `mujoco-warp` / `usd` next to the binary (~600 MB — `warp` carries its own slim CUDA subset, so there is no separate multi-GB toolkit). The embedded interpreter resolves the bundle via a `python3XX._pth` redirect. This is a **Windows** packaging mechanism.
- **A self-built / `make debug` binary needs the runtime installed.** Run `make -C src/omnisim bundle-newton-runtime` (see [Newton runtime bundle](../developer/newton-runtime-bundle.md)), or install into the Python the binary embeds:
  ```
  pip install "newton[examples]" warp-lang mujoco
  ```
  (The upstream PyPI package was renamed from `newton-physics` to `newton` in late 2025.)

**On Linux** (supported as of v5.1) the bundler is not involved: the embedded interpreter resolves the **system** `python3`, so `pip install torch warp-lang newton mujoco mujoco-warp` into the *system* interpreter is the whole setup. It must not be a venv — the embed ignores venvs.

If the runtime is missing, the world has no dynamics. `python -m omnisim doctor` is the first check.

## Verifying that Newton actually drove a run

**Read the sidecar; don't scrape the log.** At world finalize the engine writes `<OMNISIM_LOG_PATH>.newton.json` (default `omnisim_log.txt.newton.json`) containing `{backend, solver, finalised, degraded}`. `OmLog` deletes any stale copy when it truncates the log at startup, so the file's mere presence means "Newton drove *this* run".

Caveats that have each cost a diagnosis:

- `[OmNewtonBackend] imports OK` / `FFI smoke OK` prove only that the runtime **loaded**, never that it drove the sim.
- A run too short to reach world-finalize produces **no** sidecar. That proves **nothing** about the backend — only that the run ended early. Budget `--duration ≥15` on a fast local disk, and **≥45 for cold loads on slower or virtualized disks** (WSL2, network volumes, cloud pods), where 15 s ends before finalize. Re-run longer before concluding anything.
- Log-scraping for `[OmNewtonBackend] world finalised (solver=…)` is the fallback for older binaries; a tail-only read used to miss the load-time line on a large log (fixed in `ad9fff48`).
- The HTTP harness's `GET /capabilities` still reports `backend: "ode"` for its negative cases. That label predates the deletion and no longer names a working backend — read it as "Newton did not finalize this world".

**The retired environment variables**, listed so you recognise them in old scripts and *don't* set them: `OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY`, `OMNISIM_ALLOW_ODE_FALLBACK` and `OMNISIM_REQUIRE_NEWTON`.

The first two still short-circuit backend resolution *and* make the Newton registration flush return early, so setting either yields a world with **no dynamics** — strictly worse than not setting it. This is **measured, not inferred**: on 2026-08-08, `tests/physics/worlds/contact_points.omniworld` run under `OMNISIM_FORCE_ODE=1` built no Newton world, wrote no verdict sidecar, and left the scene **frozen at its authored pose for the whole run** (recorded in [OmniBench `SPEC.md`](../../tests/benchmarks/omnibench/SPEC.md), lane 1, which also now *refuses* `--backend ode` rather than silently serving Newton under an `omnisim-ode` label). `OMNISIM_ALLOW_ODE_FALLBACK=1` disables the enforcement that would otherwise refuse a silent downgrade, reaching the same dead end. `OMNISIM_REQUIRE_NEWTON=1` is vestigial: Newton is required unconditionally. The variables have not been deleted from the source yet, which is exactly why they still bite.

## Sample worlds

- `projects/samples/demos/worlds/physics/newton_smoke_test.omniworld` — a single sphere falling onto a floor. The minimal check that the Newton runtime came up.
- `projects/samples/demos/worlds/physics/newton_husky_smoke_test.omniworld` — a single Husky drives forward.
- `projects/samples/demos/worlds/starter/friction_grasp_minimal.omniworld` — a verified-holding two-finger pinch grasp. `run-headless --duration 16`.
- `projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld` — two teams of four Huskies in a head-on collision arena.
- `projects/robot_combat/worlds/demos/newton_husky_combat_2.omniworld` — combat-style multi-Husky arena.

## Where to learn more

- [friction-grasp.md](friction-grasp.md) — making a gripper actually hold something.
- [docs/reference/worldinfo.md](../reference/worldinfo.md) — every `WorldInfo` field, including which ones no longer do anything.
- [docs/benchmarks/determinism-scope.md](../benchmarks/determinism-scope.md) — the per-configuration reproducibility scope. Every external determinism claim must match this file.
- [docs/developer/engine-migration-plan.md](../developer/engine-migration-plan.md) — the master physics plan: solver decision, architecture, phase status, measured performance, remaining work.
- Newton documentation: https://github.com/newton-physics/newton
- Warp documentation: https://nvidia.github.io/warp/
