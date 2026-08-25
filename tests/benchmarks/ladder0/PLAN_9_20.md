# ladder0 — the plan for rungs 9 and up

> **STATUS: DESIGN ONLY, and THREE OF THE ELEVEN ARE NOW BUILT.**
> No number in this file is a measurement of a rung in it. Where a number
> appears it is either (a) **derived** from mechanics and the geometry proposed
> here, or (b) **cited** from a measurement that already exists in this tree,
> with its path. Predictions about how an arm will score are labelled as
> predictions and carry a confidence. Nothing in §5 is a result.
>
> **Built (2026-08-13): determinism, scale fidelity, and agreement with
> recorded reality.** Read [`README.md`](README.md) and
> [`CONTRACT.md`](CONTRACT.md) §3c for what they actually measure — this file
> is the design that preceded them and several of its premises did not survive
> contact:
>
> | § | designed | as built |
> |---|---|---|
> | 5.1 | rung 9, drop over the **centre** cube's corner, sensitivity at `t_end` | rung 9, drop over the **pile's outer** corner (the centre placement lands squarely on a 2×2 group and amplifies nothing — measured), sensitivity over the **whole run**, and as an **amplification factor** rather than a distance |
> | 5.3 | rung 11, N ∈ {1,4,8,16,32}, `truncate_budget` fault sets `newtonNjmax` low | rung 11, N ∈ {1,4,8,16} judged and 32 a variant; the budget starve is a **variant, not a row fault**, and it works by removing the *declaration* — setting `newtonNjmax` lower cannot starve anything, because it is a floor |
> | 5.11 | rung **19**, recorded reality | rung **18** — the build brief's numbering, adopted |
> | 5.10 | rung 18, closed kinematic loop | **not built.** Referred to by name, never by number, so no built rung and no designed rung share one |
>
> The six contract amendments §3 proposes are **adopted**; `CONTRACT.md` §8
> carries them with the file and line where each now lives.

Owner: the OmniSim lane. This is a proposal against [`CONTRACT.md`](CONTRACT.md)
and [`rungs.py`](rungs.py); it changes neither. Six contract amendments are
needed before most of it can be built, and they are specified in §3 rather than
assumed.

---

## 00. Three corrections to the brief — read these before the rungs

The brief that commissioned this plan carried three engine facts. Checking them
before designing around them changed two rungs' premises and moved a third up
the build order. **All three are recorded here rather than in a footnote,
because a plan built on the briefed versions would have been wrong in ways no
green row could have shown.**

Each is stated as: what the brief said → what is actually true → the evidence →
what it changes. Two of the three make our position **worse**, not better.

---

### Correction 1 — the batched path HAS a driveable entry point, and rung 17 is FURTHER away than briefed, not closer

**The brief said:** the batched training path has `nsensor = 0, ncam = 0` — no
sensors of any modality, because OmniSim never emits MJCF `<sensor>` elements —
and rung 17 (sensor-in-the-loop batched training) is therefore *"roughly one
engine change away"*.

**What is actually true — three separate things, and they do not all point the
same way:**

**(a) A driveable batched entry point already exists, from a live `.wbt`.**
`src/omnisim/physics/omnisim_newton_runtime.py` exposes the env hook
`OMNISIM_INENGINE_PYMOD="pkg.mod:func"` (line 5212), and behind it
`_mpc_rollout_buffers(K)` (line 3184) calls
`mujoco_warp.put_data(..., nworld=K, njmax=MPC_NJMAX, nconmax=MPC_NCONMAX)`
**from the same compiled CPU `MjModel` the live solver is stepping**, while
`_mpc_seed_from_live(K)` broadcasts live state into all K worlds. It is not
theoretical: `projects/policies/training/quad_walk_recipe.py` drives it with
`QUAD_ENVS` (default 4096).

**What that entry point CAN carry:** the physics — bodies, joints, contacts,
actuation — per world, seeded from the model the live simulation is already
using. That is precisely the property the "train == deploy" claim rests on, and
it means a *driver* can request K envs and read per-env state back today.

**(b) What it CANNOT carry: sensors of any kind — and the reason is deeper than
a missing MJCF element.** OmniSim's sensors are not part of the MuJoCo model at
all. They are served **outside** it, by C++/Python readback of body state plus a
separate `mj_ray` service. `grep add_sensor src/` returns nothing; the committed
MJCF export `projects/policies/research/training/mjcf/go2_newton.xml` (184
lines) has a `<worldbody>` and an `<actuator>` and **no `<sensor>` and no
`<camera>`**. So a batched env does not have a *degraded* sensor — it has **no
sensor service at all**, because the service is a per-world CPU path the batch
never enters.

**(c) The specific figure in the brief is not a measurement.** No run in this
tree has printed `nsensor` or `ncam`: `OMNISIM_NEWTON_DUMP_MJMODEL` does not
emit those counters, so it cannot even witness them. The claim's honest status is
**CITED** (source + the committed MJCF artefact), not `MEASURED`. It is very
probably true; it has not been observed.

**What it changes — in both directions:**

* **Rung 15 (batched fidelity) moves UP the build order.** It was designed as
  blocked on an engine project; it is **arm work only** — a module registered on
  the existing hook (§7, W1). It is the one rung in the plan that enters a
  category upstream Webots structurally cannot, and it is now the cheapest way
  to get there.
* **Rung 17's premise is corrected and its cost estimate roughly inverted.**
  "Emit a `<sensor>` element" is not the fix. The fix is to give the batched path
  a sensor service at all, and there are only two shapes for it: move the sensor
  model **into** MuJoCo (and then prove parity against the existing out-of-model
  service, which is a second job), or **batch the `mj_ray` service** over K
  worlds. Both are real engine projects (§7, W2). **Sensor-in-the-loop training
  is a roadmap item, and it is a larger one than the brief believed.**
* **Rung 16 (policy parity) inherits the same gap.** Its scene consumes a range
  reading, so its sensed variant is blocked on W2. Its first honest cell is
  **proprioceptive-only** (wheel rate → wheel command, no sensed stop), which is
  a weaker claim and is labelled as one in §5.8 rather than quietly substituted.

---

### Correction 2 — `SolidReference` does not poison the articulation; the loop joint is dropped two levels before it could

**The brief said:** a loop-closing joint poisons the articulation with
`Body N has multiple parents`, and the world then runs with **no physics** while
`run-headless` prints PASS.

**What is actually true:**

**(a) The error string is real, Newton's articulation model genuinely is a
tree — and it belongs to a different code path.** The runtime's own comment
names it verbatim at `omnisim_newton_runtime.py:1422`: *"Newton rejects with
'Body N has multiple parents in this articulation'."* But it is cited there for
an **eager-add ordering bug** — adding FREE joints for yet-unseen parents, which
then conflicted when the same body turned out to be a revolute child later in
the queue — and that class was fixed by queueing every joint and topo-sorting at
`finalize()`. It is not the `SolidReference` path.

**(b) A `SolidReference` loop joint never reaches the articulation builder.**
Two independent gates, both on the C++ side:

* `OmSolid::collectSolidChildren` appends a joint's endpoint to the articulation
  walk only `if (ep && j->solidReference() == NULL)` —
  `src/omnisim/nodes/OmSolid.cpp:1549` and `:1586`;
* `OmBasicJoint::setJoint()` treats the endpoint as invalid unless the reference
  points at the static environment —
  `const bool invalidEndPoint = s == NULL && (sr == NULL || !sr->pointsToStaticEnvironment());`
  at `src/omnisim/nodes/OmBasicJoint.cpp:868`, asserted again at `:881`.

So the closing joint is **silently never registered**, two levels before Newton
could reject it. And the Newton capability gate `articulationNewtonCapable`
lists only `mesh | joint | kinematic` as refusal reasons, so **no FATAL fires
and no error is logged.**

**(c) The consequence is therefore NOT a dead world.** The rest of the scene
simulates normally; only the loop-closing constraint is missing. "Runs with no
physics" would have been a *louder* and more findable failure than what is
actually there.

**(d) Status: the mechanism is source-CITED; the behaviour is UNVERIFIED.** The
only shipped closed-loop sample, `projects/samples/devices/worlds/coupled_motors.omniworld`,
was recorded as `ERROR` — explicitly *"not as a pass"* — in
`docs/developer/roll-check.md:348`, and never produced a document. **Nobody has
measured what a closed loop actually does here**, which is most of the argument
for the rung.

**What it changes:**

* **The rung's predicted signature is rewritten.** Not "frozen world, everything
  reads zero" but **"the honest row is indistinguishable from the `no_closure`
  fault"**: the crank turns at exactly its commanded rate, the slider is free,
  the loop residual is meaningless because there is no constraint to violate.
  That is a *stronger* claim than the briefed one — it says the engine behaves
  exactly as if the feature had been deleted from the scene — and it is a
  different row to write.
* **`crank_angle` changes role.** Under the briefed mechanism it was the check
  expected to catch the defect (a dead world has no motion). Under the real one
  it is a **must-GREEN companion on the honest run**, and its job is to stop the
  red being misread as "nothing ran". The motion floor stays either way; what it
  is *for* changed.
* **The scene must close the loop between two DYNAMIC bodies, deliberately.**
  Gate (b) accepts a reference that points at the static environment, so a
  mechanism closed against the world frame may well work while the case every
  real mechanism needs does not. A rung that closed its loop against the world
  would have returned a false green.
* **It names the fix, which the briefed mechanism did not.** A closed loop
  cannot be a tree edge, so it has to be an **equality constraint outside the
  articulation** — exactly `equality connect` in MuJoCo/mujoco_warp. AGENTS.md
  records **welds (`Connector`/`VacuumGripper`) as native on the Newton path**,
  so that machinery plausibly already exists and the gap is that `SolidReference`
  is not routed to it. Status UNVERIFIED and it is the first thing to check
  (§7, W5).

---

### Correction 3 — BOTH TouchSensor types are broken, not just the force type

**The brief said:** force-type `TouchSensor` reads 0 N; bumper-type contact reads
are native and default-on. (AGENTS.md and the backend guide both still say this.)

**What is actually true:** OmniBench lane 4 **measured both dead**:

* `device.touch_bumper` — `first_finite_value = 0`, `max = 0`, over **750
  samples**;
* `device.touch_force` — mean **0 N** against an expected **19.62 N**.

**The mechanism is one defect, not two:** the sensor's own `boundingObject` is
not a collider. The probe's pad protrudes 10 mm below the robot body, and the
*body* took the contact — the rig rests at **z = 0.6499**, where a genuine pad
contact would rest at **0.66**. A sensor that is never in the contact set reports
nothing, in either units or booleans.

Evidence: `docs/benchmarks/lane4-capability-matrix.md` and
`tests/benchmarks/omnibench/lane4/capabilities.py`. **The generated matrix and
the prose docs disagree, and lane 4's own charter says the matrix is the
measurement.**

**One distinction that matters and is easy to lose:** contact *queries* are a
**different path** and they work — `getContactPoints` / `/sim/contacts` have been
native and default-on since 2026-08-07 and lane 4 records them passing. So the
honest sentence is **"contact detection works; the contact *device* does not."**

**What it changes:**

* **Rung 13's prediction changes from one red to two** (§5.5): `touch_trigger_t`
  *and* `touch_force_rest`, with the shared mechanism named. The rung becomes an
  independent second measurement of a documented contradiction, in seconds and
  newtons.
* **No rung in this plan may sense contact through a `TouchSensor`.** Contact
  must be established either through the query API or geometrically from poses.
* **⚠️ Worth passing to the rung-8 lane.** Rung 8 already proves its grasp
  *geometrically* — payload airborne, tracking the wrist, clearing the table —
  rather than through a contact device, and `CONTRACT.md` justifies that on the
  grounds that a contact read is a weaker claim than a lifted part. **That
  decision is now validated by measurement rather than merely prudent**, and the
  corollary is binding: rung 8 must **not** be "improved" by adding a
  `TouchSensor` to the pads, and neither must any successor. It would report
  nothing and the rung would read as a failed grasp.

---

## 0. What changes at rung 9, and what does not

**What does not change.** Every rule that makes rungs 0–8 worth running carries
forward unaltered:

* every expected value is derived from first principles — or, for exactly one
  rung, from an external measurement of *reality* (§3 F). Never from a running
  simulator;
* every rung carries at least one invariant over the **whole run**, because a
  steady-state window can only see steady state (rung 4's `roll_overrun`);
* for every rung, **what would still pass while it was broken** is written down
  and either closed or declared as the rung's limit;
* the arm records, `analysis.py` reduces, `rungs.py` judges. No arm computes a
  verdict, and a threshold that appears in `<sim>/` is a bug;
* the fair-defaults rule R1–R5 (`CONTRACT.md` §3b) applies to every rung from
  here on, generalised from "the friction model" to "the physical model": the
  contract owns the physics, an arm may declare only settings that change *how
  accurately its solver enforces that physics*, a declared value must be a
  budget rather than a fit, it is declared in `meta` or it did not happen, and
  the pure-defaults datum is mandatory.

**What changes is the question.**

Rungs 0–8 ask *can it do this?*, and that axis has **converged**. Three mature
engines solving the same Newtonian mechanics agree to microseconds — rung 2's
fall interval matched to 4 µs across three independently authored scenes. That
is the correct outcome and worth stating plainly: **nothing in 0–8
differentiates, and nothing in 0–8 should.** A ladder that discriminated at
rung 2 would mean somebody had a bug.

Rungs 9 and up ask three different questions, one per tier:

| tier | question | why a mature engine can still fail |
|---|---|---|
| **C** (9–11) | does it stay true under **stress**? | correctness at 8 s with one robot is not correctness at 600 s with 32. Nothing in this tree runs physics for minutes; the longest physics horizon found anywhere is 32 s (§2, F13). |
| **D** (12–14) | can it model a real robot **system**? | a solver is not a simulator. Import, device models and inter-robot messaging are the layer *around* the solver, and that layer is where our own measured defects live. |
| **E** (15–19) | the **frontier** | batched rollout, policy execution, closed kinematic loops, agreement with recorded reality. Two of the five are rungs we are predicted to fail. |

### 0.1 The strategic finding this plan is built on, stated before the rungs

The brief put the constraint plainly: *against upstream Webots our edge is GPU
scale and in-engine training; against MuJoCo it is the robot / device / scene
model; MuJoCo is also our own solver, so we can never beat it on fidelity.*
Working through the rungs one at a time produced a sharper version, and it
determines which are worth building:

1. **On correctness assertions alone we cannot beat upstream Webots anywhere in
   tiers C or D.** Webots expresses every one of rungs 9–14 — it has the device
   model, the loop-closure primitive, the multi-robot scene, and a
   single-threaded deterministic solver. It would be *slower*, and **this ladder
   does not judge speed** (§4.1). Any rung whose intended headline is "we beat
   Webots at N robots" is a throughput claim wearing a correctness costume.
2. **The only categories upstream Webots structurally cannot enter are batched
   rollout and in-engine policy execution** (rungs 15–17). That is where
   `NOT_EXPRESSIBLE` against Webots is honest — and it is also where **we are
   predicted to fail rung 17**, so the tier is a roadmap, not a victory lap.
3. **Against MuJoCo the only honest claim available to us is
   non-degradation**, with a precise form: *our translation layer preserves what
   the solver gave us.* Rungs 12, 13 and 19 are that claim three ways — import
   fidelity, device fidelity, and agreement with recorded reality measured
   against the bare engine we embed. We can never win those rows; we can only
   fail to lose them, and a row that can only be lost is exactly the row worth
   publishing about your own vendor.
4. **The composite rung the brief hoped for — "requires both properties
   simultaneously" — does not exist as a *correctness* rung.** Twenty
   radio-coordinated robots is expressible on Webots and correct there. The
   composite only exists when the second property is *fidelity on a path Webots
   does not have*: rung 11's `mujoco_warp` variant, rung 15, rung 17. §4.2
   records the composite I considered and rejected rather than shipping it.

None of that shrinks the plan. It stops the ladder being asked to produce
marketing, and lets it produce what it is good at: **a small number of rows that
can go red on our own name.**

### 0.2 Mapping from the brief's proposed 9–20

Renumbered; two dropped, one split, one added. The mapping is here so nothing
goes missing silently.

| brief | this plan | disposition |
|---|---|---|
| 9 determinism | **9** | kept, plus a **sensitivity control** and a contact-rich scene — determinism is trivially satisfied by a frozen world, and a *sparse* scene cannot refute the GPU refutation either (§5.1) |
| 10 long-horizon | **10** | kept; the conservative sub-scene is a **passive pendulum read at its turning points**, which removes the velocity sensor no arm has (§5.2) |
| 11 scale fidelity | **11** | kept, but **bit-identity across N is dropped as unphysical** (§4.5) and replaced with "the same analytic tolerance at every N" (§5.3) |
| 12 throughput at matched fidelity | **rejected** | no analytic ground truth, machine-dependent; becomes a *reported, unjudged* column (§4.1) |
| 13 URDF fleet | **12** import fidelity | reshaped: the fleet part is rung 11; what is left is the claim that matters — *does importing a robot change its physics* (§5.4) |
| 14 heterogeneous devices | **13** + **14** | split: 13 is the inertial/contact **sensor suite** (expressible everywhere ⇒ a fidelity claim); 14 is **radio coordination** (where `NOT_EXPRESSIBLE` is honest) (§5.5, §5.6) |
| 15 batched rollout + fidelity gate | **15** | kept, reshaped so it needs no cross-run comparison — and it turns out to be **buildable today** (§5.7) |
| 16 train → deploy parity | **16** | reshaped to **policy-execution parity**: one fixed artefact, two execution paths. No training run needed to measure the property that matters (§5.8) |
| 17 sensor-in-the-loop batched | **17** | kept, predicted FAILING, with the mechanism corrected (§5.9) |
| 18 sim-to-real | **19** | kept as the **cross-arm** extension of `omnibench/lane1r`, not a rebuild, with a pass criterion that cannot be "we win" (§5.11) |
| 19 closed kinematic loops | **18** | kept, predicted FAILING, promoted to **build first** (§5.10) |
| 20 agent-authored | **rejected** | belongs in AgentBench, which has the freeze discipline for it; the ladder should **export its graders** there instead (§4.3) |
| — | candidate | **controller isolation** — designed in §4.4, deliberately not scheduled |

---

## 1. The shape of a rung from 9 up

Every section in §5 carries the same eight fields, and a rung is not ready to
build until all eight are written:

1. **the claim** — one sentence, in physical terms, that a practitioner would
   recognise *without knowing which simulator can run it*;
2. **the scene** — one contract-owned scene family;
3. **the ground truth** — analytic with its derivation, or external-measured
   (§3 F);
4. **the assertions** — name, quantity, expected value, tolerance **and the
   tolerance's derivation**;
5. **the whole-run invariant** — which assertion, and what it sees that a window
   cannot;
6. **what would still pass while broken** — the hole, closed or declared;
7. **the fault battery**, in the `selftest.LIVE_FAULTS` shape
   `(rung, fault, must_red, must_green)`;
8. **expected per-arm verdict, cost, dependencies.**

---

## 2. The measured facts this plan leans on

Each of these is why a rung below is shaped the way it is. Status uses
`buildbench`'s vocabulary: `MEASURED` (a run happened, machine named), `CITED`
(source or doc read), `UNVERIFIED`.

| # | fact | status | where |
|---|---|---|---|
| **F1** | **Determinism is bitwise on CPU `mj_step` and refuted on `mujoco_warp`.** CPU: 3 pairs bitwise on a 10-robot + 64-box-tower world (336 contacts, 1344 `nefc`, ten live controllers), 1 pair on a 64-box pile. GPU: **24 same-config cold pairs across six scenes, 0 bitwise** — a ring at 80/320 contacts diverges 4.15e-5–7.10e-5 m by 120 steps and **9.152 m by 1000**; the 10-robot world 0.808–1.527 m. Mechanism is cited from mujoco_warp's own source: `pairid = wp.atomic_add(...)` in `collision_driver.py:343`, ~130 `atomic_add` sites. **⚠️ Counter-note (2026-08-09): a single-contact scene reproduced bit-identical 3/3 cold on `mujoco_warp`.** | MEASURED, machine `9722d23d12a3` | `docs/benchmarks/determinism-scope.md`; harness `omnibench/lane3/determinism.py` |
| **F2** | **The `njmax` cliff, and the sizing trap next to it.** 32 constraint rows per 4WD robot; a 10-Husky world peaks at `nefc` 320 / `ncon` 80 against a built-in cap of **256, from tick 4**, and the only warning is an in-kernel `wp.printf` discarded entirely on Windows. Separately: **setting `newtonNjmax` to the measured peak (320) moves results 8.81 m** versus every other size, with a 1.71 m run-to-run spread, while 512 / 2048 / 4096 agree to 1e-4. | MEASURED | `docs/guide/newton-physics-backend.md`; `determinism-scope.md` §3 |
| **F3** | **Lane 4b tried to reproduce the cliff and could not.** Peak `nefc` tracked 32·N *exactly* (32/64/128/256/384/512 at N = 1/2/4/8/12/16) — confirming the rule of thumb — **but the allocated cap tracked it too**, and three attempts to force an overflow all read `384/384` at N = 12. The row reports **`cliff_detector_validated: false`** and refuses to call the green a pass. Its rovers are also below realtime at N = 1 (6.721 ms/step). Scope: generated rovers, **not** the 10-Husky world, so it neither confirms nor refutes the ~9-robot threshold. | MEASURED | `omnibench/lane4/run_envelope.py`, `results/envelope.jsonl` |
| **F4** | Every Newton joint is built `POSITION_VELOCITY` with `targetKe = effortLimit * 10`, so a "force" command is a live PD servo anchored at the last `setPosition`. | CITED (source) | `ladder0/omnisim/engine_facts.py` docstring |
| **F5** | **Both TouchSensor types are BROKEN, and the docs disagree.** `device.touch_bumper` read `first_finite_value = 0, max = 0` over 750 samples; `device.touch_force` read 0 N against an expected 19.62 N. Mechanism: the sensor's `boundingObject` is not a collider — the pad protrudes 10 mm below the body yet the *body* took the contact (rest z 0.6499, where a pad contact would rest at 0.66). The backend guide still claims bumper reads are native and default-on. **Cite the matrix, not the guide.** | MEASURED | `docs/benchmarks/lane4-capability-matrix.md`; `omnibench/lane4/capabilities.py` |
| **F6** | Devices that **do** work, measured: `DistanceSensor` (1.9 m vs 1.9, abs err 0), `Lidar` (min range 2.9008 vs 2.9), **`Emitter`/`Receiver`** (a packet from a second robot on channel 1 arrived), GPS, IMU, Gyro, contact-points API. Lane 4 overall: 44 probes, **26 PASS / 4 PARTIAL / 9 BROKEN / 5 absent**. | MEASURED | same |
| **F7** | **Camera recognition and Radar occlusion are CITED as fixed, never measured** — both were rebuilt on the `mj_ray` service on 2026-08-08 after the ODE ray carrier was deleted (until then *every target read as unoccluded*), and **neither has a lane-4 probe.** | CITED | `docs/guide/newton-physics-backend.md` |
| **F8** | A node **deleted at runtime keeps colliding**: a 0.2 m box held up for 61,440 steps by a floor that had been deleted. Lane 4 confirms it as `phenomenon.runtime_node_deletion: BROKEN`. | MEASURED | `AGENTS.md`; lane 4 matrix |
| **F9** | **A declared friction may not reach the contact.** Lane 4's `phenomenon.friction_declared_in_world` is BROKEN: µ = 2 declared *and audited into the model*, and the box still slid 0.833 m on a 55° ramp. | MEASURED | lane 4 matrix |
| **F10** | **A driveable batched-rollout path EXISTS from a live `.wbt`.** `omnisim_newton_runtime.py:3184` `_mpc_rollout_buffers(K)` calls `mujoco_warp.put_data(..., nworld=K, njmax=MPC_NJMAX, nconmax=MPC_NCONMAX)` **from the same compiled CPU MjModel the live solver steps**, and `_mpc_seed_from_live(K)` broadcasts live state into K worlds. The entry point is the env hook `OMNISIM_INENGINE_PYMOD="pkg.mod:func"` (line 5212). Consumers: `quad_walk_recipe.py` (`QUAD_ENVS`, default 4096). | CITED (source) + MEASURED (in use) | `src/omnisim/physics/omnisim_newton_runtime.py` |
| **F11** | **OmniSim's sensors are served OUTSIDE MuJoCo** — by C++/Python readback of body state plus an `mj_ray` service — which is why the batched model carries none. `grep add_sensor src/` returns nothing; the committed MJCF export `projects/policies/research/training/mjcf/go2_newton.xml` (184 lines) has `<worldbody>` and `<actuator>` and **no `<sensor>` and no `<camera>`**; `OMNISIM_NEWTON_DUMP_MJMODEL` does not print `nsensor`/`ncam` at all, so it cannot even witness them. **"nsensor = 0, ncam = 0" is CITED, never MEASURED — no run has printed those counters.** | CITED | same, plus the MJCF artefact |
| **F12** | **A closed loop between two DYNAMIC bodies is silently not constrained under Newton.** `OmSolid::collectSolidChildren` (`OmSolid.cpp:1549,1586`) appends a joint's endpoint to the articulation only `if (j->solidReference() == NULL)`, and `OmBasicJoint::setJoint()` (`OmBasicJoint.cpp:863-878`) returns false unless the reference `pointsToStaticEnvironment()`. The Newton capability gate `articulationNewtonCapable` lists only `mesh \| joint \| kinematic`, so **no FATAL fires** and the rest of the scene simulates normally. ⚠️ Newton's articulation genuinely **is** a tree and rejects a second parent — the runtime quotes the error verbatim (`omnisim_newton_runtime.py:1422`) — **but that is cited there for an eager-add ordering bug, fixed by topo-sorting at `finalize()`, and it is not this path**: the loop joint is dropped two levels before it could reach the builder. §00 correction 2. **No measured case exists**: the one shipped closed-loop sample, `projects/samples/devices/worlds/coupled_motors.omniworld`, was recorded as `ERROR` — explicitly "not as a pass" — and never produced a document. | CITED (source); the behavioural claim is **UNVERIFIED** | `docs/developer/roll-check.md:348` |
| **F13** | **Nothing in this tree runs physics for minutes.** The longest horizons found are lane 1's T4/T5 at 10 s, lane 3's determinism at 1000 steps (32 s), `golden_compare` at 400 ticks. Lane 1's T4 is a **3-link chaotic pendulum** with `energy_drift_rel` **0.381** at dt = 4 ms (raw MuJoCo 0.387, PyBullet 0.386) and 0.124–0.130 at dt = 1 ms. `determinism-scope.md` explicitly flags: *"if a claim depends on a 60-second rollout being replayable, measure it."* | MEASURED | `omnibench/lane1/gen_worlds.py:107`; `docs/benchmarks/lane1-postdeletion-2026-08-09.md` |
| **F14** | **`URDFRobot` is a source-level preprocessor expansion**, not a scene-tree node: `OmUrdfImporter.cpp` brace-scans the `.wbt` text for `URDFRobot { … }` and **replaces the block with generated VRML** before parsing (`OmTokenizer.cpp:404-415`). So "does importing change the physics" is exactly "does the URDF → VRML expansion preserve mass, inertia and axis". A preflight reporter exists: `scripts/dev/urdf_import.py --report --strict`. | CITED (source) | as listed |
| **F15** | Two engine defects of exactly the *import-fidelity* class have already shipped and been fixed: a husky-wheel inertia preset applied to every `Solid` lacking an explicit `inertiaMatrix`, and a revolute **child** `Solid`'s authored `rotation` dropped at joint build. | MEASURED (historical) | `AGENTS.md`; commit `98409b28` |
| **F16** | Largest measured fleets: **50 URDF robots** at 14.4 MB/robot — ⚠️ but with the empty `<generic>` controller fallback, so 50 controller *processes* and **not** 50 controller *workloads*; and **ten live controllers** in lane 3's dense determinism world. No ceiling was found and no failure mode recorded. | MEASURED | `tests/benchmarks/optim_bench.py`; `docs/developer/multi-instance-optimization-plan.md` |
| **F17** | Cross-machine bitwise identity is **untested**, and a sibling census found **56 of 180 lane-1 cells differing between two machines, 13 at relative error ≥ 1e-3.** | MEASURED | `docs/developer/cross-machine-determinism.md` |

### 2.1 The one probe this plan still needs

**P1 — is there a genuinely PASSIVE hinge on the Newton path?** F4 says every
joint is built with a live servo. If a `HingeJoint` with no motor device is
still built `POSITION_VELOCITY` and anchored at its authored angle, **OmniSim
has no passive pendulum**, and rungs 10 and 12 would be measuring that rather
than what they were written for. The probe is ten lines: author rung 10's
pendulum, release it at 30°, see whether it swings. *If it does not, that is a
first-class finding and the rung's red is correct* — but it has to be known
before anyone argues about tolerances.

The brief's other open question — *is there a batched entry point?* — is now
answered by F10. It changes rungs 15–17 substantially and is reflected in §5.7,
§5.9 and §7.

---

## 3. Contract amendments this plan requires

Six, each with the smallest shape that does the job.

### A. A rung is a scene FAMILY; a cell may contain more than one RUN

`CONTRACT.md` §2 says one scene per rung and §5 returns one sample document.
Rungs 9, 11, 12, 15, 16 and 19 need more than one run of one *family*.

> A rung owns **one generator**. A cell is **one or more runs** of scenes that
> generator produced, from one `arm.run()` call, and the rung is green only when
> every run in the cell meets its own analytic target.

```jsonc
{
  "rung": 9,
  "runs": [                       // present ONLY for multi-run rungs
    {"tag": "a", "params": {"eps_m": 0.0},  "pid": 12345, "t": [...], ...},
    {"tag": "b", "params": {"eps_m": 0.0},  "pid": 12346, "t": [...], ...},
    {"tag": "c", "params": {"eps_m": 1e-6}, "pid": 12347, "t": [...], ...}
  ]
}
```

Two rules that are not optional:

* **the tags and `params` are the CONTRACT's, not the arm's** — rung 9's replica
  set, rung 11's N sweep and rung 12's twin pair are declared in `rungs.py` and
  read by the arm, exactly as `rungs.rung5_x_cmd(t)` is today. An arm that chose
  its own N produces a row that is not comparable and nothing says so;
* **replicas must come from DISTINCT PROCESSES**, with `pid` and a process start
  time recorded so the reducer can assert it. A determinism rung whose two
  replicas are one process — or one array copied — measures the arm. This is the
  one place the ladder must check the arm rather than the engine, and it is
  cheap.

### B. `NOT_EXPRESSIBLE`, per CHECK, with evidence

Today a capability an arm does not have reads as a **failure**, which is
dishonest in both directions: it flatters us on rungs we happen to have and
punishes an engine for a category it never claimed. `buildbench` already has the
vocabulary (SPEC §0.2 rule 2); the ladder needs the mechanism.

```python
meta["not_expressible"] = {
  "radio_follow_distance": {
     "missing": "a communication device model: MuJoCo has no Emitter/Receiver "
                "node. A message between two robots would be a variable inside "
                "the driver process, so the SCENE cannot express it and the "
                "check would grade the arm rather than the simulator.",
     "citation": "<resolvable reference, quoted>",
     "status": "CITED",           # buildbench's verification_status vocabulary
  },
}
```

Runner semantics, all load-bearing:

1. an `N/E` check is **not green and not red**; it does not set the exit code;
2. a cell with ≥ 1 `N/E` prints `PARTIAL (4/6 expressed)`; all-`N/E` prints
   `NOT_EXPRESSIBLE`. Neither is ever drawn in the same colour as a failure —
   `buildbench` §0.1, adopted;
3. **an arm that declares `N/E` for a check it also produced a number for stops
   the run** (exit 2, the `ArmImportCollision` severity). A refusal and a
   measurement of the same quantity cannot both be true;
4. a declaration missing `missing` or `citation` is judged **RED**. This is the
   existing "`None` is red, never skipped" rule extended: *we did not look* must
   never read as *nothing was wrong*;
5. **`N/E` is declared in the arm's source, never inferred from a failed run.**
   An arm that tried and failed reports RED. This distinction is the whole value
   of the verdict and the easiest one to lose;
6. **`N/E` against our own name is recorded as prominently as anyone else's.**

### C. Detector validation — a green that cannot be made red is not a pass

Adopted from lane 4b, which reports `cliff_detector_validated: false` and
refuses to call its own green a pass (F3).

> For every rung and every arm, that rung's **must-red** faults must have been
> shown to go red **on that arm**. A rung whose battery has never been run on an
> arm — or was run and did not go red — reports that arm's row as `UNVALIDATED`,
> and `UNVALIDATED` is not a pass.

This bites hardest exactly where tier C does: rung 11's `truncate_budget` fault
sets a field lane 4b has already failed to make bite once (F3).

### D. Decimated sampling, and a lossless float round-trip

Rung 10 runs 150,000 steps. Recording every step is megabytes per series for no
gain: the fastest thing in that scene has a 1.67 s period.

> A rung may declare `SAMPLE_EVERY` in `rungs.py` (steps between samples,
> default 1). The arm samples on that stride and records the true simulated `t`
> of each sample. **The stride is the contract's, not the arm's.**

And, for rung 9 only, a rule that is invisible until it bites:

> Sample documents must round-trip float64 **exactly**. Python's `json` writes
> `repr(float)` and does; an arm that formats through `"%.6f"` — or records
> float32 — destroys the very quantity rung 9 measures, and the result would
> look like an engine that is deterministic to six decimals.

### E. Variants stay variants

`omnisim/variants.py` already has the right pattern: change one field at a time
against the identical scene, report it *beside* the row. Tiers C–E add several
(`mujoco_warp` for 9 and 11, `newtonNjmax` for 11, integrator for 10).

> A variant is never the headline. The ladder row is the declared-model run
> (R1–R5); every variant is published next to it with the one field it changed
> and that field's engine default.

⚠️ **F2's sizing trap is a rule for this ladder, not just a caution:** a
`newtonNjmax` set to a scene's *measured peak* produced results 8.81 m away from
every other size. Any rung that declares a constraint budget declares a
**generous** one (512+), records it under R4, and never sizes at the peak.

### F. External measured ground truth — an amendment to §1, deliberately narrow

`CONTRACT.md` §1 forbids an expected value read out of a running simulator. Rung
19 compares against 550 recorded tosses of a real cube.

> An expected value may also come from a **measurement of physical reality**,
> provided the recording is external to this project, vendored with its licence,
> and carries its own calibration record. **A measurement of a simulator remains
> forbidden — including ours, including a previous build.**

The distinction is the entire point of §1 and survives intact: a golden captured
from today's behaviour certifies today's defects; a tracked cube does not know
what a simulator is. `lane1r` already meets every condition — it re-derives its
scale factor, timestep and quaternion convention on every run rather than
trusting the dataset's own metadata.

---

## 4. Rungs I am NOT proposing, and why

The brief asked to be challenged. These five are the challenges.

### 4.1 Rejected: throughput as a rung (the brief's 12)

**It has no analytic ground truth, and the ladder's whole claim is that every
expected value has one.** "Env-steps per second" is a property of a machine, a
batch size and a counting convention: the same in-engine trainer measures
**10,228 env-steps/s at batch 256 on an RTX 3060 and 333,036 at 4096 on a
4090**, and even the clean graphed A/B pairs are 40,092 vs raw mjwarp's 47,949
at 256 (1.20×) and 91,577 vs 117,349 at 1024. A threshold would be a fit to one
machine; no threshold means no verdict; a rung with no verdict is telemetry with
ceremony.

**What replaces it is strictly better.** The ladder already records
`startup_s` / `step_s` / `total_s` per cell and prints them **without judging
them** (`run_ladder.print_timings`). Add a machine-attributed
`sim_seconds_per_wall_second`, and note the property that falls out:

> because a timing is printed beside a row that has already been judged against
> an analytic target, **every timing in this ladder is by construction a
> matched-fidelity timing.** That is what the brief's rung 12 wanted, and it
> needs no new assertion.

Judged throughput stays where it belongs, in OmniBench lane 2.

### 4.2 Rejected: the "requires both properties" composite (20 radio robots)

The brief's constraint is right — a rung that beats both competitors must
require both properties at once — but the composite I could build does not. A
twenty-rover radio convoy is **expressible and correct** on upstream Webots: it
is upstream's own device model and upstream's own multi-robot scene. It would be
*slower*, and §4.1 says the ladder does not judge speed. So the composite's red
column against Webots would be empty, and an honest composite would have to
smuggle a throughput threshold back in.

The genuine composite is **fidelity on a path Webots does not have**: rung 11's
`mujoco_warp` variant, rung 15, rung 17. Those are in the plan. The convoy is
not.

### 4.3 Rejected: an agent-authored rung (the brief's 20)

**It would cost the ladder the property that makes it useful.** The README's
first words about this suite are "Hand authored, no agent in the loop. Fast,
deterministic, runnable on every commit." An agent-authored rung brings an LLM,
an API key, non-determinism, per-cell cost and a pre-registration obligation
into a suite whose value is that a maintainer runs it in ninety seconds and
believes the answer.

**The alternative is better than the rung.** `analysis.reduce_samples` and
`rungs.check_rung` are already simulator-neutral, in physical units, and carry
their derivations. AgentBench already has adapters, a freeze discipline, an
oracle/null gate and a bare-shell control condition.

> Export the ladder's reducer and judge as AgentBench graders, and register
> "author rung N from its prompt" as AgentBench tasks. One grader, two suites,
> no duplicated ground truth — and the agent claim lands where it has the
> procedural protection it needs.

That is an interface, not a rung (§7, W3).

### 4.4 Designed, not scheduled: controller isolation

Worth writing down because it is the *honest* form of the brief's "N robots each
with its own controller process", and because it would go red on our own name.

**Claim.** One robot's controller dying must not perturb the other robots'
physics, and must not stop the clock. **Scene.** Rung 7's five rovers, each
closing its own sensor loop at its own threshold; at t = 3 s, r2's controller
exits. **Ground truth.** The four survivors each stop at their own analytic gap
(rung 6's derivation, per robot); `sim_time_end` reaches the requested duration.

**Why it is not scheduled.** The expected result on both Webots-family arms is a
red that reflects a *design decision* rather than a defect: the engine waits for
a controller inside `step()` unless the robot declares `synchronization FALSE`,
so a dead controller stops the world by construction. A rung whose red says
"this simulator is synchronous" is documentation — unless run in both
configurations, at which point it is a two-cell study of one field. It is a good
study. It is not a rung.

### 4.5 Rejected: bit-identity of robot *i* across different N (part of the brief's 11)

**It is not physically required, so a correct engine would fail it.** Adding
robots changes the size and ordering of the constraint system; floating-point
summation is not associative; robot *i* can differ in the last ULP at N = 32
versus N = 1 in an engine with no defect at all. A red that means nothing trains
everyone to ignore the row.

What replaces it is stronger: **the same analytic tolerance at every N, with no
N-dependent slack** (§5.3). The failure that motivates the rung — a silently
truncated constraint vector — is documented at 9 % displacement error, nearly
2× `DISTANCE_TOL`, so the analytic assertion catches it without claiming a
property physics does not have.

### 4.6 Why eleven and not eight

The brief asked for eight defensible rungs rather than twelve with filler, and
asked to be pushed back on. The push-back is §4.1–§4.5: **two of the brief's
twelve are rejected outright, one is designed and deliberately not scheduled,
and one more (camera recognition, §5.5) is deferred with its reason.** What is
left is eleven, and each has a claim no other rung makes. Rather than assert
that, here is the ranking — including which ones I would cut first, and what it
would take to cut them.

**Load-bearing; cutting any of these loses a claim nothing else makes:**

| rung | the claim only it makes |
|---|---|
| 18 closed loop | a capability we lost against the engine we forked from, with a closed-form ruler and no engine work to measure |
| 9 determinism | the reproducibility claim every other claim in this tree rests on, with a sensitivity control so a frozen world cannot satisfy it |
| 11 scale fidelity | the one open question two other lanes have failed to settle, put to an instrument already validated at N = 5 |
| 12 import fidelity | the strongest non-degradation claim available to us, on a defect class that has shipped twice |
| 10 long horizon | the only physics horizon in this tree longer than 32 s |
| 15 batched fidelity | the only rung that enters a category upstream Webots structurally cannot |
| 13 sensor suite | an independent second measurement of a contradiction between our matrix and our docs (§00 correction 3) |
| 16 policy parity | a falsifier for the project's loudest claim, which currently has none |
| 17 sensor-in-batch | the roadmap row we fail, and the reason W2 gets done |

**The two I would cut first, and I am naming them rather than waiting to be
asked:**

* **Rung 14 (radio).** It differentiates nothing: Webots expresses it natively
  and MuJoCo's `N/E` is honest but says nothing about physics. It survives on
  three grounds — it is the only rung testing a **causal chain between two
  robots**, its causal control (`radio_silence` ⇒ the follower must not move at
  all) is cheap and decisive, and the device model is a category a robot
  simulator is judged on. **What would cut it:** amendment B gives every rung
  per-check `NOT_EXPRESSIBLE`, so rung 14's four checks could fold into rung 13
  as a fifth device, dropping a scene and a launch. If the plan has to lose one,
  lose this one first.
* **Rung 19 (recorded reality).** It reuses `omnibench/lane1r`'s dataset,
  calibration and reducer, and the ladder adds only a cross-arm harness. It
  survives on one ground, but a strong one: **`embed_gap` is the only check
  anywhere in this plan that measures our translation layer against the engine
  we embed, using a ruler neither of us wrote.** **What would cut it:** if
  lane1r grows its own Webots and MuJoCo arms, rung 19 should not exist and the
  ladder should link to it instead. Check that before building it.

**If eight is the budget, the eight are 18, 9, 11, 10, 12, 13, 15, 16** — and
what is given up is stated rather than hidden: the device model loses its
coordination case, the plan loses its only external ruler, and the
sensor-in-batch roadmap row disappears along with the pressure it puts on W2.
Note that the eight still contain **three rungs we are predicted to fail** (18,
13, and 16's blocked sensed variant), so the reduced set does not become a tier
engineered to green.

---

## 5. The rungs

### 5.1 Rung 9 — determinism, with its own sensitivity control

**Claim.** The same scene run twice, from two fresh processes, produces
identical numbers — in a scene where a perturbation a thousand times smaller
than the tightest tolerance in this ladder would *not* stay small.

**Scene, and why it is a PILE.** A 5 × 5 grid of `BOX_EDGE` cubes resting on the
rung-1 floor with 1 mm gaps, plus a 26th cube released from `RUNG9_SPAWN_Z` =
1.6 m onto the corner of a centre cube. 8 s.

The pile is not decoration. F1's GPU refutation has a **named mechanism** —
`pairid = wp.atomic_add(...)` in mujoco_warp's contact-pair assignment — which
requires many simultaneous contact pairs to bite, and F1's own counter-note
records a **single-contact scene reproducing bit-identical 3/3 cold on
`mujoco_warp`**. A two-body rung-9 scene would therefore return a *false green*
on the GPU variant, in exactly the way rung 5's static scene "cannot refute a
stale-scene freeze and is not offered as doing so". 25 resting cubes give ~100
contact points, inside the 80–320 band where divergence is measured.

**Runs (amendment A).** Three, from three processes:

| tag | scene | purpose |
|---|---|---|
| `a` | as authored | the reference |
| `b` | as authored | the repeat |
| `c` | dropped cube's spawn x + `RUNG9_EPS` | the sensitivity control |

`RUNG9_EPS = 1e-6 m` — 1000× below `PENETRATION_TOL`, i.e. physically
irrelevant by the ladder's own standards.

**Ground truth.** Identity is self-consistent (two identical inputs, one
machine, one build). The analytic anchor is rung 2's: the dropped cube falls
from 1.6 m onto a cube top at 0.7 m, so its centre crosses `RUNG9_GATE_HI` = 1.2
and `RUNG9_GATE_LO` = 0.8, and

```
fall_interval = √(2(1.6−0.8)/g) − √(2(1.6−1.2)/g) = 0.40385 − 0.28557
              = 0.11828 s
```

**Assertions.**

| check | measured | expected | tol | derivation |
|---|---|---|---|---|
| `repeat_delta` | max over every series and sample of \|a − b\| | 0.0 | **0.0** | identical inputs, identical code path, one machine. Anything else is state that leaked between runs |
| `repeat_length` | \|steps(a) − steps(b)\| | 0.0 | 0.0 | a run that ended early has a trivially small delta **over the overlap** |
| `sensitivity_shortfall` | max(0, `RUNG9_SEP_MIN` − \|a − c\| at t_end) | 0.0 | 0.0 | one-sided, the `roll_overrun` idiom. `RUNG9_SEP_MIN` = 1e-3 m ⇒ 1000× amplification of a 1 µm seed. Round-off cannot do this: float64 relative round-off is 2.2e-16 and a random walk over 2000 steps reaches ~1e-14 relative, ten orders short |
| `fall_interval` | the dropped cube's gate-to-gate interval | 0.11828 s | `FALL_INTERVAL_TOL` | rung 2's derivation, unchanged. It is the **analytic anchor**, and see the hole below |

**Whole-run invariant.** `repeat_delta` — a maximum over every sample of every
series, not an end-state comparison. Two runs that diverge at t = 2 s and
reconverge by chance at t = 8 s fail it.

**What would still pass while broken.**

* **A frozen world is perfectly deterministic.** Zero motion ⇒ `repeat_delta` =
  0 exactly. This is the rung's central hole, closed twice: `sensitivity_shortfall`
  (a frozen world amplifies nothing) and `fall_interval` (no crossing ⇒ `None` ⇒
  RED). Lane 3's existing determinism harness already grades a `no_motion` case
  for the same reason; this rung makes it an assertion instead of a grade.
* **An engine that is deterministic and WRONG passes both determinism checks.**
  Gravity at 5 m/s² is exactly as reproducible as 9.81. `fall_interval` is the
  only check that sees it — which is why an analytic anchor is mandatory in any
  rung whose other checks are self-consistent.
* **An arm replaying a cached result** passes everything. Closed by amendment
  A's distinct-`pid` rule.
* **Declared, not closed:** a red `sensitivity_shortfall` is ambiguous between
  "this engine damps perturbations" and "this scene is not chaotic on this
  engine". Read it only alongside `repeat_delta`; the informative signal is the
  *pair*.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `seed_nudge` — replica `b` spawned 1e-12 m off | `repeat_delta` | `sensitivity_shortfall`, `fall_interval` |
| `frozen` — the dropped cube is made static | `fall_interval`, `sensitivity_shortfall` | **`repeat_delta`** — the proof that determinism alone is not evidence |
| `short_b` — replica `b` runs 4 s | `repeat_length` | `fall_interval` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` (CPU `mj_step`) | **PASS** | F1: bitwise at 336 contacts with ten live controllers | high |
| `omnisim` (`mujoco_warp` **variant**) | **FAIL `repeat_delta`** | F1: 0 of 24 pairs across six scenes; the pile is deliberately inside the contact-count band where it was measured | medium-high — and *low* if the pile turns out sparser than intended, which is why the contact count must be recorded in `meta` |
| `mujoco` | PASS | documented deterministic, single-threaded CPU | high |
| `webots` | PASS | ODE single-threaded. ⚠️ `WorldInfo.optimalThreadCount > 1` would break it; the arm must record the value it ran at | medium |

**Cost.** 3 × 8 s = 6,000 steps, 26 bodies. **Every commit.**

**Dependencies.** Amendments A and D. No engine work.

---

### 5.2 Rung 10 — 600 seconds

**Claim.** Nothing drifts. A conservative system keeps its energy and a static
system stays where it was put, over a run 19× longer than the longest physics
horizon anywhere in this tree (F13).

**Scene.** One scene, two spatially separated sub-systems on the rung-1 floor:

* **a passive pendulum** — a uniform rod, `PEND_L = 1.0 m`, `PEND_M = 0.5 kg`,
  hinged about a **horizontal** axis at `z = FLOOR_TOP + 1.4`, no motor, no
  damping, released from rest at `θ0 = 30°`. At θ0 the rod's lowest point clears
  the floor by 0.4 m, so nothing it does can touch anything;
* **a three-box stack** — three `BOX_EDGE` cubes at `x = 3.0`, centres at
  z = 0.6 / 0.8 / 1.0, in static equilibrium as authored.

`RUNG10_DURATION = 600 s`, `SAMPLE_EVERY = 5` (20 ms; 83 samples per pendulum
period, 30,000 samples total).

**Why a single rod and not a chain.** Lane 1's T4 *is* a pendulum — a **3-link
chaotic chain**, and at dt = 4 ms it drifts **38.1 %** of its energy in 10 s
(raw MuJoCo 38.7 %, PyBullet 38.6 %). A chaotic chain is an unusable long-horizon
instrument: its drift is dominated by trajectory divergence, and no tolerance on
it can be derived. A single rod at small amplitude is integrable and its drift is
analytically bounded, which is the entire reason it is the scene here. The two
measurements are complementary and neither replaces the other.

**Ground truth.**

*Period.* For a uniform rod pivoted at one end, `I = mL²/3`, `d = L/2`:

```
T0 = 2π √(I/(m g d)) = 2π √(2L/3g) = 1.63799 s            (L = 1, g = 9.81)
T  = T0 (1 + θ0²/16 + 11θ0⁴/3072 + …) = 1.66649 s         (θ0 = 30°)
ω  = 2π/T = 3.7703 rad/s
```

The truncated term is 2.7e-4 of T: the analytic value is good to 0.03 %.

*Energy, without a velocity sensor.* No arm here has a joint-velocity readout on
a passive hinge, and finite-differencing a 20 ms-sampled angle would put the
sampling scheme into the physics. It is not needed: **at every turning point the
energy is purely potential**, so the sequence of turning-point amplitudes *is*
the energy history, read from the position sensor alone:
`E_k = m g d (1 − cos θmax,k)`.

*The band.* Semi-implicit (symplectic) Euler — every arm's default family —
conserves a modified Hamiltonian exactly, so the true energy oscillates with a
**bounded** amplitude of order `ω dt / 2` and does **not** drift secularly. Here
`ω dt/2 = 0.754 %`. Near θ0, `δE/E = 3.732 δθ`, so the band in *amplitude* is
0.386 %.

**Assertions.**

| check | measured | expected | tol | derivation |
|---|---|---|---|---|
| `pend_period` | mean interval between successive same-sign zero crossings, whole run | 1.66649 s | 8 ms (0.5 %) | 25× the 0.03 % truncation; symplectic Euler's frequency shift `ω²dt²/24 ≈ 1e-5` is negligible |
| `pend_amp_drift` | (mean θmax over the last 60 s − over the first 60 s) / θ0 | 0.0 | **0.015** | the *secular* claim: 4× the 0.386 % bounded band. A first-order **non**-symplectic scheme drifts monotonically straight through it |
| `pend_amp_band` | peak-to-peak θmax over every turning point / θ0 | 0.0 | 0.02 | 5× the derived band; separates bounded oscillation (physical) from drift (a defect) |
| `swing_count` | max(0, 300 − turning points seen) | 0.0 | 0.0 | one-sided motion floor; 600 s / 1.666 s ⇒ ~360 half-cycles, so 300 is comfortable |
| `stack_creep` | max over the run of the bottom box's planar displacement from its authored pose | 0.0 | 1 mm | static equilibrium: there is no force to move it. 1 mm over 600 s is 1.7 µm/s |
| `stack_tilt` | max over the run of the top box's tilt from vertical | 0.0 | 1.0° | a 0.1 mm differential penetration across a 0.2 m base is 0.03°; 1° is 30× that and 1/90 of a collapse |
| `stack_top_z` | mean top-box centre z over the last 60 s | 1.0 m | `REST_Z_TOL` | three cubes on a floor topped at 0.5, plus rung 1's compliance bound |
| `run_length` | `sim_time_end` | 600 s | `2 × DT` | rung 0's `finite_clock`, at 150,000 steps rather than 250 |

**Whole-run invariants.** `pend_amp_band`, `stack_creep`, `stack_tilt`,
`swing_count` are whole-run extrema. `pend_amp_drift` is deliberately a
*between-window* comparison — it is the one quantity a whole-run maximum cannot
express, because slow monotone drift and fast bounded jitter can share a maximum.

**What would still pass while broken.**

* **A locked pendulum has perfect energy conservation.** θmax = 0 forever ⇒
  `pend_amp_drift` and `pend_amp_band` both green. Closed by `swing_count` and
  `pend_period` (no crossings ⇒ `None` ⇒ RED). This is rung 10's frozen world,
  and `locked_hinge` exists to prove it.
* **A stack that never had contact** — three boxes floating on a phantom plane —
  passes creep and tilt. Closed by `stack_top_z`, and by the floor being at
  z = 0.5 so the phantom plane at z = 0 is separable.
* **Declared, not closed:** 600 s is not six hours. This bounds drift *rates* at
  the millimetre / percent level over ten minutes and no claim beyond that may be
  made from it.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `damped_hinge` — joint damping added to the pendulum | `pend_amp_drift` | `stack_creep`, `stack_tilt`, `stack_top_z` |
| `locked_hinge` — the hinge held at θ0 | `swing_count`, `pend_period` | **`pend_amp_band`** — the proof that energy conservation alone is not evidence |
| `tilted_floor` — floor authored 0.5° out of level | `stack_creep` | `pend_period`, `pend_amp_drift` (the pendulum touches nothing) |
| `short_run` — stops at 60 s | `run_length` | — |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | **UNKNOWN, gated on P1** | if a Newton hinge with no motor is still built `POSITION_VELOCITY` with a live servo (F4), the pendulum will not swing freely and the rung goes red on `pend_period`/`swing_count`. That would be a real finding about the engine — but P1 must be probed before the rung is built | — |
| `mujoco` | PASS on the pendulum; **`stack_creep` is a live risk** | MuJoCo's default cone is pyramidal with `noslip_iterations` 0 — the same soft-tangential drift that fails it at rung 8's own defaults (`CONTRACT.md` §3b). 600 s is a much longer lever than 7.5 | medium |
| `webots` | PASS | ODE is a hard-constraint LCP and passes rung 8 with nothing declared; static friction over 600 s should be its strong suit | medium |

**Cost.** 150,000 steps, ~5 bodies. MuJoCo: seconds. OmniSim: at the 0.45 ms/step
measured for a 5-box scene, ≈ 68 s. **Nightly**, with a 60 s smoke variant every
commit — and the smoke variant's green **may not be quoted as the 600 s claim**;
it exists to catch a scene that stopped loading.

**Dependencies.** Amendment D; probe P1.

---

### 5.3 Rung 11 — fidelity at scale

**Claim.** Every robot in a fleet meets the **same** analytic target it would
meet alone, with the **same** tolerance, at every fleet size.

**Scene.** Rung 4's rover, `N ∈ {1, 4, 8, 16, 32}`, in a lane grid, each robot
commanded its own wheel rate cycled from `RUNG7_OMEGA`. Five runs, one generator
(amendment A). The floor is sized by a contract function carrying the 3× margin
`FLOOR_SIZE`'s comment already argues for — *a rover that runs off the lip
produces a beached-and-spinning signature that masks whatever defect pushed it
there.*

**Ground truth.** Rungs 4 and 7, unchanged: `d_i = ω_i r T` over `RUNG7_WIN`,
`roll_overrun = 0`, axle at `FLOOR_TOP + WHEEL_R`, separation geometric —
evaluated per robot, per N.

**Assertions** (per N, worst over the fleet; tolerances are rungs 4/7's and are
**not** widened with N):

| check | expected | tol |
|---|---|---|
| `distance_worst(N)` | 0.0 | `DISTANCE_TOL` = 0.05 |
| `wheel_omega_worst(N)` | 0.0 | `OMEGA_REL_TOL` = 0.01 |
| `roll_overrun_worst(N)` | 0.0 | `ROLL_OVERRUN_TOL` = 0.05 |
| `ride_worst(N)` | 0.0 | `RIDE_HEIGHT_TOL` = 0.02 m |
| `min_separation(N)` | lane spacing | `LATERAL_TOL` = 0.10 m |
| `robots_seen(N)` | N | 0 |

**Why no N-dependent slack.** The failure this rung exists to catch — a silently
truncated constraint vector (F2) — is documented at **9 % displacement error**,
nearly twice `DISTANCE_TOL`. A tolerance that grew with N would be
pre-authorised to miss exactly the defect it was built for.

**Two declarations this rung must carry under R4.** The constraint budget
(`newtonNjmax`/`newtonNconmax`) and the number of controller processes.
F2's sizing trap makes the first mandatory: a budget set at a scene's *measured
peak* moved results 8.81 m, so the honest scene declares a **generous** budget
(512+) and the *fault* is what starves it. And F16 makes the second load-bearing:
the largest fleet measured in this tree with **live** controllers is ten; 50 was
measured only with the empty `<generic>` fallback. At N = 32 the ~18 MB Python
interpreter per robot and its IPC dominate, so the rung must say whether it
commanded the fleet from one driver or from N — otherwise the row silently
measures controller plumbing at scale rather than physics at scale.

**Whole-run invariants.** `roll_overrun_worst`, `ride_worst`, `min_separation`,
`lateral_worst` — all maxima/minima over every sample, inherited from rung 7.

**What would still pass while broken.**

* **An engine that silently drops robots** reports a well-behaved smaller fleet.
  Closed by `robots_seen` and `min_separation` (coincident spawns).
* **An engine that simulates one robot and copies it** passes distance and
  overrun. Closed by the **distinct commands**: clones land on one target and
  N−1 reds appear.
* **Declared, not closed:** rung 4's limit is inherited — the wheels are shown
  kinematically consistent with the motion, not to have propelled it.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `stalled_robot` at N = 16 | `distance_worst` | `min_separation`, `lateral_worst` |
| `lane_offset` at N = 16 — a **spawn** offset, never a per-step write (`CONTRACT.md` §6 records why) | `min_separation` | `distance_worst`, `wheel_omega_worst` |
| `truncate_budget` — OmniSim variant, `newtonNjmax` pinned far below 32·N | `distance_worst` **or** `roll_overrun_worst` | `robots_seen` |

**⚠️ `truncate_budget` is under amendment C and is expected to be difficult.**
Lane 4b already tried to force this overflow three ways at N = 12 and read
`384/384` every time, concluding either that the runtime auto-sizes the buffer or
that `Data.njmax` is not the cap that governs truncation (F3). If it cannot be
made to bite here either, **rung 11's OmniSim GPU rows are `UNVALIDATED` and are
not a pass** — and that is the most useful thing this rung could produce, because
it puts a *second* instrument on a question lane 4b left open in both directions,
on a rover whose analytic target is known and which already passes at N = 5.

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` (CPU) | PASS at every N | constraint arrays are not the CPU path's limit; rung 7 passes at N = 5 with 0.086 % worst distance error | medium-high |
| `omnisim` (`mujoco_warp` **variant**) | **UNKNOWN — this is the instrument** | F2's ~9-robot threshold is unconfirmed in both directions; N = 8 vs N = 16 straddles it. ⚠️ Also: lane 4b's rovers ran **below realtime at N = 1** on the GPU path, so this variant is not cheap | — |
| `mujoco` | PASS | sizes its own constraint arrays | high |
| `webots` | PASS, slowly | ODE has no equivalent fixed cap; §4.1 is why "slowly" is not a verdict | medium |

**Cost.** 5 runs × 6.5 s; the N = 32 run is 160 bodies. **Every commit for
N ≤ 16; N = 32 nightly.**

**Dependencies.** Amendments A, C, E. No engine work; a contract-owned floor
sizing function and lane generator.

---

### 5.4 Rung 12 — import fidelity: the same robot, authored twice

**Claim.** Importing a robot from URDF does not change its physics.

This is §0.1's non-degradation claim in its purest form, it is the claim our own
layer is most likely to break (F15: two shipped defects of exactly this class),
and it is stated entirely without reference to who can run it.

**What is actually under test.** F14: `URDFRobot` is a **source-level
preprocessor expansion** — the importer brace-scans the `.wbt` text and replaces
the block with generated VRML before the parser ever sees it. So the imported
twin and the native twin end up as the *same kind of thing*, and the rung is a
clean differential on one translation step: **did mass, inertia, COM and axis
survive the expansion?**

**Scene.** One generator, two authorings, three bodies:

* **the native twin** — rung 10's pendulum, authored in the arm's own dialect
  from `rungs.py`;
* **the imported twin** — the identical rod (same mass, COM offset, inertia
  tensor, joint axis, release angle) from a URDF **`rungs.py` generates**, so the
  two authorings cannot drift apart;
* **an imported rover** — rung 4's rover as URDF, driven at `RUNG4_OMEGA_CMD`.

20 s for the pendulums (12 periods) plus the rover's 6.5 s.

**Ground truth.** The pendulum period `T = 1.66649 s` (§5.2), which is
*mass-distribution sensitive*: `T = 2π√(I/(mgd))` pins `I`, `m` and `d`
together. And the rover's `d = ω r T`, which is mass-*in*sensitive and therefore
only pins that actuation survived. **Both are needed**: a period alone would not
notice a dead motor; a distance alone would not notice a discarded inertia
tensor, which is precisely what one shipped defect did (F15).

**Assertions.**

| check | measured | expected | tol | derivation |
|---|---|---|---|---|
| `pend_period_native` | as §5.2 | 1.66649 s | 8 ms | §5.2 |
| `pend_period_urdf` | as §5.2, imported twin | 1.66649 s | 8 ms | §5.2 |
| `import_period_delta` | \|T_urdf − T_native\| | 0.0 | **1.6 ms** | the twin comparison is *tighter* than either absolute check because the integrator's systematic phase error is common to both and cancels. 0.1 % of T, 5× the analytic truncation |
| `import_axis_error` | max over the run of the imported bob's out-of-plane displacement | 0.0 | 2 mm | a pendulum released in the x–z plane about a y axis stays in it. This is the check for the most common import defect — an axis or frame convention — and 2 mm on a 1 m rod is 0.11°. F15's dropped child `rotation` is exactly this failure |
| `urdf_distance` | rover distance over `RUNG4_WIN` | `ω r T` | `DISTANCE_TOL` | rung 4 |
| `urdf_roll_overrun` | rung 4's one-sided invariant | 0.0 | `ROLL_OVERRUN_TOL` | rung 4 |

**Whole-run invariants.** `import_axis_error`, `urdf_roll_overrun`.

**What would still pass while broken.**

* **An importer that ignores the URDF and reuses a built-in robot** passes every
  check. This is the central hole, closed the way rung 8 closes its own: **a
  causal control in the fault battery.** `urdf_inertia_x2` doubles the inertia
  in the URDF and nothing else; the imported period must move by √2 (0.69 s,
  430× the tolerance). *If it does not move, the rung is void on that arm* —
  `UNVALIDATED` under amendment C, never green.
* **Declared, not closed:** a rod and a four-wheel rover. Nothing here covers
  meshes, collision-hull generation, joint limits, or the 6-DOF chains where
  real importers actually fail. A later rung.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `urdf_inertia_x2` — **the causal control** | `pend_period_urdf`, `import_period_delta` | `pend_period_native`, `urdf_distance` |
| `urdf_axis_swapped` — the hinge axis declared about x | `import_axis_error` | `pend_period_native` |
| `urdf_motor_cut` — the rover's URDF transmission removed | `urdf_distance` | `pend_period_urdf`, `import_period_delta` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | PASS expected; **`import_period_delta` is the row to watch** | `URDFRobot` is native and one of this project's headline additions. F15 says this exact defect class has shipped twice and been fixed twice — an argument for a standing check, not against one. `scripts/dev/urdf_import.py --report --strict` is the preflight | medium |
| `mujoco` | PASS | `mj_loadXML` reads URDF natively. ⚠️ **CITED, not measured**: MuJoCo's URDF path has documented restrictions and may need a `<mujoco>` compiler block for inertia handling; whatever it needs is declared under R4 | medium |
| `webots` | **`N/E` or PASS-with-scaffold — record whichever, plainly** | upstream has **no runtime URDF node**; the supported route is `urdf2webots`, an official but *offline* Cyberbotics converter. If the arm runs it in `worldgen`, the row is a genuine PASS and should read "via urdf2webots, offline"; if the converter is not in the tree, `N/E` with that named as the missing capability | medium |

Note which way that last row cuts. It is the honest answer, and it is *less*
flattering than "Webots cannot import URDF" — the sentence a rung
reverse-engineered from a competitor's gap would have produced.

**Cost.** 2 × 20 s + 6.5 s ≈ 11,600 steps of small scenes. **Every commit.**

**Dependencies.** Amendments A, B, C; a URDF emitter in `rungs.py` (~80 lines,
generated from the same constants as the native twin or the rung is measuring two
different rods).

---

### 5.5 Rung 13 — the sensor suite reports physics

**Claim.** An inertial or contact sensor's reading equals the physical quantity
it is a model of — in SI units, with no convention negotiated inside the arm.

**Scene.** One scene, three independent sub-systems on the rung-1 floor:

* a **sensor cube** — rung 2's box carrying an `Accelerometer`, released from
  `RUNG2_SPAWN_Z` onto a pad;
* the **pad** — a plate carrying a touch sensor, on the floor;
* a **spinner** — rung 3's vertical hinge and link, unchanged, carrying a `Gyro`.

3 s, so the fall, the impact and the settle all fit.

**Ground truth — every value is asserted on a MAGNITUDE, so no frame, sign or
axis-order convention can hide in it.**

| quantity | value | derivation |
|---|---|---|
| accelerometer, free fall | **0 m/s²** | an accelerometer measures *proper* acceleration; a body in free fall is inertial. Exact, and the single best cross-engine check in the suite |
| accelerometer, at rest | **9.81 m/s²** | supported at rest ⇒ proper acceleration = g |
| gyro, on the driven link | **2.0 rad/s** = `RUNG3_OMEGA_CMD` | rung 3's own ground truth |
| touch trigger time | `fall_time_s(RUNG2_DROP_M)` = 0.4515 s | rung 2's derivation |
| touch force at rest | `BOX_MASS × G` = **9.81 N** | static equilibrium |

**Assertions.**

| check | expected | tol | derivation |
|---|---|---|---|
| `accel_freefall` (mean \|a\| over 0.1–0.35 s) | 0.0 | 0.05 m/s² | 0.5 % of g; no drag and no contact in the window, so the only residual is solver noise |
| `accel_rest` (mean \|a\| over the settle window) | 9.81 | 0.05 m/s² | same bound, other end |
| `accel_up_alignment` (\|a·ẑ\|/\|a\| at rest) | 1.0 | 0.01 | the reaction is vertical to 0.8° — catches an axis permutation *without* asserting a sign convention |
| `gyro_rate` (mean \|ω\| over `RUNG3_WIN_A`) | 2.0 rad/s | `omega_tol(2.0)` = 0.02 | rung 3 |
| `touch_trigger_t` | 0.4515 s | `4 × DT` | rung 2's `fall_time_abs` tolerance, same argument |
| `touch_force_rest` | 9.81 N | 0.5 N | 5 %; contact compliance oscillates the normal force for a few steps after impact, so it is read in the settle window as a static quantity |

**Whole-run invariant.** `accel_consistency` — max over the whole run of
\|a_sensor − a_derived\|, where `a_derived` is the second difference of the
cube's recorded pose. It is rung 5's `range_tracks` for an accelerometer: the
only check that sees a sensor which is right at both ends and stops updating in
between. Second-differencing a 4 ms-sampled pose amplifies noise, so this one is
deliberately loose (1.0 m/s²) and is **excluded across the impact**, where a
second difference of a sampled pose is meaningless — with the exclusion window
declared in `rungs.py`, never chosen by an arm.

**What would still pass while broken.**

* **A sensor returning a computed quantity rather than a measured one** —
  rung 5's declared, unclosed limit — passes here too. It is *narrowed* by
  `accel_freefall`: an engine computing "acceleration" as the second derivative
  of the pose returns −9.81 in free fall, not 0, so the shortcut must be a
  deliberately correct proper-acceleration computation to fool this rung.
* **A touch sensor wired to the clock rather than to contact** passes
  `touch_trigger_t`. Closed by the `no_pad` fault: remove the pad's
  `boundingObject` and the trigger must never fire.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `no_pad` — the pad loses its `boundingObject` | `touch_trigger_t`, `accel_rest` | `accel_freefall`, `gyro_rate` |
| `frozen_accel` — the accelerometer held at its t = 0 value | `accel_consistency`, `accel_rest` | `touch_trigger_t`, `gyro_rate` |
| `spin_stop` — the spinner's motor cut | `gyro_rate` | every accelerometer and touch check |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | **PASS on accel/gyro; BOTH touch checks predicted RED** | F5: lane 4 measured `touch_bumper` at `max_value = 0` over 750 samples *and* `touch_force` at 0 N against 19.62 N expected, with the mechanism being that the sensor's own `boundingObject` is not a collider. ⚠️ **The backend guide still says bumper reads are native and default-on** — this rung would be an independent second measurement of a documented contradiction, in newtons and seconds | high |
| `mujoco` | PASS | `accelerometer`, `gyro` and `touch` are native sensor types | high |
| `webots` | PASS | `Accelerometer`, `Gyro` and `TouchSensor` (bumper **and** force) are upstream device nodes | high |

That table is the shape §0.1 predicts: our own row carries the only red, the red
is a specific documented gap, and no competitor is disadvantaged by the choice of
task.

**Deferred deliberately: camera recognition.** It has an analytic ground truth
(the target's true relative pose) and a clean differential (an occluder must make
the target disappear), and F7 says the recognition/Radar occlusion path was
rebuilt on `mj_ray` in August and **has no lane-4 probe** — so a rung would be
its *first* measurement, which is an argument for building it. It is deferred
anyway because it needs a headless GL path on three arms including WSL2, and
because per-arm rendering variance would land in the tolerance. Recorded as a
candidate, with the reason, rather than dropped.

**Cost.** 3 s, one small scene. **Every commit.**

**Dependencies.** Amendment B (so an arm lacking a sensor type refuses that check
rather than failing the rung). No engine work — the expected red *is* the
finding.

---

### 5.6 Rung 14 — a message changes another robot's motion

**Claim.** One robot broadcasts a number; another receives it and its motion
changes accordingly, by an amount the geometry predicts.

**Scene.** Two rung-4 rovers in adjacent lanes. The **leader** carries an emitter
and broadcasts a commanded wheel rate every step: `0.0` until
`RUNG14_T_GO = 2.0 s`, then `RUNG4_OMEGA_CMD`. The **follower** carries a
receiver and drives at whatever rate it last received. Neither robot holds the
schedule; the follower has only the messages. 6.5 s.

**Ground truth.** The follower's distance over
`(RUNG14_T_GO, RUNG14_DURATION)` is `ω r (T − Δ)` with Δ the delivery latency.
Every engine here delivers on the following step at the latest, so `Δ ≤ 2 DT`
= 8 ms — 3.2 mm of travel against a 1.8 m target, 0.18 %, and 28× inside
`DISTANCE_TOL`. The latency is therefore **bounded, not modelled**, and no
engine-specific delivery semantics enter the expectation.

**Assertions.**

| check | expected | tol | derivation |
|---|---|---|---|
| `radio_follow_distance` | `ω r (RUNG14_DURATION − RUNG14_T_GO)` = 1.8 m | `DISTANCE_TOL` | rung 4, with the ≤ 8 ms latency shown above to be 28× inside it |
| `radio_quiet_distance` | max follower displacement over every sample **before** `RUNG14_T_GO` | 0.0 | 5 mm | **the causal check**: a follower that ignored the radio and simply drove would be 0.8 m along by then |
| `radio_leader_distance` | `ω r (T − T_GO)` | `DISTANCE_TOL` | the leader is an ordinary rung-4 rover; if it is wrong the rung is not about the radio |
| `radio_lag` | follower start − `RUNG14_T_GO` | 0.0 | `4 × DT` | one step to send, one to receive, one to act, carried at 4/3 |

**Whole-run invariant.** `radio_quiet_distance` — a maximum over every sample
before the go message. A follower that crept forward and then corrected passes a
final-distance check and fails only here.

**What would still pass while broken.** A follower whose controller happens to
contain the same schedule passes everything. **Closed by the causal control**,
exactly as rung 8's `no_grip` licenses its lifting claim: with `radio_silence`
the follower **must not move at all**. That differential is this rung's licence
to say the message did the work.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `radio_silence` — **the causal control** | `radio_follow_distance` | `radio_leader_distance`, `radio_quiet_distance` |
| `radio_early` — the leader broadcasts the go value from t = 0 | `radio_quiet_distance` | `radio_leader_distance` |
| `radio_wrong_value` — the leader broadcasts `2 × ω` | `radio_follow_distance` | `radio_leader_distance`, `radio_quiet_distance` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | PASS | F6: lane 4 **measured** a packet arriving from a second robot on channel 1; the `Receiver` is served by the native `mj_ray` service, default on | high |
| `webots` | PASS | upstream's own device pair. **This rung does not differentiate us from Webots and must never be presented as if it did** | high |
| `mujoco` | **`N/E`** | MuJoCo models no communication device. A message between two robots would be a variable inside the driver process, so the *scene* cannot express it and the check would grade the arm rather than the simulator. Named capability + citation required under amendment B | high |

**Note against `buildbench` rule 1.** Rung 7 deliberately excluded radio because
a verdict one arm structurally cannot express "says nothing about the physics".
That is correct **for a physics rung** and does not apply here: tier D's question
is explicitly the robot *system* model, and "robots coordinating over a radio" is
work a practitioner recognises without knowing who can run it. Rung 7's exclusion
stands on its own terms.

**Cost.** 6.5 s, two rovers. **Every commit.**

**Dependencies.** Amendment B. No engine work.

---

### 5.7 Rung 15 — the batched path is the same physics

**Claim.** K environments stepped together each produce the physics they would
produce alone.

**This is buildable today, and that is new information.** F10: the engine
already exposes `_mpc_rollout_buffers(K)`, which builds `nworld = K` mujoco_warp
buffers **from the same compiled CPU `MjModel` the live solver steps**, and
`_mpc_seed_from_live(K)`, which broadcasts live state into all K worlds; the
entry point is the documented env hook `OMNISIM_INENGINE_PYMOD`. The ladder's arm
registers a module on that hook, seeds K worlds, steps them and records per-env
poses. That is **arm work, not engine work** — the brief assumed otherwise and
so did the first draft of this plan.

**Scene.** Rung 4's rover, one per env, `K ∈ {1, 16, 64}`, with **two
populations inside each batch**:

* envs 0…K/2−1 each get a **different** wheel rate cycled from `RUNG7_OMEGA`;
* envs K/2…K−1 are **clones**: identical scene, identical command, identical
  initial state.

**Ground truth.** Distinct envs: `d_i = ω_i r T`, exactly rung 4. Clone envs:
identical to each other — a *self-consistent* zero, legitimate because the clones
share one code path and one set of inputs inside one process.

**Assertions.**

| check | expected | tol | derivation |
|---|---|---|---|
| `batch_distance_worst` | 0.0 | `DISTANCE_TOL` | rung 4 per env, no K-dependent slack (§4.5) |
| `batch_overrun_worst` | 0.0 | `ROLL_OVERRUN_TOL` | rung 4's one-sided invariant, per env, whole run |
| `batch_clone_spread` | 0.0 | **0.0** | clones in one batch share the code path and the inputs; any difference is per-env state corruption or a reduction crossing env boundaries. ⚠️ On `mujoco_warp` this is the check F1's `atomic_add` mechanism is most likely to redden — and that would be a **finding**, not a tolerance to widen |
| `batch_vs_solo` | 0.0 | `DISTANCE_TOL / 5` | env 0 against the same scene run singly. Not bitwise (§4.5); 1 % is 5× tighter than the absolute check because the systematic part cancels |
| `envs_seen` | K | 0 | an engine that quietly ran fewer envs |

**Whole-run invariants.** `batch_overrun_worst`, and `batch_clone_spread` (a
maximum over every env, series and sample).

**What would still pass while broken.**

* **An engine that steps env 0 and copies it** passes clone-spread perfectly.
  Closed by the distinct-command population: copies land on one target and half
  the batch goes red.
* **An engine that runs the batch but reads back only env 0** — closed by
  `envs_seen` plus the distinct targets.
* **Declared:** nothing here says how *fast* the batch was, by design (§4.1).

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `env_bleed` — env 3's command applied to env 4 | `batch_distance_worst` | `batch_clone_spread`, `envs_seen` |
| `clone_seed_jitter` — one clone's spawn nudged 1e-9 m | `batch_clone_spread` | `batch_distance_worst` |
| `half_batch` — K/2 envs actually stepped | `envs_seen` | `batch_distance_worst` on the envs that ran |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | **UNKNOWN, and worth knowing** | the path exists (F10) and seeds from the live model, which is the parity claim's own machinery. The risks are F1's GPU nondeterminism showing up as clone spread, and `MPC_NJMAX`/`MPC_NCONMAX` (both 256) needing to be declared under R4 and E | — |
| `mujoco` | PASS | `mujoco_warp` / MJX batch natively and this tree already vendors the runtime | medium |
| `webots` | **PASS, trivially — and say so** | upstream has no vectorised API, so the honest implementation is K processes, in which "env 0 versus solo" is the same code path and the check is satisfied for free | high |

**That last row inverts the marketing instinct and the plan keeps it.** A
process-per-env engine gets batch/solo parity for nothing; our GPU path has to
*earn* it. A rung where the primitive competitor passes trivially and we might
not is more credible than the reverse, and it is the honest shape of this claim.

**Cost.** 3 K-values × 6.5 s plus one solo reference. **Nightly.**

**Dependencies.** Amendments A, B, C, E; arm work (an `OMNISIM_INENGINE_PYMOD`
module). **No engine work now known to be required** — see §7, W1.

---

### 5.8 Rung 16 — one policy, two execution paths

**Claim.** A fixed policy artefact produces the same trajectory whether it is
executed inside the batched rollout or by a deployed controller.

**Why this shape and not "train, then deploy".** Training adds a learning run, a
seed, a checkpoint, hours of GPU time and a result that is a distribution rather
than a number — none of which is the property anyone needs. The property is
*execution parity*, and it is measurable with a policy nobody trained.

**Scene.** Rung 6's rover — drive, stop at a sensed threshold — driven by a
**hand-written, contract-owned policy**: a linear map from `(range, wheel_rate)`
to a wheel command, coefficients in `rungs.py`, exported once to ONNX by a
contract-owned script so both paths execute **the same bytes**. Run twice: once
in the batched rollout, once in the deployed controller.

**Ground truth.** Both paths must satisfy rung 6's analytic stopping bound
(`RUNG6_STOP_BOUND`, from latency plus friction-limited braking), and must agree
with each other.

**Assertions.**

| check | expected | tol | derivation |
|---|---|---|---|
| `policy_stop_train` | `RUNG6_STOP_GAP − RUNG6_STOP_BOUND/2` | `RUNG6_STOP_BOUND/2` | rung 6, unchanged |
| `policy_stop_deploy` | same | same | rung 6, unchanged |
| `parity_max_delta` | 0.0 | **1 mm** | max over the whole run of \|x_train − x_deploy\|. Not bitwise (two code paths); 1 mm is `PENETRATION_TOL`, the smallest length this ladder asserts, and 29× inside rung 6's own stopping budget |
| `parity_action_delta` | 0.0 | 1e-6 | the *commanded* wheel rate, step for step. The observation pipeline is where parity actually breaks — scaling, ordering, a stale frame — and this sees it one step later instead of 2 s later in the pose |
| `policy_hash_match` | 0.0 | 0.0 | sha256 of the artefact each path loaded. A parity claim about two different files is not a parity claim |

**Whole-run invariants.** `parity_max_delta`, `parity_action_delta`.

**What would still pass while broken.**

* **Two paths that both ignore the policy** — a constant-action controller —
  agree perfectly. Closed by `policy_stop_*`: the policy must actually stop the
  rover at the analytic gap, which a constant action cannot do.
* **A "deploy" path that is secretly the training path** passes everything.
  Closed by amendment A's distinct-`pid` rule and by `policy_hash_match` being
  read from two independent loads.
* **Declared:** this proves *execution* parity, not that a **learned** policy
  transfers. `docs/developer/closed-loop-chaos-diagnostic.md` already splits that
  question into pipeline-parity and durability; this rung is the pipeline half
  and must say so rather than imply the other.

⚠️ **A parity claim must never be quoted without an artefact-loaded assertion.**
This tree has a measured case where a missing `onnxruntime` silently ran a
zero-residual baseline and exited 0 — assert the load, never the exit code. That
is what `policy_hash_match` is for.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `obs_scale` — the range observation × 1.05 in the deploy path only | `parity_action_delta`, `policy_stop_deploy` | `policy_stop_train`, `policy_hash_match` |
| `stale_obs` — the deploy path feeds the previous step's observation | `parity_max_delta` | `policy_hash_match` |
| `wrong_artefact` — the deploy path loads a different export | `policy_hash_match` | `policy_stop_train` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | **BLOCKED on rung 17's gap, not on rung 15's** | the batched path exists (F10) but has **no sensors** (F11), and this scene's policy consumes a range reading. Either rung 16 waits for W2, or its first version uses a **proprioceptive-only** policy (wheel rate → wheel command) and drops the sensed stop — which is a weaker but still honest first cell, and should be labelled as such | — |
| `mujoco` | PASS | one model, two drivers | medium |
| `webots` | **`N/E`** | no in-engine policy runtime and no batched path: both "execution paths" would be the same one, so the check would be vacuous. **A vacuous green is worse than a refusal** | medium |

**Cost.** 2 × 8 s. **Nightly.**

**Dependencies.** Amendments A, B; rung 15's arm work; a contract-owned ONNX
export (~40 lines, regenerable byte-identically); W2 for the sensed variant.

---

### 5.9 Rung 17 — sensors in the batched path (predicted failing)

**Claim.** A robot in a batched rollout can sense its own environment and act on
what it sensed.

**Scene.** Rung 6, batched: K envs, each a rover approaching a wall at a
**per-env** distance, each stopping when its own range sensor crosses its own
threshold.

**Ground truth.** Per env, rung 6's stopping bound against that env's wall
position — analytic, per env, and *different* per env, so an engine that returns
one env's reading for all of them lands K−1 reds.

**Assertions.** `sensor_batch_stop_worst` (worst over envs against each env's own
analytic gap, tolerance `RUNG6_STOP_BOUND/2`); `sensor_batch_min_gap` (rung 6's
whole-run minimum, per env); `sensor_batch_distinct` (the number of distinct
readings across envs at a given step must be K, tolerance 0); `envs_seen`.

**Whole-run invariant.** `sensor_batch_min_gap`.

**The mechanism, corrected.** The first draft of this plan repeated "the batched
path has `nsensor = 0, ncam = 0`" as a measurement. It is not one (F11): **no run
in this tree has printed those counters**, and `OMNISIM_NEWTON_DUMP_MJMODEL` does
not even emit them. What is *source-cited* is stronger and more useful:
**OmniSim's sensors are served outside MuJoCo altogether** — by C++/Python
readback of body state plus an `mj_ray` service — `grep add_sensor src/` returns
nothing, and the committed MJCF export carries no `<sensor>` and no `<camera>`.
So a batched env does not have a *degraded* sensor; it has **no sensor service at
all**, because the service is a per-world CPU path the batch never enters.

That materially changes the size of the fix. It is not "emit a `<sensor>`
element": it is either (a) emit MJCF sensors so mujoco_warp computes them per
world, which moves the sensor model into MuJoCo and would need parity against the
existing service, or (b) batch the ray service over K worlds. **Both are real
engine work and neither is a one-liner** (§7, W2).

**What would still pass while broken.** An engine that computes each env's range
*from the pose* rather than casting a ray passes everything — rung 5's declared
limit, inherited. And an engine whose envs all read env 0's sensor passes the
stop check if the walls were at equal distances, which is why they are not.

**Expected verdicts.**

| arm | prediction | reasoning |
|---|---|---|
| `omnisim` | **RED expected, and including it is the point** | F11. The rung cannot even reach its assertions if no env has a sensor, and the honest first row is `NULL ⇒ RED`, never `N/E` — the capability is claimed by the product, so a refusal would be the wrong verdict |
| `mujoco` | **UNVERIFIED** | ray-based sensors under `mujoco_warp` / MJX may be unimplemented. The arm must find out and record `CITED` or `MEASURED`, never assume |
| `webots` | PASS, trivially | K processes, each an ordinary rung 6 |

**A tier engineered to green is not a roadmap.** This rung is in the plan
precisely because we fail it, and its first green — if it comes — is a genuine
capability change rather than a tuning. It should be built **after** rung 15,
because it cannot be interpreted before it.

**Cost.** K × 8 s. **On demand** until it passes.

**Dependencies.** Rung 15's arm work, plus engine work W2.

---

### 5.10 Rung 18 — a closed kinematic loop (predicted failing)

**Claim.** A mechanism whose links form a closed loop moves the way the loop's
geometry says, and stays closed while it does.

**Scene.** A **slider-crank**: a crank of radius `r = 0.10 m` driven at
`ω = 2.0 rad/s` (= `RUNG3_OMEGA_CMD`) about a horizontal axis, a connecting rod
`l = 0.30 m`, and a slider on a horizontal prismatic track. **The loop is closed
between the rod and the slider — two DYNAMIC bodies — deliberately**: F12 records
that `OmBasicJoint::setJoint()` returns false unless the reference
`pointsToStaticEnvironment()`, so a loop closed against the *world frame* may
well work while the case every real mechanism needs does not. The rung must
exercise the case that matters. 13 s ≈ 4 revolutions.

**Ground truth — closed form, exact, and recognisable to any mechanical
engineer:**

```
x(θ) = r cos θ + √(l² − r² sin²θ),      θ = ω t
x_max = r + l = 0.40 m      (θ = 0)
x_min = l − r = 0.20 m      (θ = π)
stroke = 2r = 0.20 m        exactly, and independent of l
```

**Assertions.**

| check | expected | tol | derivation |
|---|---|---|---|
| `slider_track_err` | 0.0 | 2 mm | max over the **whole run** of \|x_measured(t) − x(ωt)\|. 1 % of the stroke and ~4× a plausible constraint residual; the trajectory is pure kinematics with no contact and no integrator error of consequence at 2 rad/s |
| `slider_stroke` | 0.20 m | 2 mm | exactly 2r, independent of the rod length — so a mechanism with the rod wrong and the crank right is still caught |
| `loop_residual` | 0.0 | 1 mm | max over the whole run of the distance between the two points the closure constrains. This is the classic failure of loop closure in a maximal-coordinate solver: the constraint drifts and the mechanism comes apart while every body still moves plausibly |
| `crank_angle` | `ω T` = 26.0 rad | `omega_tol(ω) × T` | rung 3's motion floor — and see below |

**Whole-run invariants.** `slider_track_err`, `loop_residual`, `slider_stroke`.

**What would still pass while broken.** **A world with no physics has a loop
residual of exactly zero.** Every body sits where it was authored, the closure is
perfect, and the constraint check is green. `crank_angle` is the motion floor
that closes it, and this rung is the cleanest illustration in the ladder of why
every rung needs one.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `no_closure` — the loop-closing constraint removed | `slider_track_err`, `slider_stroke` | **`crank_angle`** (the crank still turns), `loop_residual` (nothing left to violate) |
| `locked_crank` — the crank motor cut | `crank_angle` | `loop_residual` — the frozen-world proof |
| `rod_short` — the connecting rod authored 10 % short | `slider_track_err` | `slider_stroke`, `crank_angle` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | **RED expected — and the predicted signature is precise: the honest run should be INDISTINGUISHABLE from the `no_closure` fault.** `crank_angle` green (the crank is an ordinary motorised hinge), `slider_track_err` and `slider_stroke` red, `loop_residual` meaningless because there is no constraint to violate | F12: `collectSolidChildren` appends a joint's endpoint only `if (j->solidReference() == NULL)`, and `setJoint()` refuses any reference that is not the static environment — so the closing joint is **silently never registered**, and the capability gate does not list `SolidReference` as a reason, so no FATAL fires. ⚠️ **The behavioural claim is UNVERIFIED**: the only shipped closed-loop sample, `coupled_motors.omniworld`, was recorded as `ERROR` — explicitly "not as a pass" — and never produced a document | medium-high on the mechanism; the *measurement* is exactly what this rung is for |
| `webots` | PASS | `SolidReference` is upstream's own loop-closure primitive and predates the fork | medium-high |
| `mujoco` | PASS | `equality` constraints (`connect`) express closed loops natively | high |

We would be **the only arm that fails**, on a capability the engine we forked
from has, and there is no way to present that as anything else.

**Note on the shape of the red, and on what `crank_angle` is FOR.** "The honest
row equals the `no_closure` fault" is the strongest form a defect report can take
in this ladder: it says the engine behaves exactly as if the feature had been
deleted from the scene.

It also **changes what the motion floor is doing**, and this is worth being
explicit about because the brief's version of the mechanism implied the opposite
(§00 correction 2). Under a "the world runs with no physics" mechanism,
`crank_angle` would be the check that *catches* the defect. Under the real
mechanism — the loop joint silently dropped, everything else stepping normally —
`crank_angle` is a **must-GREEN companion on the honest run**, and its job is to
stop the red being misread as "nothing ran". The floor is mandatory either way;
what it is for is different, and a rung that got that backwards would have
reported the wrong diagnosis with the right colour.

**And the red is actionable, which the briefed mechanism was not.** A closed loop
cannot be a tree edge, so the fix is not "stop poisoning the articulation" — it is
to represent the closure as an **equality constraint outside the articulation**,
which is exactly `equality connect` in MuJoCo/mujoco_warp. AGENTS.md records
welds (`Connector` / `VacuumGripper`) as **native** on the Newton path, so that
machinery plausibly already exists and the gap is that `SolidReference` is not
routed to it (§7, W5 — status UNVERIFIED, and the first thing to check).

**Cost.** 13 s, 4 bodies, no contact. Cheapest rung in the plan.
**Every commit.**

**Dependencies.** None **to measure** — this is the one tier-E rung that needs no
amendment and no engine work to produce its row. W5 is what it would take to
*pass*, and naming it is part of the rung's value.

---

### 5.11 Rung 19 — agreement with recorded reality

**Claim.** Replaying a recorded real-world impact reproduces the real trajectory
about as well as the engine we embed does.

**Scene.** `omnibench/lane1r`'s dataset: an acrylic cube tossed onto a wooden
table, AprilTag-tracked at 148 Hz, 550 trajectories, each with a full initial
condition **including both velocities**. The ladder uses a fixed, contract-owned
subset — **indices 0–49** — replayed from row 0 and compared to the recording.

**Ground truth (amendment F).** The recording, plus the published baselines:
Drake 13.5 ± 8.2 %, Bullet 14.9 ± 8.9 %, MuJoCo 25.1 ± 10.8 % of cube width
(Acosta, Yang & Posa, RA-L 2022).

**Assertions — the pass criterion is the part that matters.**

| check | expected | tol | derivation |
|---|---|---|---|
| `real_pos_err` | ≤ 35.9 % of cube width | one-sided | the published MuJoCo baseline **plus one published standard deviation** (25.1 + 10.8) — an externally anchored bound set by a paper that never heard of this project. **It is not "we win"**: the best engine in the world scores 13.5 % on this data |
| `real_rot_err` | ≤ 43.1° | one-sided | same construction (21.7 + 21.4) |
| `embed_gap` | \|our error − the **bare MuJoCo arm's** error on the same 50 tosses\| ≤ 5 % of cube width | | ⭐ **the translation-fidelity assertion.** Our solver *is* MuJoCo, so a gap between our arm and the bare arm is our layer, not the physics. §0.1's non-degradation claim in one number, and a check we can only fail — never win |
| `no_tunnel` | 0.0 | 1 mm | whole-run: the cube's lowest corner never passes below the table top on any toss. A replay that fell through the table can still score a plausible mean error |
| `replay_ic_fidelity` | 0.0 | 1 % | the trajectory's velocity over the first three samples against row 0's recorded velocity. **This exists because this tree has shipped `setVelocity` being silently dropped at t = 0** — a defect that would otherwise present as "poor real-world agreement" |

**Whole-run invariants.** `no_tunnel`; and the error statistics are taken over
every sample of every toss rather than at a final horizon.

**What would still pass while broken.**

* **An engine that reproduces inelastic impacts and nothing else** scores well
  here. lane1r already records Acosta's finding that every engine handles
  inelastic impacts and all of them fail on elastic ones. Declared, not closed —
  and the sentence travels with the number wherever it is quoted.
* **A better score is not evidence of a better simulator.** The
  Drake > Bullet > MuJoCo ordering on rigid impacts *inverts* on cloth. lane1r
  says this; the rung inherits it verbatim.
* **We are the author and a contestant.** `embed_gap` is the only check here we
  cannot flatter ourselves with, which is why it is the headline and the other
  two are floors.

**Fault battery.**

| fault | must go RED | must stay GREEN |
|---|---|---|
| `ic_drop_velocity` — row 0's velocities not applied | `replay_ic_fidelity`, `real_pos_err` | `no_tunnel` |
| `scale_uncorrected` — the 2.2 % tracked-length scale factor left in | `real_pos_err` | `replay_ic_fidelity` |
| `table_hologram` — the table loses its collider | `no_tunnel` | `replay_ic_fidelity` |

**Expected verdicts.**

| arm | prediction | reasoning | confidence |
|---|---|---|---|
| `omnisim` | PASS on the floors; **`embed_gap` is the row** | we embed MuJoCo, so a large gap is a defect in our translation layer and is exactly what the run is for | medium |
| `mujoco` | PASS by construction on the floors | it *is* the baseline; its value here is to be our reference, not to be graded | high |
| `webots` | genuinely UNKNOWN | ODE against this data has not been measured by us or, as far as this plan knows, by anyone. A publishable independent number either way | low — which is why it is worth running |

**Cost.** 50 tosses × ~1.5 s of sim, dominated by 50 process launches: lane1r
measured ~8 s wall per toss ⇒ **≈ 7 minutes per arm**. **On demand / release**,
never on a commit.

**Dependencies.** Amendments A and F; reuse of `lane1r/dataset.py` (external to
the ladder, so no arm-collision hazard) and its calibration probe. **No new
ground truth may be authored here** — if the ladder and lane1r disagree about the
cube's inertia, lane1r is right.

---

## 6. What to build first, and why

**Order: 18, 9, 11, 10, 12, 15, 13, 14, 16, 17, 19.**

1. **Rung 18 (closed loops) — first.** Cheapest rung in the plan (13 s, four
   bodies, no contact), **no contract amendment and no engine work** to measure,
   a closed-form ground truth, and it converts a source-level defect that has
   never been measured (F12) into a number. It also demonstrates the motion-floor
   rule better than anything else here: the broken state has a *perfect*
   constraint residual.
2. **Rung 9 (determinism) — second.** Cheap, and it forces amendment A, which
   four later rungs need. Everything this tree publishes about training, parity
   and reproducibility rests on a determinism claim that currently lives in a
   document; this puts it in the table with a sensitivity control beside it so it
   cannot be satisfied by a frozen world.
3. **Rung 11 (scale fidelity) — third.** It resolves an *open* question instead
   of adding one. Lane 4b could not force the `njmax` overflow and honestly
   reported `cliff_detector_validated: false` (F3); rung 11 puts a second
   instrument on the same question, one already validated at N = 5. Either it
   reproduces the cliff — settling a documented threshold — or it does not, in
   which case **two** independent instruments have failed to and the ~9-robot
   figure should stop being repeated as fact.
4. **Rung 10 (600 s) — fourth**, gated on probe **P1**. If a motorless Newton
   hinge is not passive (F4), P1 is a ten-line finding worth having on its own,
   and it decides whether rungs 10 and 12 measure what they say.
5. **Rung 12 (import fidelity) — fifth.** The pendulum exists by then, so the
   marginal cost is a URDF emitter. Strongest non-degradation claim available to
   us, and F15 says this exact defect class has shipped twice.
6. **Rung 15 (batched fidelity) — sixth, and it moved up.** F10 shows the batched
   path is reachable from a live `.wbt` through a documented hook, so this is arm
   work rather than the engine project the brief assumed. It is the only rung in
   the plan that enters a category upstream Webots cannot, and it is the
   precondition for interpreting 16 and 17.
7. **Rungs 13 and 14 (devices, radio)** — cheap, every-commit coverage; 13 puts
   a number in newtons on a break the guide and the matrix disagree about (F5).
8. **Rungs 16 and 17** — 16 needs an ONNX export and can ship a weaker
   proprioceptive-only first cell; 17 needs W2 and is the roadmap row.
9. **Rung 19** whenever there is a release to characterise. Most externally
   credible row in the plan, most expensive, and it does not belong on a commit
   hook.

**If only four rungs are ever built, build 18, 9, 11 and 12.** Between them they
cost under two minutes of simulated time, need one contract amendment, and cover:
a capability we lost against the engine we forked from; the reproducibility claim
everything else rests on; the scale question two other lanes have left open; and
the one claim we can make against MuJoCo without overreaching.

---

## 7. Cost, cadence, and the work that does not exist yet

### 7.1 Cost

| rung | sim seconds | steps | bodies | cadence | dominated by |
|---|---|---|---|---|---|
| 9 | 3 × 8 | 6,000 | 26 | every commit | 3 process launches |
| 10 | 600 | 150,000 | 5 | nightly (60 s smoke per commit) | stepping |
| 11 | 5 × 6.5 | 8,125 | up to 160 | N ≤ 16 per commit; 32 nightly | the N = 32 cell; on the GPU variant, see F3 — rovers were below realtime at N = 1 |
| 12 | 2 × 20 + 6.5 | 11,625 | 3 | every commit | 3 launches |
| 13 | 3 | 750 | 4 | every commit | launch |
| 14 | 6.5 | 1,625 | 10 | every commit | launch |
| 15 | 3K × 6.5 + solo | K-scaled | K × 5 | nightly | the batched path |
| 16 | 2 × 8 | 4,000 | 5 | nightly | ONNX load |
| 17 | K × 8 | K-scaled | K × 5 | on demand | the batched path |
| 18 | 13 | 3,250 | 4 | every commit | launch |
| 19 | 50 × 1.5 | 18,750 | 2 | on demand | **50 process launches ≈ 7 min/arm** |

Every-commit total: **≈ 40 s of simulated time per arm**, the same order as rungs
0–8 today. The 600 s rung and the 50-toss rung are the only two that cannot be,
and both are marked.

**A cost the table does not show.** The Webots arm runs R2025a under WSL2, where
AGENTS.md records asset-heavy cold loads at 46–79 s. For these small scenes that
is launch overhead rather than stepping — but "cheap in steps" and "cheap in wall
clock" are not the same statement on that arm, and the cadence should be read
per-arm.

### 7.2 Work this plan depends on

| id | work | needed by | size |
|---|---|---|---|
| **P1** | *probe*: does a Newton `HingeJoint` with no motor swing freely? | 10, 12 | ten lines |
| **W1** | *arm work, not engine work* (F10): an `OMNISIM_INENGINE_PYMOD` module that seeds K worlds via `_mpc_rollout_buffers` / `_mpc_seed_from_live`, steps them, and records per-env body poses into the sample document. Declares `MPC_NJMAX`/`MPC_NCONMAX` under R4 and E | 15, 16, 17 | a few hundred lines in `omnisim/` |
| **W2** | **engine work**: give a batched env a sensor service at all. Either emit MJCF `<sensor>` elements so mujoco_warp computes them per world (which moves the sensor model into MuJoCo and needs parity against the existing service), or batch the `mj_ray` service over K worlds. **Not a one-liner** — F11 shows the sensors live entirely outside the MuJoCo model | 17, and 16's sensed variant | the tier-E gate |
| **W3** | export `analysis.reduce_samples` + `rungs.check_rung` as AgentBench graders (§4.3) | the agent claim | small, and it replaces a whole rung |
| **W4** | machine-attributed `sim_seconds_per_wall_second` on every cell (§4.1) | the throughput claim | small; the timing plumbing exists |
| **W5** | **engine work**: route a `SolidReference` loop closure to an **equality constraint outside the articulation** (`equality connect`) instead of dropping the joint at `collectSolidChildren` / `setJoint`. A closed loop cannot be a tree edge, so no amount of work inside the articulation builder can fix it | 18, **to pass** (not to measure) | unknown until the first check: AGENTS.md records welds as native on the Newton path, so if that path already builds mjEQ constraints this is a **routing** change rather than new machinery |

**The brief's biggest assumption was wrong in our favour and wrong against us at
the same time.** The batched *rollout* is reachable today (W1 is arm work), and
the batched *sensor* path does not exist at all in a deeper way than "no
`<sensor>` element" (W2 is a real project). Rung 15 therefore comes forward and
rung 17 goes back.

---

## 8. What tiers C–E still will not prove

Written here so it does not have to be discovered later, in the spirit of
`CONTRACT.md` §3a.

1. **Nothing here proves propulsion.** Rung 4's declared limit is inherited by
   11, 14, 15 and 17: wheels are shown kinematically consistent with the motion,
   never to have caused it.
2. **Nothing here distinguishes a real ray from a correct bookkeeping computation
   of the same quantity.** Rung 5's declared limit is inherited by 13 and 17.
   Rung 13 *narrows* it for accelerometers — a pose-derived "acceleration" reads
   −g in free fall, not 0 — and closes it for nothing.
3. **600 s is not a day.** Rung 10 bounds drift rates over ten minutes.
4. **Determinism is scoped to one machine and one build.** Rung 9 says nothing
   about cross-machine identity, which is untested — and a sibling census already
   found **56 of 180 lane-1 cells differing between two machines, 13 at relative
   error ≥ 1e-3** (F17). A rung 9 green on one box is not a claim about two.
5. **Rung 19's score is not a ranking.** A better number on tossed cubes does not
   generalise; the ordering inverts on cloth. The only claim rung 19 licenses
   about us is `embed_gap`.
6. **No rung in tiers C–E measures speed**, and no green here may be quoted in a
   sentence containing "faster" (§4.1).
7. **A `NOT_EXPRESSIBLE` verdict is never a defeat for the arm that returns it**
   — `buildbench` SPEC §0.1, adopted whole. Rungs 12, 14, 15 and 16 each carry at
   least one, and rung 17's expected red is *ours* and is deliberately **not**
   an `N/E`: the capability is claimed by the product, so a refusal would be the
   wrong verdict.
8. **Every prediction in §5 is a prediction.** Eleven arm-verdict cells in this
   file say PASS or RED before anything has run. The value of writing them down
   is that they can be wrong in public; the risk is that they become expectations
   that shape the build. Amendment C — a green that cannot be made red is not a
   pass — is the mitigation, and it is weaker than a pre-registration.
