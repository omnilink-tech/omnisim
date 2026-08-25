# Product findings from the first capability-ladder agent cell (2026-08-02)

**Status: findings ON THE RECORD, not fixes.** Nothing in this file has been
repaired. It exists because the ladder's first real agent cell — an autonomous
Claude Code session given one sentence, a full OmniSim workspace and no help —
spent 38 minutes, 63 tool calls and $6.36 finding three things about *this
product* that our own tests, docs and demo headers do not say. The cell itself
was **INVALID for instrument reasons** (see
[§5](#5-the-cell-that-produced-these-findings-was-invalid)); these findings are
independent of that, because each one is a fact about the tree that a reader
can re-derive from the source without re-running any cell.

- **Cell**: `tests/benchmarks/ladder/results/ladder_cell/20260802_155111_omnisim_T2/`
- **Task**: `T2_transfer` — *"Here is an arm, a block and a bin; make the arm
  put the block inside the bin, and prove it held the block for ten seconds on
  the way."*
- **Column**: `omnisim`. **Machine**: `9722d23d12a3` (RTX 3060 laptop, Windows 11).
- **Evidence**: `transcript.jsonl` (228 entries), `cc_stdout.json`,
  `repo_artifacts/` (the session's own probe worlds, sweep script and result
  files, preserved).

The session's own summary, verbatim from `cc_stdout.json`, is the shortest
statement of findings (a) and (b):

> 1. **The bridge's grasp is a kinematic weld** (`_attach_nearest` + a per-tick
>    `setSFVec3f(tcp)` teleport), despite three comments claiming friction. […]
> 2. **`WorldInfo.contactProperties` is inert under Newton.**
>    `coulombFriction [ 5 ]` is read only by `OmSimulationCluster.cpp` — the ODE
>    path. The Newton backend uses `default_shape_cfg.mu =
>    OMNISIM_NEWTON_GROUND_MU`, default **1.0**. So this world (and its
>    siblings, whose headers reason about "high contact friction … keep the grip
>    stable") has been running at μ=1.0, not 5. My softCFM sweep returned
>    bit-identical numbers across three orders of magnitude, which is what
>    exposed this.

> **⚠ 2026-08-08 — finding (b) got STRICTLY WORSE, not stale.** The quote above is verbatim
> and unedited. But `bdc02139` deleted the vendored ODE library, so the *one* consumer of
> `WorldInfo.contactProperties` — `OmSimulationCluster::fillSurfaceParameters` filling an ODE
> `dContact` — is **gone**. `coulombFriction`, `softCFM`, `softERP`, `bounce`,
> `forceDependentSlip` and `rollingFriction` are now read by **nothing, on any path**: they
> parse, they validate, and no code anywhere consumes them. The escape hatch is unchanged
> (`OMNISIM_NEWTON_GROUND_MU`, process-wide). Record:
> [ode-retirement-campaign.md](ode-retirement-campaign.md).

---

## (a) The arm bridge's "grasp" is a kinematic weld, and its own comments say friction

### What was measured

`omnilink_arm_bridge` — the shipped HTTP control surface for **every** arm in
the tree (`omnilink_omniarm6`, `omnilink_ur3e`, `omnilink_ur10e`,
`omnilink_multi_arm`, the OmniArm 6 flagship demos) — does not hold objects with
contact. It **welds** them:

| what | where |
|---|---|
| the weld itself | [`projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py:4316`](../../projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py#L4316) — `def _attach_nearest(self, tcp)`, docstring: *"Weld the nearest GRASP_ object within grasp_radius to the tool."* |
| the per-tick teleport | same file, **lines 2156–2167** — `# Kinematic-attach: teleport the held object to the TCP each tick.` → `self.held_tfield.setSFVec3f(tcp)` + `self.held_node.resetPhysics()` |
| the weld on the **`physics_grasp`** path too | same file, **lines 4369–4377** — `if (self.gripper_cfg or {}).get("physics_grasp"): … out["attached"] = self._attach_nearest(tcp)` |
| the comment that claims friction | same file, **lines 4362–4368** — *"the real fingers close on the part and **contact friction does the grabbing** (visible pinch). A position-servo pinch alone slowly creeps under gravity in the MuJoCo contact over a long hold, so for a reliable interactive hold we ALSO attach the gripped part to the tool"* |
| a shipped world repeating the claim in its header | [`projects/samples/demos/worlds/flagship/omniarm6_physics_pick_place.omniworld:7`](../../projects/samples/demos/worlds/flagship/omniarm6_physics_pick_place.omniworld#L7) — *"CLOSES THE FINGERS ON THE CUBE (**contact friction holds it — no kinematic weld**)"* |

The weld is not a fallback. `physics_grasp` is the *only* mode that claims
friction and it takes `_attach_nearest` unconditionally; the two paths differ
only in what they put in the JSON response. The capture is by **DEF name**
(`_iter_graspables` at line 4289 yields only nodes whose DEF starts with
`GRASP_`), so the mechanism is invisible from the scene: an object named
`GRASP_CUBE` is held by a teleport, an identical object named `CUBE` is not
held at all.

The consequence the agent stated and then engineered around: **under a weld,
"the gripper held the block for 10 s" is unfalsifiable — it would hold with the
fingers wide open, forever.** Its own world
(`repo_artifacts/…/worlds/flagship/omniarm6_block_in_bin.wbt`) names the block
`DEF BLOCK` *specifically so the weld cannot fire*, and says so in its header.

### What the honest version measured

Driving the finger motors directly, with no weld, on the OmniArm 6 + Robotiq 2F-85
under `newtonSolver "mujoco"`
(`repo_artifacts/…/controllers/omniarm6_block_in_bin/_block_in_bin_result.json`):

```
in_bin        : true          (the transfer itself works)
held_s        : 1.376         (required 10.0)
max_slip_mm   : 313.08
never_welded  : true
hold_breaks   : ["t=10.58s after 1.39s: slip=15.0mm>15mm"]
```

A real friction pinch of this block on this gripper holds for **1.4 s**. The
shipped demos hold it forever, because nothing is gripping.

### How to reproduce

```bash
# the weld, statically
grep -n "_attach_nearest\|Kinematic-attach\|contact friction does the grabbing" \
  projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py

# the weld, dynamically: run any omniarm6 demo, then rename the part's DEF from
# GRASP_* to anything else and re-run. The grasp stops working entirely --
# which it would not, if fingers and friction were doing it.
python -m omnisim run-headless \
  projects/samples/demos/worlds/flagship/omniarm6_physics_pick_place.omniworld --duration 30
```

### Blast radius

- **Every arm demo in the tree.** The bridge is the arm surface: seven chat
  worlds plus eight OmniArm 6 flagship worlds route through `act_grasp`.
- **Anyone building on `PROTOCOL.md`'s arm verbs.** `act_grasp` is a documented
  bridge action; a user who writes a manipulation policy against it is writing
  against a teleport, and their policy will not transfer to a real gripper or to
  a friction-based simulator.
- **Any measurement of grasp quality taken through the bridge is vacuous** —
  including, if it were ever used, a ladder T2 cell. The tier's
  `hold_mechanism` field exists precisely to record this distinction and would
  read `attachment` here.

### Does it affect a published claim?

**Yes, in the world files and in the controller's comments** — three separate
places assert friction where the code welds, and one of them
(`omniarm6_physics_pick_place.omniworld:7`) explicitly says *"no kinematic weld"*.
`AGENTS.md` and `DEMOS.md` describe these worlds as pick-and-place demos without
claiming a grasp mechanism, so the top-level docs are not wrong; the
world headers are. **T2's `meta.json` already pre-registers the honest version
of this** (`hold_mechanism.why_not`: *"an earlier carry demo used a
grasp-stabilisation weld"*), so the ladder is not surprised — the demos are.
Fixing this is a *labelling* decision at minimum (say "attachment") and a
physics decision at most (make the pinch hold).

---

## (b) `WorldInfo.contactProperties` is read only by the ODE path and is inert under Newton

> **⚠ 2026-08-08 — heading superseded: it is now read by NOTHING.** `bdc02139` deleted the ODE
> library, i.e. the only reader. The finding below is not obsolete — it is worse. Read "the ODE
> path" throughout this section as "the path that no longer exists".

### What was measured

`ContactProperties` — `coulombFriction`, `softCFM`, `softERP`, `bounce`,
`forceDependentSlip`, `rollingFriction` — had exactly **one** consumer in the
engine, and it was the ODE collision callback (⚠ **deleted in `bdc02139`, so the
consumer count is now ZERO**):

| what | where |
|---|---|
| the only read of `coulombFriction` (⚠ **all four symbols deleted in `bdc02139`** — `fillSurfaceParameters`, `dContact`, `OmOdeContact`, `OmOdeContext`; the file:line link is a historical citation and will not resolve) | [`src/omnisim/engine/OmSimulationCluster.cpp:240,247`](../../src/omnisim/engine/OmSimulationCluster.cpp#L240) — `frictionSize = cp->coulombFrictionSize(); … mu[j] = cp->coulombFriction(j);` inside `fillSurfaceParameters(...)`, which fills a `dContact` (ODE), via `OmOdeContact` / `OmOdeContext` |
| what Newton uses instead | [`src/omnisim/physics/OmNewtonBackend.cpp:142-143`](../../src/omnisim/physics/OmNewtonBackend.cpp#L142) (inline `newton_runtime` Python) — `self._ground_mu = float(os.environ.get("OMNISIM_NEWTON_GROUND_MU", "1.0"))` → `self.builder.default_shape_cfg.mu = self._ground_mu`; the URDF shape config at lines 444–470 reads the same env var |
| grep proof | `grep -rn "coulombFriction" --include=*.cpp --include=*.hpp src/omnisim/` returns `OmSimulationCluster.cpp` and `OmContactProperties.{cpp,hpp}` and **nothing under `src/omnisim/physics/`** |

So on the default backend a world's declared `coulombFriction [ 5 ]` is
**silently replaced by μ = 1.0**, with no warning, no log line and no field on
`/capabilities` that says so.

**The session's independent numerical proof**, before it ever read the C++: a
softCFM sweep across three orders of magnitude produced **bit-identical**
results (`repo_artifacts/…/_sweep_contact.py`, sweep output in
`transcript.jsonl`):

```
baseline     softCFM 3e-4   [probe] held=1.26s creep=19.83mm/s max_slip=277.6mm
cfm_1e5      softCFM 1e-5   [probe] held=1.26s creep=19.83mm/s max_slip=277.6mm
cfm_1e6      softCFM 1e-6   [probe] held=1.26s creep=19.83mm/s max_slip=277.6mm
cfm_1e6_erp  softCFM 1e-6 + softERP 0.9
                            [probe] held=1.26s creep=19.83mm/s max_slip=277.6mm
```

Four configurations, identical to the last digit. A field that changes nothing
across 300× is not being read.

### Which shipped worlds reason in their headers about friction they are not getting

Scanned the whole tree for `.wbt` files declaring `coulombFriction` whose
**header comments** reason about friction (line numbers are the header lines):

| world | what the header claims |
|---|---|
| [`projects/samples/demos/worlds/flagship/omniarm6_physics_pick_place.omniworld`](../../projects/samples/demos/worlds/flagship/omniarm6_physics_pick_place.omniworld) | L7 *"contact friction holds it — no kinematic weld"*; **L10 "High contact friction + small softCFM keep the grip stable"**; L13 *"A friction pinch grasp needs persistent static-friction contact"*. Declares `coulombFriction [ 5 ]`, `softCFM 0.0003`; runs `newtonSolver "mujoco"`. **Gets μ=1.0 and an ignored softCFM.** |
| [`projects/samples/demos/worlds/flagship/omniarm6_bin_picking.omniworld`](../../projects/samples/demos/worlds/flagship/omniarm6_bin_picking.omniworld) | L13 *"Newton + MuJoCo solver (friction grasp)"*; L82 *"high-friction contact"*. Same declaration, same backend. |
| [`projects/policies/research/worlds/omniquad_shadow_deploy.omniworld`](../../projects/policies/research/worlds/omniquad_shadow_deploy.omniworld) | L11 *"same friction, same contactProperties, same spawn, same solver. Do not 'improve' the…"* — i.e. a reproducibility contract written around a field the backend ignores |
| `projects/policies/research/worlds/omniarm6_grasp_train.omniworld`, `g1_stand_deploy.omniworld`, `g1_walk_deploy.omniworld`, `g1_walk_deploy_run.omniworld`, `omniarm6_declutter_train.omniworld`, `omniquad_baton_deploy.omniworld`, `omniquad_model_walk_newton_demo.omniworld` | header comments reasoning about friction alongside a declared `coulombFriction` |
| `projects/robot_combat/battlebots/worlds/battlebox_royal_rumble.omniworld` | ditto |

Totals, measured: **322** `.wbt` files in the tree declare `coulombFriction`;
**14** pin `physicsBackend "ode"`; the rest run the default path where the field
is inert. ⚠ **2026-08-08: those 14 got worse, and not in a way you can see.** Since `bdc02139` an
`"ode"` pin selects a deleted backend — but the pin still **wins**, so the worlds **still load and
still run, with no physics at all, silently**: inert stub, no FATAL, no ERROR, no warning, nothing
moves, nothing collides. So it is not just `contactProperties` that is ignored in those 14; the
whole solver is. Narrowed to shipped demos: **14** under `projects/samples/demos`, of
which **2 flagship OmniArm 6 worlds reason about friction in their headers**, and
**137** under `projects/policies` (the legged lane — which mostly compensates by
setting `OMNISIM_NEWTON_GROUND_MU` in its 40-odd launch scripts, i.e. it already
knows the field does not work, without saying so anywhere a world author would
read).

### How to reproduce

```bash
# 1. the engine, statically
grep -rn "coulombFriction" --include=*.cpp --include=*.hpp src/omnisim/
#    -> OmSimulationCluster.cpp (ODE), OmContactProperties.{cpp,hpp} (the node).
#       Nothing in src/omnisim/physics/.

# 2. the engine, dynamically: same world, two values of coulombFriction,
#    Newton default. Identical trajectories.
#
#    ⚠ THE CONTROL ARM IS UNRUNNABLE (bdc02139). "Now pin physicsBackend "ode"
#    (or OMNISIM_FORCE_ODE=1) and they diverge" was the half that showed the field
#    IS honoured somewhere. OMNISIM_FORCE_ODE now selects nothing (it is warned
#    about and ignored), and an explicit "ode" pin still WINS but resolves to an
#    inert stub -- the world loads and runs with NO PHYSICS, silently, so the
#    "control" trajectory is a body that never moves. THIS RECIPE CAN NO LONGER
#    DEMONSTRATE THE FINDING --
#    it can only show the identical-trajectory half, which alone is consistent
#    with "the field does nothing" AND with "the sweep was mis-set up".
#    The static grep in step 1 is now the only proof, and step 1's own answer
#    changed: there is no reader left at all.
```

The session's own sweep harness is preserved and re-runnable:
`…/20260802_155111_omnisim_T2/repo_artifacts/projects/samples/demos/controllers/omniarm6_block_in_bin/_sweep_contact.py`.

### Blast radius

- **Any world author tuning grip, traction or bounce through
  `ContactProperties` on the default backend is tuning nothing.** There is no
  error to notice: the field parses, validates (`OmContactProperties.cpp:139-149`
  range-checks it), and is discarded.
- **Worse for the diagnosis than for the physics**: the agent spent ~15 minutes
  and two full engine sweeps establishing that a parameter it had correctly
  identified was doing nothing, before finding out from the C++ that it could
  not. That is exactly the failure mode
  [`docs/developer/tool-design-for-agents.md`](tool-design-for-agents.md)
  warns about — the world file returns the value it was *asked for*, so the
  agent forms a false belief and reasons confidently from it.
- **The escape hatch is a process-wide env var, not a per-world field.**
  `OMNISIM_NEWTON_GROUND_MU` is global: two worlds in one parallel batch cannot
  have different friction, and nothing in the `.wbt` records what a run used.

### Does it affect a published claim?

**Yes — a documentation claim, by omission.**
[`docs/reference/worldinfo.md:150`](../reference/worldinfo.md) documents
`contactProperties` with no scope note, while **every** Newton-specific field
immediately below it (`newtonSolver` L158, `newtonSubsteps` L159, `newtonCone`
L160, `newtonImpratio` L161, `newtonNjmax` L162, `newtonStatics`,
`newtonRobotColliders`, `newtonCompoundColliders`) carefully ends with *"Ignored
on ODE-backed Solids"*. The one field that is ignored on the **only** backend
(⚠ it read "the **default** backend" before `bdc02139` deleted the other one)
is the one with no disclosure. The minimum honest fix is one sentence in that
doc; the real fix is plumbing `ContactProperties` into the Newton shape config.
Note the ODE-scope sentences on the `newton*` fields are now themselves vestigial —
there are no ODE-backed Solids.

It does **not** invalidate any OmniBench number: lane 1's `t*_odepin` worlds pin
ODE, and the Newton-side runs never claimed to be honouring these fields.
⚠ **2026-08-08:** those `t*_odepin` worlds **still load and still score — on no physics at all**
(`bdc02139`: the explicit `"ode"` pin wins and resolves to an inert stub; no FATAL, no warning).
So **OmniBench lane 1 needs re-basing urgently**: its ODE reference arm is gone, the cross-solver
correctness comparison it existed to make is gone with it, and the `_odepin` rows will keep
producing numbers that look like measurements. Lane 1 can still run Newton against the
*analytic* ground truth; it can no longer run Newton against ODE.

---

## (c) The pattern: the default backend silently ignores several things the world file declares

Findings (a) and (b) are not isolated. Cross-referenced with the two Newton
divergences already recorded this session in
[`tests/benchmarks/ladder/READINESS.md`](../../tests/benchmarks/ladder/READINESS.md)
§4:

| what the world / API declares | what Newton does | evidence | recorded where |
|---|---|---|---|
| `WorldInfo.contactProperties` → `coulombFriction`, `softCFM`, `softERP`, `bounce` | ignored; fixed `mu = OMNISIM_NEWTON_GROUND_MU` (default 1.0) | 4 softCFM configs bit-identical across 300×; `OmSimulationCluster.cpp:240` is the only reader | **this file, new** |
| a `Solid`'s contact points, read through the supervisor contact query | query runs, `supported: true`, `error: null`, returns **nothing** | **1008 support contacts on ODE vs 0 on Newton**, same unchanged probe scene, 126 sampled steps — ⚠ **historical, unrepeatable** (see note below) | [`READINESS.md` §4](../../tests/benchmarks/ladder/READINESS.md); [`ladder/adapters/omnisim/evidence.py:382-407`](../../tests/benchmarks/ladder/adapters/omnisim/evidence.py#L382) (`NEWTON_CONTACT_BLINDNESS`) |
| `wb_supervisor_node_add_force` / `add_torque` — the applied wrench | write-only; nothing reads back what another controller applied, and contact points carry no force | T4's `applied_support` channel can only enumerate *routes*, never total a wrench; a harnessed T4 cell lands in `T4-support-unverified` and is `excluded_from_comparison` | [`READINESS.md` §4](../../tests/benchmarks/ladder/READINESS.md), [`ladder/adapters/omnisim/BRINGUP.md`](../../tests/benchmarks/ladder/adapters/omnisim/BRINGUP.md) §4 |

> **⚠ 2026-08-08 — the contact row's evidence is preserved but its contrast is gone.** The
> **1008 vs 0** pair is kept verbatim as a dated measurement; it is now **unrepeatable**,
> because ODE was deleted in `bdc02139` and there is no second backend to produce the 1008.
> What made this a *bug report* was precisely the A/B — one backend saw the contacts, the
> other did not — and that comparison can never be re-run. Two things also changed on the
> Newton side: **native contact readback is default-ON since 2026-08-07**, so a resting body
> does report its contacts and `/sim/contacts` + `/sim/grips` are no longer structurally
> blind; and the `NEWTON_CONTACT_BLINDNESS` evidence code therefore describes a fixed
> condition, not current behaviour. Re-verify it against Newton alone before quoting it.

**Stated plainly: on OmniSim's default physics backend, several things a world
file or a supervisor API declares are accepted, validated, and then ignored —
with no error, no warning, and no field anywhere that reports the substitution.**

Three properties make this a product problem and not a footnote:

1. **It is the default.** `physicsBackend "auto"` resolves to Newton wherever
   the runtime is present, and a stock `make release` bundles it
   (`BUNDLE_NEWTON ?= 1`). A user authoring an ordinary world gets this path
   without choosing it.
2. **Every instance is silent in the same way** — the declaration parses and
   validates, so there is nothing for a linter, a `run-headless` PASS, or
   `/capabilities` to catch. This is the same shape as the two defects
   `AGENTS.md` already warns about at length (the `newtonNjmax` 256 overflow
   whose only warning is discarded on Windows; the gravity field that was never
   plumbed and made *every* Newton world run at −9.81 regardless of
   `WorldInfo.gravity`, fixed in `e7b9fb11`). The gravity bug is the precedent
   that matters: it is exactly this class, it shipped, and it took a benchmark
   to find.
3. **It costs an agent more than it costs a human.** A human tuning friction
   notices nothing changed and moves on; an agent has no independent access to
   the world, so a field that returns what it was asked for installs a false
   belief it then reports confidently. Measured here: ~15 minutes and two
   engine sweeps.

### Suggested shape of a fix (not done here)

- **Cheapest, honest**: a world-build warning — *"`ContactProperties` declared
  but the Newton backend does not read it; friction is μ=…"* — plus the missing
  scope sentence in `worldinfo.md:150`. Cost: hours. Removes the silence.
- **Right**: plumb `ContactProperties` into `default_shape_cfg` / per-shape
  config in `OmNewtonBackend`. Cost: days, and it moves the physics of 308
  worlds, so it needs the same champion re-verification gate the `newtonCone`
  default flip carries.
- **A published-scope note** either way: `docs/benchmarks/determinism-scope.md`
  already scopes determinism claims per solver; the same discipline should apply
  to "which `WorldInfo` fields the backend actually reads".

---

## Two smaller observations, recorded without a claim

- **The agent never opened the tier's `container/`.** T2 stages
  `container/bench_arm/urdf/bench_arm.urdf` + `block.urdf` + `bin.urdf` into the
  workspace root, and the prompt says *"Here is an arm, a block and a bin"* —
  but names no path. The session ran `ls` at t=8 s, saw `container` in the
  listing, and never entered it; it built the scene from the product's own
  OmniArm 6 and hand-authored primitive Solids. That is a **task-design** question
  for the freeze (does the prompt point at the container?) and a `reuse_class`
  question for a human reviewer — recorded here, decided by neither.
- **The measured cost of a real T2 attempt on this column** is in
  `tests/benchmarks/ladder/cell/run_ladder_cell.py` → `MEASURED_SESSION_COST`,
  so the next reader sizes a cap from data rather than guessing.

---

## 5. The cell that produced these findings was INVALID

For completeness, because a reader will ask why a $6.36 cell with this much in
it produced no ladder result. Three instrument defects, all in
`tests/benchmarks/ladder/cell/` and all repaired the same day (see that
package's tests):

1. **The junction sweep ran before deliverable discovery**, deleting the
   session's own world out of the workspace's `projects/` junction; discovery
   then fell back to the quarantine copies and phase B was handed
   `.omnisim_probe_pyramidal_mu2.wbt` — one of six throwaway sweep variants.
   The re-run loaded 0 robots (`ERROR: Cannot open URDF file
   …/repo_artifacts/…/../../../../robots/omnisim/omniarm6/omniarm6_2f85.urdf`) and
   recorded 0 rows.
2. **The cell was labelled `not_achieved(install_failed)`** — an
   agent-attributable verdict — and its `unanswered_channels` listed all six T2
   channels as unanswerable on a column the readiness audit had certified
   **READY** four hours earlier. Both were artefacts of grading the wrong file.
3. **The session did not hit any cap.** It ended at 2266.8 s of a 5400 s cap
   because Claude Code auto-backgrounded a >10-minute foreground command and the
   agent yielded to await a notification that headless `claude -p` never
   delivers.

The row has been rewritten to `INVALID` with the instrument blocker named, and
the original verdict is preserved beside it as superseded.

---

## 6. Second cell, same day: a working grasp that the `.wbt` cannot describe

The T2 re-run (omnisim, Opus 5, 99 min, capped) reached a **working two-finger
friction pinch** — the thing this column was pre-registered to fail. Its own
README claims a 10.0 s airborne hold, 18.46 s continuous grip, block landing
dead-centre in the bin, Newton-verified by sidecar, reproduced on three cold
runs. That claim is **not graded** and is not treated here as a result; the
session was cut off 90 s later and the artefacts are recovered, not observed.

What *is* established, and matters regardless of whether the claim holds:

### (f) Five of the seven knobs a working grasp needs have no `.wbt` field

The agent's own run script exports these before the engine starts:

| knob | reachable from the world file? |
|---|---|
| `OMNISIM_NEWTON_STATICS` | **yes** — `WorldInfo.newtonStatics` |
| `OMNISIM_NEWTON_COMPOUND_COLLIDERS` | **yes** — `WorldInfo.newtonCompoundColliders` |
| `OMNISIM_NEWTON_GROUND_MU` | **no** |
| `OMNISIM_NEWTON_CONTACT_KE` | **no** |
| `OMNISIM_NEWTON_CONTACT_KD` | **no** |
| `OMNISIM_NEWTON_ITERS` | **no** |
| `OMNISIM_NEWTON_LS_ITERS` | **no** |

The five are read straight from `os.environ` with hard-coded defaults —
[`OmNewtonBackend.cpp:142`](../../src/omnisim/physics/OmNewtonBackend.cpp#L142)
(`mu` default `1.0`), `:150-151` (`ke` `2500`, `kd` `100`), `:1637-1640`
(iteration counts) — and no `WorldInfo` field is consulted on the way.

So **a world file is not a complete description of its own physics**. Hand
someone this `.wbt` and they do not reproduce the result; they get default
friction and a soft contact, and the grasp fails. That is a portability defect
in the format, not a tuning inconvenience.

It compounds finding **(b)**: the world *does* declare
`contactProperties { coulombFriction [ 5 ] }`, which is inert under Newton,
while the knob that actually sets friction is an environment variable. The
field that looks like the answer is ignored and the real control is invisible
from the file — which is exactly how an agent, or a user, ends up tuning the
wrong thing. The agent found this independently and wrote it down:
*"the `WorldInfo.contactProperties coulombFriction` is IGNORED on the MuJoCo
path — default mu is 1.0"*.

Two knobs (`newtonStatics`, `newtonCompoundColliders`) **do** have fields, and
are documented in [worldinfo.md](../reference/worldinfo.md) — the agent used
the env vars anyway while putting `newtonCone` and `newtonImpratio` in the
`.wbt`. It found some fields and not others, so discoverability is uneven even
where the field exists.

### (g) Consequence for the ladder: phase B cannot grade an env-dependent deliverable

Phase B re-runs the world **bare** — no run script, no environment. The
recovered deliverable therefore graded `FAIL 0/5`, every assertion failing on
*the motion channel has 1 rows and none belongs to 'block'*, with
`unanswered_channels` listing `object_pose` for T2.1–T2.4. The agent had named
the body correctly (`DEF BLOCK Solid { … name "block" }`), so this is **not**
the deliverable failing: without `newtonStatics` / `newtonCompoundColliders`
the floor and the multi-box bin do not collide at all and the scene degenerates
before it can be sampled.

**The regrade is therefore INCONCLUSIVE, not a failure**, and must not be
quoted as one. Recorded in
`results/ladder_cell/20260802_171846_omnisim_T2/forensics/recovered_verdict.json`.

The ladder has to pick one, before the freeze:

1. the prompt requires a self-contained deliverable (the world must carry its
   own physics), which makes (f) a hard blocker for this column rather than a
   finding; or
2. phase B honours a declared run manifest shipped beside the world; or
3. both — require self-containment, and accept a manifest as a documented
   deviation that is recorded in the row.

Until that is settled, **no OmniSim T2 cell can be graded on its merits**, and
that is our defect, not the column's.

### (h) Two engine defects the agent found that we had not

- **A single-link, free, centred-box body imported via `URDFRobot` rests on its
  own frame, not its box bottom**, sinking half its height. Measured by the
  agent: URDF block settles at **z = 0.0004** where a physically identical
  native `Solid` box settles at **z = 0.0250** (half of a 50 mm cube — correct).
  A multi-link URDF bin also let a dropped block tunnel through its floor. The
  agent's workaround was to author the props as native `Solid`s, which means
  **the ladder's own URDF props do not behave as described on this column**.
- **`setPosition()` does not reopen a motor that was put in force mode with
  `setForce()`.** Releasing requires `setForce(-N)`. Whatever the intended
  semantics, it is not documented, and the failure is silent — the gripper
  simply never opens.

Neither is in the fix queue above because neither was known before this cell.
