# Capability-ladder readiness: can a real agent cell produce a genuine verdict?

**Status: 2026-08-02. Determined by RUNNING the machinery, not by reading it.**
Every row below cites what was executed and what it returned; the commands and
their output are in §5. Nothing here is a cell, and nothing here is a result
about any simulator's *capability* — this file is about the **instrument**.

---

## 1. What "READY" means in this file

A `(tier, column)` is **READY** when all four of these hold, and each was
checked rather than assumed:

1. **the workspace stages** — the column's template builds, the tier's
   `container/` lands in it, and the answer-key quarantine comes back clean;
2. **the deliverable is unambiguously identifiable** — the column declares a
   convention in `cell/run_ladder_cell.py` → `DELIVERABLE_RULES` and the
   discovery walk can find it without guessing;
3. **phase-B grading runs the tier's own channels** — the cell runner resolves
   a phase-B runner *for that rung* and the resulting verdict carries
   `unanswered_channels: null`, i.e. nothing fell back;
4. **a red would be attributable to the agent** — a failed assertion is a fact
   about the scene the session authored, not about an instrument we never
   built.

Three labels are used, and the distinction between the last two is the whole
point of the exercise:

| label | meaning |
|---|---|
| **READY** | all four hold. A cell here can be published. |
| **BLOCKED-`<name>`** | the machinery exists and runs, but one named thing stops the verdict being genuine. The `<name>` says **whose** it is. |
| **NOT-BUILT** | we never built the instrument. This is always `scaffolding_defect_ours` and may never be published as a finding about somebody else's product. |

> **The binding rule, restated because this file exists to honour it**
> (`capability-ladder-plan.md` §4): a red assertion whose cause is our own
> missing scaffolding is **not** a capability finding about a simulator. Two
> of the blockers below are genuinely the simulator's (`no_measurement_surface`
> in the plan's taxonomy) and are labelled so; the rest are ours and are
> labelled so.

---

## 2. The matrix

| tier | omnisim | webots | mujoco |
|---|---|---|---|
| **T1 arrive** | **BLOCKED-newton_contacts_invisible** *(theirs — conditional)* | **NOT-BUILT** *(no ladder channel module)* | **BLOCKED-no_t1_phase_b_runner** *(ours)* |
| **T2 transfer** | **READY** | **NOT-BUILT** | **READY** |
| **T3 quadruped** | **BLOCKED-base_name_mismatch** *(tier open question)* | **NOT-BUILT** | **READY** |
| **T4 humanoid** | **BLOCKED-base_name_mismatch** *(tier open question)*, and every T4 cell here is `T4-support-unverified` — see `BLOCKED-no_wrench_readback` | **NOT-BUILT** | **READY** |

`gazebo` and `isaac` are roster columns in `capability-ladder-plan.md` §4 and
are **NOT-BUILT at every tier** (no template beyond the bare one, no channel
module, no oracle). They are out of this file's scope and their absence is
stated rather than omitted.

### Row by row

#### omnisim / T1 — BLOCKED-`newton_contacts_invisible` *(theirs, conditional)*

Everything else is green. The workspace stages clean (4799 files, 545
excluded, 7 redactions, `workspace_clean: true`), the deliverable convention
is `.wbt`, `phase_b_runner("omnisim","T1")` resolves `t1_run_standalone`, and a
probe deliverable graded **4/5 with T1.4 green on 1008 support contacts**.

The blocker: **the supervisor contact query returns nothing under the Newton
backend**, which is the engine's default wherever the Newton runtime is
present. One unchanged probe scene returned **1008** support contacts on ODE
and **0** on Newton — with `supported: true`, `error: null`, 126 sampled steps:
the query ran cleanly and saw nothing. `WbSolid`'s contact-point list is fed
from the ODE collision callback (`src/omnisim/physics/WbPhysicsBackend.cpp`
calls the `contact_points` smoke worlds *"ODE-specific"* in as many words) and
the Newton backend never populates it.

**Whose it is: the simulator's** — `no_measurement_surface` in the plan's
taxonomy. It is not ours (our sampler works and proves it works on the other
backend) and it is not a physics shortfall (the wheels really are touching the
floor). ⚠ **But the cell runner's own classifier cannot see that difference**:
a supplied-but-empty channel reads to `classify()` as a measured shortfall and
would be labelled `no_physics_capability`. So the sentence is now carried
**inside the channel** (`evidence.NEWTON_CONTACT_BLINDNESS`, added this
session) — a reviewer re-labelling under §3.4 has the backend, the solver and
the measured 1008-vs-0 A/B in the row.

*Cheapest path to READY*, in order of cost: **(a)** a reviewer re-label, which
costs nothing and is already provided for; **(b)** run phase B with
`--backend ode` and disclose it, which is cheap but changes the physics the
agent authored and should not be the default; **(c)** populate `WbSolid`'s
contact points from the Newton backend — engine work, out of this file's
scope, and by far the most valuable of the three because it is a gap in
OmniSim's *agent-facing* surface and not only in the grader's.

#### omnisim / T2 — READY

All four conditions hold, verified end to end on the engine. A probe
deliverable (bench_arm + block + bin + floor) graded `FAIL 2/5` with
**`unanswered_channels: null`** — every one of the eight T2 channels answered:
object mass **0.2 kg**, gravity **9.81 m·s⁻²**, container rim **0.2094 m**
attested from the bin's own subtree, one static support surface found
structurally, and **clock skew 0.0 s** between the object and end-effector
series with the end effector's orientation present. The three reds are the
scene's: the block never moved, because the probe controller does not do the
task.

Two caveats that do not block: the contact blindness above also reaches T2.2
and the grip observation (the same disclosure rides in those channels); and
`hold_mechanism` reads `unknown` when nothing holds the object, which is the
correct reading rather than a gap.

#### omnisim / T3 and T4 — BLOCKED-`base_name_mismatch` *(the tiers' own open question)*

The instrument is complete on both rungs. With the base resolvable, T3 graded
`FAIL 2/5` and T4 `FAIL 2/5`, both with **`unanswered_channels: null`**,
`support_attestation: attested`, `arena_attestation: attested`, T4
`cell: T4-unsupported`, a controller attested from 12/12 joints moving, 216 (T3)
and 200 (T4) make-and-break ground-contact transitions with the feet named, and
an arena run-up of 28.4 m / 29.1 m measured against the tier's 15 m rule. The
reds are the robot's: the probe controller is not a gait and the humanoid falls
at t = 0.0 s.

The blocker: **T3 and T4 declare the base as `base_link`, and OmniSim's URDF
importer does not produce a body of that name.** The importer folds a
description's root link into the `URDFRobot` node, which carries the name the
`.wbt` gave it. On the probe, the shipped `walker.urdf` (root link `base_link`,
robot name `walker`) produced named bodies `walker`, `hip_fl`, `thigh_fl`,
`shank_fl`, … and **no `base_link` at all** — with the root's own mass (6.0 kg)
and the subtree's (10.8 kg, exactly `robot.mass_kg_declared`) both correct on
the `walker` node. Graded with the natural name, **all five assertions report
their witness absent** and the verdict is `0/5` with the sentence *"the base
series is about 'walker', but the task names 'base_link'"*.

**Whose it is: the tier's, and it is pre-registered.** Both `meta.json` files
already carry it as `robot.open_question_for_the_freeze` — *"a column whose
converter names the base body after the robot rather than after the link will
not match"*. It is not `scaffolding_defect_ours` (the channels are built and
answer) and it is not a capability gap (the physics is fine and the masses are
right); it is an unresolved question about what the tier declares.

The adapter does what it honestly can: it substitutes the **single**
robot-class body that is not the grader's own sampler, refuses on ambiguity,
and **discloses the substitution in every base channel's citation**. The T3/T4
core then correctly refuses the substituted body because its name is not the
declared one — which is the core doing its job, and is why this is a blocker
rather than something the adapter can paper over.

*Cheapest path to READY*: **(a)** the tier owner decides — either the core
accepts an adapter-disclosed substitution, or `robot` gains an
`alternate_names` list (`walker` / `strider` are the robots' own declared
names and are in the shipped file). Either is a small, reviewable change to
files outside this workstream's scope. **(b)** Failing that, an agent that
happens to name its `URDFRobot` node `base_link` produces a fully gradeable
cell today — verified, that is exactly what the probe did — so the blocker may
simply not fire on some real cells. It must not be *engineered around* by
telling the agent, which would be answer-key leakage.

#### omnisim / T4 — additionally BLOCKED-`no_wrench_readback` *(theirs, and it caps what a T4 cell can say)*

T4 permits a support harness and requires the wrench to be **measured** rather
than asserted absent. **OmniSim has no wrench read-back**:
`wb_supervisor_node_add_force` / `add_torque` are write-only from a Supervisor
and nothing reports what another controller applied; contact points carry no
force either.

So this column's `applied_support` channel is a **structural route
enumeration**, not a total: it attests zero only by proving that no route is
open (no foreign Supervisor, no kinematic base, no parenting into another body,
no `Connector`, no physics plugin), and reports `attested=None` — the tier's
`unverified` cell, *not failed and not credited* — the moment one is. Full
argument and the five routes: [`adapters/omnisim/BRINGUP.md`](adapters/omnisim/BRINGUP.md) §4.

The consequence, and it must travel with any T4 comparison: **a scene where the
agent legitimately uses a harness lands in `T4-support-unverified` here and in
`T4-supported` with numbers on MuJoCo**, which streams `xfrc_applied` plus the
equality-constraint reaction. An unverified cell is `excluded_from_comparison`
by the tier's own rule, so a harnessed T4 attempt on this column produces a
cell that cannot be compared with anybody's. A *free-standing* attempt is
unaffected and grades normally — which is what the probe did.

*Cheapest path to READY*: engine work — a supervisor read-back for the
per-body applied wrench (the deploy hook already prints it under `CRANE_LOG`;
what is missing is a read path that does not require the harness's own
cooperation). Nothing in the ladder's scope can close it.

#### webots / T1–T4 — NOT-BUILT *(ours, at every tier)*

The workspace half is **fine and verified**: the template builds, the directory
symlink to the upstream R2025a install materialises and resolves
(`\\wsl$\Ubuntu-22.04\opt\upstream-webots\R2025a` → `bin/ docs/ include/ lib/
projects/ resources/ scripts/ webots webots-controller`), the tier's
`container/` stages, and the quarantine is clean. The deliverable convention
(`.wbt`) is declared.

What is missing is the **ladder-side channel module**:
`ladder/adapters/__init__.py` → `LADDER_CHANNELS` has entries for `omnisim` and
`mujoco` and none for `webots`, so `resolve_ladder_channels("webots")` returns
`None` and `phase_b_runner("webots", <any rung>)` refuses with *"the 'webots'
column ships no ladder-side channel module, so there is no phase-B runner"*.
Every tier therefore blocks with `scaffolding_defect_ours` before any physics
is read. Grading an *existing* run directory is possible —
`agentbench.adapters.webots.evidence.build_bundle` exists and the shim reuses
it — but phase B, which is the only thing a ladder cell may be scored from
(SPEC §2.3), is not built.

*Cheapest path to READY*, and it is the largest single item in this file:
a `ladder/adapters/webots/` mirroring the OmniSim column — a launcher over
`agentbench.adapters.webots.launcher` (WSL paths, its own recorder injection),
the ladder recorder ported to upstream's controller API, and the same channel
builders. The OmniSim column's split (a thin launcher + a pure `channels.py`)
exists partly so that port is a re-use rather than a rewrite. Estimated at the
same order as this session's T2–T4 work, plus the WSL launch path.

#### mujoco / T1 — BLOCKED-`no_t1_phase_b_runner` *(ours)*

⚠ **This one is easy to miss, and the dry run hides it.** `--dry-run --column
mujoco --task T1_arrive` reports `achieved / PASS`, because the T1 MuJoCo
*oracle* drives and records in one process: there is no separable deliverable,
the oracle hands the cell its own run directory, and the cell grades it through
`grader.grade(<run dir>)` without ever needing a phase-B runner.

A **real agent cell** has no oracle. It produces a `scene.xml` directory that
must be re-run cold, and `phase_b_runner("mujoco","T1")` **refuses**: the
column has no `t1_run_standalone`, and its generic `run_standalone` is
**T2's** runner (its own docstring says so), which the cell runner deliberately
declines to use for another tier because *"that is worse than a refusal: it
produces a number"*. The cell would read
`not_achieved(blocker=scaffolding_defect_ours)`.

*Cheapest path to READY*: a `t1_run_standalone` on the MuJoCo column — the T1
runner already exists (`ladder/adapters/mujoco/runner.py`) and drives+records;
what is needed is the `(deliverable, run_dir, backend=, duration=, settle=,
stride=, surfaces=, timeout_s=)` wrapper that re-runs a *given* scene rather
than building its own. That column is outside this workstream's file scope, so
it is named here rather than fixed.

#### mujoco / T2, T3, T4 — READY

Verified by running the full cell pipeline with the scripted oracle standing in
for the agent: staging, quarantine, deliverable discovery, phase B through the
tier's own runner, the real grader, the §3 row. All three returned
`achieved / PASS`, and T3/T4 additionally returned
`support: attested / T4-supported`, `distance_to_termination_m` and
`termination_cause`. The tier-specific runner is resolved in each case
(`t3_run_standalone`, `t4_run_standalone`, and `run_standalone` **which is
T2's**).

The caveat that keeps this honest: a dry run proves the **pipeline**, not that
an agent can do the task. The oracle is a scripted control a human wrote
knowing the thresholds, which `capability-ladder-plan.md` §2 explicitly says is
not a cell.

---

## 3. What this session changed, and what it did not

Built (all inside `ladder/adapters/omnisim/` and
`ladder/controllers/ladder_recorder/`): the whole T2/T3/T4 channel surface for
the OmniSim column — the sampler's tier document, the pure channel builders,
the four tier-specific phase-B hooks, the Newton-blindness disclosure, and 76
tests. Before it, `ladder/adapters/omnisim/evidence.py` shipped only
`run_standalone` + T1's `support_observation`, so **every** T2–T4 assertion on
our own column read `scaffolding_defect_ours`. Record and effort ledger:
[`adapters/omnisim/BRINGUP.md`](adapters/omnisim/BRINGUP.md).

Not touched, by scope: the graders, the cores, the cell package, AgentBench,
and the MuJoCo column. Two of the blockers above live in those files
(`mujoco/T1`'s missing runner; `graders/t2.py`'s lack of a tier-specific
preference loop, which was worked *around* rather than fixed — the OmniSim
column's generic `run_standalone` now defaults to the full scan so that T2's
hard-coded lookup lands on a runner that records T2's channels).

---

## 4. The ours-versus-theirs ledger, in one place

| finding | whose | why |
|---|---|---|
| Newton returns 0 contacts where ODE returns 1008 | **theirs** (`no_measurement_surface`) | our sampler works and proves it on the other backend; the contact list is ODE-fed in the engine |
| no wrench read-back (T3/T4 `applied_support`) | **theirs** (`no_measurement_surface`) | `add_force` is write-only and contact points carry no force; the channel attests what it structurally can and says `unverified` otherwise |
| `base_link` matches no body after URDF import | **the tier's**, pre-registered | both `meta.json` files carry it as `open_question_for_the_freeze`; the physics and the masses are correct |
| a world with `basicTimeStep > 50 ms` cannot satisfy T3.1's continuity clause | **the agent's scene** | the sampler already records every step; the clause reports its own witness absent |
| no `webots` ladder channel module | **ours** | never built |
| no `t1_run_standalone` on the MuJoCo column | **ours** | never built |
| no scripted oracle for the omnisim or webots column at any tier | **ours**, and it affects `--dry-run` only | a real cell has an agent where the oracle goes; this does not block a real cell |

---

## 5. Evidence — what was run, and what it returned

All on machine `9722d23d12a3` (RTX 3060 laptop, Windows 11), engine
`msys64/mingw64/bin/omnisim-bin.exe`, mujoco 3.8.1, python 3.12.9.

**Dry runs through the whole cell pipeline** (`cell/run_ladder_cell.py
--dry-run`):

| command | result |
|---|---|
| `--column omnisim --task T1_arrive` | `not_achieved(scaffolding_defect_ours)` — *no committed scripted oracle*. Workspace staged **4799 files**, 545 excluded, 7 redactions, `answer_key_exposure.workspace_clean: true`, t = 22.6 s |
| `--column webots --task T1_arrive` | same blocker (no oracle). Container staged, quarantine clean, t = 4.2 s |
| `--column mujoco --task T1_arrive` | **`achieved / PASS`**, t = 4.0 s |
| `--column mujoco --task T2_transfer` | **`achieved / PASS`**, t = 4.6 s |
| `--column mujoco --task T3_quadruped` | **`achieved / PASS`**, `support: attested`, `distance_to_termination_m: 31.5416`, `termination_cause: time_limit`, t = 8.7 s |
| `--column mujoco --task T4_humanoid` | **`achieved / PASS`**, `support: attested / T4-supported`, `distance_to_termination_m: 73.3048`, t = 19.9 s |

**Phase-B routing** (`cell.phase_b_runner(column, rung)`, re-verified *after*
the tier-specific-hook preference fix in `graders/t1.py` and `t4.py`):

```
omnisim  T1->t1_run_standalone  T2->t2_run_standalone  T3->t3_run_standalone  T4->t4_run_standalone
webots   T1..T4 -> REFUSED: "ships no ladder-side channel module"
mujoco   T1 -> REFUSED: "generic run_standalone is T2's runner"
         T2->run_standalone     T3->t3_run_standalone  T4->t4_run_standalone
```

**Direct grader invocations on the OmniSim column** (probe scenes, ODE pinned
except where stated; `duration` 20–25 s):

| run | verdict | the reading that matters |
|---|---|---|
| `t1.run_and_grade` husky, **Newton default** | `FAIL 1/5` | T1.4 red: *contacts of any kind observed: **0***, `supported: true`, `error: null`, 126 steps sampled |
| `t1.run_and_grade` husky, `backend="ode"` | `FAIL 3/5` → `4/5` after removing a bogus field from the probe world | T1.4 **green**: *1008* support contacts, all seen while moving, fastest 0.4955 m/s |
| `t3.run_and_grade` walker named `walker` | `FAIL 0/5`, all five vacuous | *"the base series is about 'walker', but the task names 'base_link'"* |
| `t3.run_and_grade` walker named `base_link` | `FAIL 2/5`, `unanswered_channels: null` | `support_attestation: attested`, 216 make-and-break transitions, arena 28.359 m, controller attested from 12/12 joints moving |
| `t2.run_and_grade` bench_arm + block + bin | `FAIL 2/5`, `unanswered_channels: null` | mass 0.2 kg, gravity 9.81, rim 0.2094 m, 1 static surface, **clock skew 0.0 s** |
| `t4.run_and_grade` strider named `base_link` | `FAIL 2/5`, `unanswered_channels: null` | `cell: T4-unsupported`, `arena_attestation: attested`, sample interval 0.032 s, 200 transitions |
| `t1.run_and_grade` husky, after the T2–T4 build | `FAIL 4/5`, **no `.channels.json` written** | the T1 path is unchanged and still cheap |

**Tests**: `python -m pytest tests/benchmarks/ladder -q` → **698 passed** in
~35 s (622 before this session + 76 new), with no engine, no GPU and no
network.

**Staging**: `cell/stage_ladder_workspace.py --column webots --task T1_arrive
--verify --keep` → template built, quarantine `CLEAN (0 hits)`, all nine
container files staged, and the instance carries a working
`webots-R2025a → \\wsl$\Ubuntu-22.04\opt\upstream-webots\R2025a` symlink.
