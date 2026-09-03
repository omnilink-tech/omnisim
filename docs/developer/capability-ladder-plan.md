# The capability ladder — can an agent reach a real robotics outcome, and where?

**Status: design, revision 1 — 2026-08-01. Nothing in this file is a result.** No ladder cell
has been run, no grader for it exists, and the tier statements below are **not yet frozen**
(§6 step 0 is the freeze).

**What this is.** A programme that fills a grid: four **physical robot outcomes** × several
simulators, each cell recording whether an autonomous agent — given one sentence, no human
help, and that simulator's product as shipped — produced a run in which the outcome was
**measured to have happened**, and what it cost. It promotes "Lane C — the capability
frontier" out of the quarantine it sits in inside
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.1 and makes it a
first-class programme with its own pre-registration, its own correction window and its own
reviewer.

**Why it exists, stated as the adverse finding that caused it.** The Phase W head-to-head
campaign completed 2026-08-01 (commit `124ae7f3`; rows under
[`tests/benchmarks/agentbench/results_published/`](../../tests/benchmarks/agentbench/results_published/)).
Its decision set was constrained by a fairness rule that is correct and that we keep — a task
enters Lane B only if it runs on **both** simulators through each one's native idiom
([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.1's oracle lane test) —
and the set that survived that filter is a set of **authoring, inspection and file-repair
tasks**. On those, both contestants largely saturate or both largely fail: upstream Webots led
completion on `B1` (4/5 vs 3/5) and `B2`, OmniSim led `B3` (2/5 vs 1/5), the two debug tasks
**tied 5/5 each** with mirrored cost profiles, and the `A1` authoring control scored **0 on
both arms**. That is a real finding about a real question, and §7 keeps it. But it is not the
question a roboticist asks, and a decision set that a competent text editor can service is
structurally unable to answer the question a roboticist asks.

This programme asks the other question. It is not a rematch and it cannot produce a winner.

**Relationship to the other files.**
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) is the cost-to-outcome
programme (E1, the F withdrawal rule, Phase W); this file is the reachability programme, and
**neither is evidence for the other** (§7).
[`tests/benchmarks/agentbench/SPEC.md`](../../tests/benchmarks/agentbench/SPEC.md) remains the
contract: its outcome taxonomy (§3.3), its `NOT_EXPRESSIBLE` discipline (§6.4), its
competitor-fairness machinery (§6.2), its zero-intervention rule (§3.4) and its
author-and-contestant commitments (§8.2, §8.3) bind here **verbatim and without amendment**.
[`tests/benchmarks/omnibench/lane3/DRIVEABILITY.md`](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md)
is the format precedent — a capability contract stated in simulator-neutral terms, one row per
probe, hand-scorable by a stranger.
[`simulator-comparison.md`](simulator-comparison.md) is what we actually know about the
competitors, and its **§9.2 list of eight things this project previously got wrong about them**
is why §3 forbids a bare ✗.

---

## Contents

- [1. The question, stated so it can fail](#1-the-question-stated-so-it-can-fail)
- [2. The ladder](#2-the-ladder)
- [3. What goes in a cell](#3-what-goes-in-a-cell)
- [4. The roster, and what each column costs](#4-the-roster-and-what-each-column-costs)
- [5. The three traps, as binding rules](#5-the-three-traps-as-binding-rules)
- [6. Sequencing and cost](#6-sequencing-and-cost)
- [7. What this programme does not claim](#7-what-this-programme-does-not-claim)
- [8. The forbidden sentences](#8-the-forbidden-sentences)
- [9. Open questions, recorded rather than resolved](#9-open-questions-recorded-rather-than-resolved)

---

## 1. The question, stated so it can fail

> **E2 (the ladder).** For each of four physical outcomes a roboticist wants — a wheeled robot
> arriving at a commanded point on the floor; an arm moving an object into a container and the
> grip holding it; a four-legged robot walking ten metres without falling; a two-legged robot
> walking ten metres without falling — and for each simulator in the roster: **given exactly
> one sentence, no human in the loop, and that simulator's product as shipped, does an
> autonomous agent produce a deliverable in which the outcome is measured to have happened —
> and at what cost in agent tool calls, wall clock, GPU-hours and dollars?**
>
> The recorded answer per cell is one of three values (§3), never a tick. The headline is the
> **filled grid**, not a ranking of it.

E2 is a question, not a claim — but a question with no losing answer is worthless, so the
adverse outcomes are pre-registered here, before any cell runs, and each is published as
prominently as a favourable one would be:

1. **OmniSim's own column stalls low.** If an agent cannot reach T1 or T2 on OmniSim from one
   sentence, the grid says so in the leftmost column and that is the headline. Our prior is
   that this is a live possibility, not a formality — Phase W's `A1` authoring control scored
   **0/5 on the OmniSim arm** (`124ae7f3`), and the failures were not exotic: working
   ten-robot scenes that died on relative asset paths.
2. **The ladder discriminates nothing.** If every simulator reaches the same rungs, the honest
   finding is *"at this instrument, on these four outcomes, the field is not separated"* — and
   the programme publishes that and stops, rather than adding tiers until a gap appears.
   Adding a tier after the first cell runs voids the pass (§5a).
3. **A competitor reaches a rung we do not.** MuJoCo Playground is the specific, named risk:
   quadruped joystick locomotion in ~5 minutes and a Unitree G1 walking in under 30 minutes on
   2× RTX 4090, with zero-shot transfer to six physical platforms
   ([`simulator-comparison.md`](simulator-comparison.md) §5.3, evidence ✅). **We expect to
   lose T3 and possibly T4 to that column**, and the ladder is published with that loss at the
   top, not in a footnote.
4. **Every column that reaches a rung reached it by invoking a shipped asset.** Then the grid
   measured the assets, not the agents. §3's `reuse_class` field exists to make this visible
   rather than arguable, and a tier where every column is `assembled` is published labelled as
   measuring shipped assets.

**What E2 cannot do.** It cannot support or refute E1 in either direction. Reachability is not
throughput; a rung is not a cost comparison; and a grid is not a score (§7).

---

## 2. The ladder

Four tiers. Each is stated as a **physical outcome**, in the vocabulary a roboticist uses,
containing no file format, no endpoint, no node type, no solver name and no product's proper
noun. The binding form of that constraint is §5a's outcome-not-feature rule and its test.

Every tier's grading criteria are in SI units in the world frame, and every assertion is
graded from a **grader-owned recorder** — never from what the agent says, never from a value
polled over a network, never from an artifact the agent hands us unverified. The evidence
channels are the ones already contracted in
[`tests/benchmarks/agentbench/adapters/__init__.py`](../../tests/benchmarks/agentbench/adapters/__init__.py)
(`REQUIRED_EVIDENCE`: `roster`, `t0`, `contacts`, `trajectory`, `process`, `attribution`,
`identity`, `view`), and a simulator that cannot supply a channel a tier needs produces a
`NOT_EXPRESSIBLE` **citing that line** (§3), not a failure.

**Two-phase grading, inherited from `A1`** ([SPEC](../../tests/benchmarks/agentbench/SPEC.md)
§2.3): phase A observes what the agent built while it is live; **phase B re-runs the
deliverable standalone and cold**, with the grader's own recorder injected and no agent
present. A behaviour that only happens while the agent is holding it up is not a behaviour.

**Repeats: n = 3 per cell.** Below the SPEC's n = 5 scoring floor, deliberately: this is a
reachability map, not a pass rate. Consequences, binding — ladder cells are **`exploratory`
under [SPEC](../../tests/benchmarks/agentbench/SPEC.md) §3.5**, `achieved` means **reached at
least once in 3** (the `solved_at_least_once` statistic, which §3.5 requires be reported
separately from `pass@1` and never conflated with it), and **every cell prints `k/3` inside
the cell** so a 1/3 can never be read as a 3/3. A ladder cell may never be quoted as a pass
rate (§8).

---

### T1 — a wheeled robot arrives at a point on the floor

**Outcome.** The robot described by the files in the workspace exists in a scene, drives under
its own actuation, and ends up at a commanded location on the ground plane, and the arrival is
recorded.

**One-sentence prompt (shape; exact wording frozen at §6 step 0).** *"Here is a robot
description; put it in a world and drive it to the point five metres north of where it starts,
then show me it got there."*

**What the container ships.** The robot as a **URDF plus its meshes, and nothing else** — no
pre-converted scene, on any simulator. This is a deliberate deviation from Phase W, which
supplied the robot pre-converted on both arms specifically so that the conversion step could
not decide a cost comparison ([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md)
§3, Phase W wrinkle 1). Here the conversion **is** part of the capability under test, which is
legitimate because this programme is not a cost comparison and because URDF is a
vendor-neutral interchange format every roster member documents an import path for (§4 — each
of those paths is marked with its evidence tier, and most are **unverified**). The risk is
named: if a simulator's URDF path is broken or undocumented, this tier will read
`not_achieved(no_importer)` on it, and that is a finding a maintainer can refute in the
correction window with one working recipe.

**T1 ships the meshes; T2 ships primitives. That divergence is a decision, not an
inconsistency** (ratified 2026-08-02, before the freeze; §9 Q4 is closed in this direction).
T1's container carries the real third-party robot description **unmodified**, COLLADA meshes and
upstream `package://` references intact, because *bringing a real robot description in is what
this tier measures*. T2's container carries **primitive geometry and not one mesh**, because
that tier is decided by collision geometry and grasp physics, where a mesh-decoding gap would
fail the cell for a reason with nothing to do with manipulation. Same reasoning, two tiers,
opposite answers — and the mesh risk here is now **bounded rather than hypothetical**: the
MuJoCo bring-up brought this exact description in with its meshes navigable through that
simulator's own first-party settings, no re-authoring and no benchmark-side special case
([`adapters/mujoco/BRINGUP.md`](../../tests/benchmarks/ladder/adapters/mujoco/BRINGUP.md)). A
column whose loader cannot decode COLLADA still reads `not_achieved(no_importer)` **naming the
format in the blocker**, and the countermeasure §9 Q4 proposed (a separately-labelled re-run on
a primitive description) stays available and is deliberately *not* pre-emptively applied, since
applying it everywhere would delete the measurement. Recorded in both task files.

**Grading — all must hold, measured in the world frame, SI.**

| # | assertion | pass condition |
|---|---|---|
| T1.1 | **arrived** | the robot base's horizontal position is within **0.25 m** of the commanded waypoint, and stays within **0.35 m** of it, for a continuous window of **≥ 2.0 s of simulated time** ending at the run's end |
| T1.2 | **stayed up** | base `z` within **±0.30 m** of its settled start `z` at **every** recorded sample, and `> −0.10 m` at every sample (it did not sink, fly, or fall through) |
| T1.3 | **drove, not teleported** | integrated path length over the run is **≥ 0.9 ×** and **≤ 5.0 ×** the straight-line start→goal distance, and **no single inter-sample displacement exceeds 0.5 m** at the recorder's per-physics-step rate |
| T1.4 | **touched the surface while moving** | ≥ 1 contact pair naming (a robot body, the ground) is observed during motion, with the vacuity counters (`total_observed`, `distinct_named`) present so "no contact" is distinguishable from a query that names nothing |
| T1.5 | **the run is real** | the process reached world-finalize with independent evidence (an exit code alone is never accepted), zero `ERROR`-class lines, and the physics backend/solver that drove phase B recorded with a citation |

**T1.4 was called "under its own actuation" until 2026-08-02, and the row was renamed rather
than re-checked.** What it measures is a contact naming the robot and a distinct non-robot body
seen while the robot was moving — which **a robot riding a moving pallet or a conveyor
satisfies**, demonstrated in the pre-freeze audit by a T1 run grading PASS 5/5 on a robot whose
only support contact was with a body named `pallet`. The stronger reading — actually detect that
the robot drove itself — was **considered and declined**: it needs wheel torque, motor command
or joint velocity, and **no column in the roster supplies any of them today**, so the clause
would impose a new capability requirement on every column and redden cells for a channel *we*
never built, which §4 forbids publishing as somebody else's product failure. The honest response
to a label that over-claims is to fix the label. **Not one measured value changed.** The
physical question — did it drive, or was it carried — stays visibly unanswered rather than
silently answered, and if a reviewer wants it graded, the channel has to exist on more than one
column first.

**Deliverable.** Whatever the agent wrote, re-runnable standalone by one command in the
workspace. Phase B runs that command cold.

**Cost class.** CPU minutes. No GPU. Tokens only. This is the cheapest cell in the programme
and it runs locally on every column that installs locally.

**Can OmniSim do this agent-unassisted today? — genuinely uncertain, and I would not bet on
3/3.** The capability is unambiguously present: `URDFRobot` imports URDF natively, the
converter is also available offline
([`scripts/dev/urdf_import.py`](../../scripts/dev/urdf_import.py), `--report --strict`), and a
`drive_to` primitive that reports `{commanded, achieved, error, settled}` exists in
[`projects/samples/demos/controllers/omnilink_mobile_bridge/`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/).
But **that primitive lives in a demo controller that must already be attached to the robot in
the world** — an agent that imports a fresh URDF gets none of it and must write a controller.
And the empirical warning is direct: on Phase W's authoring control the OmniSim arm produced
working ten-robot scenes that died on **relative asset paths** and scored 0/5 (`124ae7f3`).
Path plumbing, not physics, is the most likely blocker here.

---

### T2 — an arm moves an object into a container, and the grip holds

**Outcome.** An object that starts on a surface ends up inside a container, having been carried
there by a robot arm, and the arm held it for ten seconds on the way.

**One-sentence prompt (shape).** *"Pick up the block on the table, put it in the bin, and prove
the gripper held it for ten seconds."*

**Grading — all must hold.**

| # | assertion | pass condition |
|---|---|---|
| T2.1 | **transferred** | the object's centre **starts outside** the container's world-space AABB **and ends inside** it, with its lowest point **below the container rim**, and it is at rest (speed **< 0.02 m/s** averaged over the last **1.0 s**) |
| T2.2 | **lifted, not dragged** | during the run the object's lowest point is **≥ 0.05 m** above every static support surface for a continuous **≥ 3.0 s** |
| T2.3 | **held ten seconds** | over a continuous **≥ 10.0 s of simulated time**, the object's position **in the end-effector's frame** varies by **≤ 0.02 m RMS**, with max excursion **≤ 0.04 m** |
| T2.4 | **the object is a real body** | from the `t0` inventory: mass **> 0**, gravity acting, dynamic — a kinematic prop being flown to the bin fails here — **and** no single inter-sample displacement exceeds **0.5 m** (T1.3's own bound, re-used and not re-declared) at a recorder interval **≤ 0.05 s**, so a body whose position is *written* to the bin fails here as surely as a massless one does |
| T2.5 | **the run is real** | as T1.5 |

**Two of those clauses were added before the freeze, each after it had been demonstrated
missing on an artifact that graded PASS without it.** Both are recorded here rather than left in
the grader, because a clause the tier text does not state is a clause a reviewer cannot attack:

- **T2.1's "starts outside"** (2026-08-02). T2 graded an end state, a lift and a hold, and *no
  assertion read where the object began* — so a run that lifts the object **out** of the
  container, holds it twelve seconds above it and puts it back graded **PASS 5/5**. Every row was
  honest about what it measured; the tier simply was not asking. The outcome sentence above says
  the object *starts on a surface*, so this was the tier failing to grade what it states. It
  costs no column anything new: the clause reads the **first** sample of exactly the series whose
  **last** sample T2.1 already grades. It deliberately does **not** assert "on a surface" — that
  would grade where the agent chose to set the object down, which is scene authorship, not
  transfer.
- **T2.4's jump bound** (2026-08-02, ratified from the grader's own derived clause). T2 as
  written had no teleport guard: mass, dynamic and gravity catch a kinematic prop, but **not a
  scripted position write onto a body that genuinely has mass**, and the tier's own gloss on
  T2.4 is *"a kinematic prop being flown to the bin fails here"*. It declares no new number —
  it re-uses T1.3's constant and T1.3's rate witness, and a series too coarse to see a jump makes
  the clause report its witness absent rather than passing it.

**What the container ships.** One arm description, one object description, one container
description, **all primitive geometry — not one mesh and not one external reference** — plus
nothing else: no ground, no scene, no placement, no launch script, no controller, no
inverse-kinematics helper and no grasp pose, on any column. **The divergence from T1, which
ships a real robot with its COLLADA meshes, is deliberate** (§2 T1 states the other half): this
tier is decided by *collision geometry and grasp physics*, so a column that cannot decode a
visual mesh format would fail T2 for a reason with nothing to do with grasping — exactly the
confound §9 Q4 names, and §9 Q4's own countermeasure is "a self-contained primitive-geometry
description". The cost of the choice is stated rather than hidden: the arm is not a robot
anybody has ever built, so a column shipping a first-party model of a *real* arm gets no credit
for having it.

**A known limit, published rather than repaired** (ratified 2026-08-02). T2.2's clearance is
measured against **static** surfaces only — a body that can move is not a fixed datum — so an
object resting motionless on a **dynamic** body taller than 0.05 m reads as *lifted*, and after
the T2.3 carry-gate repair that hold inherits the same reading. Demonstrated: a block at rest for
a whole 20 s run on a 0.10 m dynamic crate scores "longest continuous window clear by ≥ 0.05 m:
20.0 s". **It is not fixed, deliberately.** Closing it needs the world AABB of every *dynamic*
body at every sample, which no column in the roster supplies, and a clause whose witness is
permanently absent is the vacuity this tier's pre-freeze audit existed to remove. What contains
it is T2.1: on the container this tier ships, a crate tall enough to clear the bin's own rim by
0.05 m puts the object's centre **above** the container's box, so the full cell does not pass on
this hole. Any reader of a T2 cell is entitled to this paragraph.

**The mechanism is recorded, not graded.** Every T2 cell carries
`hold_mechanism ∈ {friction, suction, attachment, unknown}` with the adapter's source citation.
It is **not** a pass condition, and here is the reasoning, stated so it can be attacked: this
tree's own shipped bin-picking success uses a **suction** end-effector
(`docs/developer/omniarm6-suction-bin-pick-journey.md` (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)):
deterministic 36/36 after finger-gripper attempts topped out ~18/36), and an earlier carry demo
used a grasp-stabilisation weld. Failing an attachment-based grasp would be writing the task
around a technique we ourselves abandoned — the inverse of task-rigging and equally
uninformative. So the outcome is the physical transfer and hold; the mechanism is published
**inside the cell**, and no prose sentence about T2 may omit it (§8).

**Deliverable.** The scene plus whatever drives the arm, re-runnable standalone.

**Cost class.** CPU minutes to hours. No GPU required. Training is permitted but not expected.

**Can OmniSim do this agent-unassisted today? — I expect this tier to FAIL on OmniSim, and I
am saying so before running it.** Three specific reasons, each from this tree:

- The one-wall pinch of a **free** object was proven **not fixable by any parameter** here —
  sweeps of finger stiffness, contact stiffness, friction, `IMPRATIO`, iterations, commanded
  width, grasp depth and object mass all failed, and the demo was retired rather than propped
  up ([`real-grasp-and-the-cold-first-load-trap.md`](real-grasp-and-the-cold-first-load-trap.md),
  retirement note 2026-07-05). The physical diagnosis was geometric, not tuning.
- The pinch that *does* work requires pinning `newtonSolver "mujoco"` — XPBD, the default
  Newton solver family, structurally cannot hold a pinch. An agent that does not know to pin
  the solver gets a grasp that slips, with no error anywhere.
- The shipped success needed a **purpose-built end-effector**
  (`projects/robots/omnisim/omniarm6/omniarm6_suction.urdf`) authored across four documented acts of
  human iteration. That is human-months compressed into a manifest, not a one-prompt run.

The honest prediction for the OmniSim T2 cell is `not_achieved`, most likely with
`blocker=no_physics_capability` (grip does not hold) or `blocker=agent_lost` (the solver pin is
undiscoverable from one sentence). If it passes, the `hold_mechanism` field is what tells the
reader whether that was a grasp or an attachment.

---

### T3 — a four-legged robot walks ten metres without falling

**Outcome.** A quadruped crosses ten metres of flat ground on its legs and is still standing at
the end.

**One-sentence prompt (shape).** *"Make this four-legged robot walk ten metres across flat
ground without falling over, and show me the recorded path."*

**Training is permitted.** GPU-hours are a **cost field**, not a disqualification; a cell that
reaches the outcome by training a policy and a cell that reaches it by a model-based gait are
both `achieved`, with `method` recorded.

**Grading — all must hold, over one continuous run.**

| # | assertion | pass condition |
|---|---|---|
| T3.1 | **ten metres** | net horizontal displacement of the base **≥ 10.0 m** in a single continuous run (no reset, no restart) |
| T3.2 | **never fell** | base `z` **≥ 0.60 ×** its settled standing `z` at **every** recorded sample, and \|roll\|, \|pitch\| **≤ 0.8 rad** at every sample. The standing `z` is read per robot from the `t0` inventory, not fixed as a constant, so the criterion is robot-neutral |
| T3.3 | **walked, did not slide or drift** | mean forward speed **≥ 0.15 m/s** over the window; **and** ≥ 8 distinct make-and-break transitions of (robot body, ground) contact are observed, with the vacuity counters present. The base's **vertical bob about its trend is measured and printed in every row and is not a pass condition** — *repaired 2026-08-02, see below; the retired bar was ≥ 0.005 m RMS and every row still prints it beside the measured value* |
| T3.4 | **nothing held it up** | no contact with any body other than the ground, **and** total non-gravitational, non-contact force applied to the robot's base is **≤ 0.02 × m·g** peak and **≤ 2 N·m** of applied torque. Where a simulator cannot attest the applied-force total, the cell records `support_attestation: unverified` and is **excluded from comparison with any cell that has it** |
| T3.5 | **the run is real** | as T1.5, and the policy/controller is asserted to have actually loaded (an exit code is not evidence — see below) |

**T3.3's vertical-bob conjunct was retired as a pass condition on 2026-08-02, before the
freeze, after it was demonstrated failing a robot that was plainly walking.** Recorded here
rather than left in the grader, for the reason T2's two added clauses are: a reading the tier
text does not state is a reading a reviewer cannot attack.

- **What was demonstrated.** On this tier's own achievability oracle (§5c requires one, and
  MuJoCo supplied it — [`BRINGUP_T3.md`](../../tests/benchmarks/ladder/adapters/mujoco/BRINGUP_T3.md)
  §5.2), the same scripted four-beat crawl driven **slowly** covers **16.5 m at 0.27 m/s
  without falling**, with **733 make-and-break ground-contact transitions**, and bobs
  **0.0018 m RMS** — red against 0.005 m. The identical gait reaches 0.0089 m only once it is
  driven to 0.49 m/s. A rigid body carried at a constant commanded height does not rise and
  fall much *however honestly it is walking*, so the clause was not measuring whether the robot
  walked; it was measuring how **dynamic** its gait was. A statically stable crawl is a correct
  answer to the prompt — arguably the safest one — and the tier failed it.
- **What was decided.** Ground-contact make/breaks remain the gate for *"it stepped rather than
  slid"*; the bob becomes a **reported measurement**. The bob's failure mode is demonstrated and
  real; the failure it guarded against — a **slider** producing ≥ 8 make/breaks *while also*
  covering 10 m at ≥ 0.15 m/s — is much harder to produce and is already excluded by T3.1 and
  the speed clause acting together. A dragged body reports **zero** transitions (that is what
  §5c's named *"crawling robot"* fixture is), and a body cycling its feet on the spot fails the
  speed floor. Both halves are pinned by their own tests.
- **No threshold moved, and the retired one is still printed.** 8 transitions and 0.15 m/s are
  untouched, and `MIN_BOB_RMS_M = 0.005` is deliberately **not deleted**: it is still computed,
  still in the task file's constants table, and printed in every row beside the measured bob as
  *the bar this clause applied until 2026-08-02*, together with whether that bar would have been
  met. Any row that passes only because of this change says so in its own detail line. Moving
  the number after seeing a measurement is the act §5a voids a pass for; repairing a predicate
  that does not test what its sentence says is a different act, and it is the same repair T2.3
  received the same day.
- **Regression evidence, both directions.** `slow_static_gait` (0.27 m/s, 16.47 m, 732
  transitions, 0.0018 m bob) is a declared **must-PASS control** in T3's coverage table and was
  red under the old predicate; `slid_without_gait` stays red **through the transition clause
  alone**, asserted by restoring its bob in full and checking it is still red.

**The tier is achievable as shipped, it needs no training, and the proof is reproducible from
this repository.** §5c makes an achievability demonstration a precondition of the freeze,
because a robot no column can walk would make the cell a measurement of *our asset* rather than
of anybody's agent. It was demonstrated on 2026-08-02 on the MuJoCo column — and from a scratch
harness outside the tree, which under the standing *"no row, no result"* rule
([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §0.2) left it *demonstrated but
not reproducible*. The oracle is now committed
([`run_t3.py`](../../tests/benchmarks/ladder/adapters/mujoco/run_t3.py),
[`BRINGUP_T3.md`](../../tests/benchmarks/ladder/adapters/mujoco/BRINGUP_T3.md)) and reproduces
with **one command in about 5 s of CPU wall clock, no GPU and no network**: a scripted four-beat
crawl with **no learning of any kind** walks **31.54 m in 70 s**, still walking when the clock
stops, and grades **PASS 5/5** through the real T3 path with **no vacuous clause and no
unanswered channel**. It is a scripted control and **not a cell**, and it says nothing about any
agent or any other column. Two things it found are on the record rather than smoothed over: the
rebuild is **9 % short of the scratch run** (31.54 m against 34.73 m) because the recipe was
prose and three parts of it were under-determined, and **URDF cannot express rotor inertia** —
with `armature` at the importer's default of 0 this robot bounces across the floor and flips over
inside 3 s *while being told to stand still*, which is the single most likely first blocker on
any column and reads exactly like a physics defect.

**Deliverable.** The world, the policy or controller, and one command that reproduces the walk
cold. Phase B runs that command with the grader's recorder.

**Cost class.** GPU. On the sanctioned cloud venue this is cheap in dollars and expensive in
engineering: three RunPod sessions producing three quadruped results cost **≈ $2.7 total**
([`rl-current-state.md`](rl-current-state.md), 2026-07-17 banner). Locally, an RTX 3060 laptop
measures **10,228 env-steps/s at K = 256** on the in-engine trainer (OmniBench lane 2 tier C,
[`omnibench-2026-07-24.md`](../benchmarks/omnibench-2026-07-24.md)) against the shipped Go2
recipe's **K = 16384** — so a local T3 attempt on M1 is itself an experiment whose likely
outcome is `not_achieved(resource_limit)`, and §6 sequences it as such rather than pretending
it is free.

**Can OmniSim do this agent-unassisted today? — the capability exists; whether an *agent*
reaches it from one sentence is exactly what is uncertain, and the `reuse_class` field is the
whole point of this cell.** `go2_shadow_walk` is `verified` in the tree
([`projects/policies/skills/quadruped/go2_shadow_walk/skill.json`](../../projects/policies/skills/quadruped/go2_shadow_walk/skill.json)):
0.429 m/s, zero falls over ~1600 s of simulated time per side, on Newton/mujoco_warp, with the
backend sidecar clean. But read what produced it — a **recorded ghost of the incumbent
champion's own achieved gait**, eight builder gates, a `ghost_validator` verdict, and a
two-phase corridor curriculum discovered only after a from-scratch run converged to a shuffle
([`rl-current-state.md`](rl-current-state.md), 2026-07-12 banner). That is human-months.

An agent on the OmniSim product does have a front door — `python -m omnisim policy list` /
`train <skill>` / `run <skill>`, documented in
[`skill-library.md`](skill-library.md) — and the answer-key ruling (§5c) leaves it in the
workspace, because it is **product capability documentation, not benchmark answer key**. So
the most likely OmniSim T3 outcome is `achieved` with **`reuse_class: assembled`** — the agent
invoked a shipped champion. That is a real and publishable capability finding *and* it is a
much weaker statement than "an agent made a quadruped walk", and the grid must never let the
two be confused (§3, §8).

One trap that has already voided a quadruped result in this tree and will void a ladder cell:
if `onnxruntime` is missing from the interpreter that spawns **controllers** (a different
interpreter from the one the engine embeds), the deploy runs a **zero-residual bare-ghost
baseline and exits 0**. **Assert the `ONNX loaded:` line, never the exit code** — T3.5 encodes
this, and the head-to-head that ignored it produced a spectacular, entirely invalid result
(`skill.json` `voided_first_attempt`).

---

### T4 — a two-legged robot walks ten metres without falling

**Outcome.** A humanoid crosses ten metres on its legs and is still standing at the end.

**One-sentence prompt (shape).** *"Make this two-legged robot walk ten metres across flat
ground without falling over, and show me the recorded path."*

**Grading.** T3.1, T3.2, T3.3, T3.5 unchanged. T3.4 is **replaced** by the support measurement
below.

**The bar is 10.0 m and it does not move — decided 2026-08-02, after the measurement came back
favourable.** §9 Q1 required the distance be measured against what this tree ships *before* the
freeze. It was: [`g1-endurance-2026-08-01.md`](g1-endurance-2026-08-01.md) (committed
`86c6522e`) records the shipped flagship crossing **10.0 m at t = 82.9 s, 6 / 6 runs, zero
falls**, reaching **12.9 m** before the shipped arena's wall stops it, and **29.5 m in 254 s** in
a longer arena with the run *still* ending on geometry rather than on the policy — all on the
λ = 0.9 weight-bearing balance harness, which no sentence about it may omit. That refutes this
document's own prior expectation, which was that 10 m might be unreachable even harnessed.

**The bar stays at 10.0 m verbatim anyway, and the reason is the whole point of pre-registering
one.** The number was written down before it was measured. A threshold that moves after the
measurement is worthless *whichever way it moves*: lowering it after a bad result is the failure
§9 Q1 was written to forbid, and raising it after a good one — to 25 m or 30 m, say, which this
tree demonstrably clears — is the same act with better optics, choosing the axis after seeing
where our own robot lands, and it would have to be disclosed as such in the same sentence as the
number ([`g1-endurance-2026-08-01.md`](g1-endurance-2026-08-01.md) §8 Option 2 spells out why
that disclosure costs more credibility than the extra discrimination is worth). **The honest
consequence is recorded instead of hidden: 10 m now sits at roughly a third of our own
artifact's demonstrated reach, so this tier is less discriminating than it looks, and a column
that can walk at all will likely clear it.** That is a stated weakness of the tier, not a reason
to rewrite it after the fact.

**Headroom is therefore published as a measurement, never as a bar.** Every T4 cell — passing or
failing, on every column — must carry:

| field | what it records |
|---|---|
| **`distance_to_termination_m`** | the net horizontal base displacement actually reached, whatever it was, not clipped at the bar |
| **`termination_cause`** | what ended the run: `fell` · `arena_geometry` · `time_limit` · `controller_stopped` · `unknown`, with the evidence named |

Neither is a pass condition and neither may be turned into one. They exist so a `10.1 m` and a
`29.5 m` are distinguishable in the published grid, and so that a run which stopped because it
**ran out of floor** is never read as a walk that degraded or a robot that fell.

**A build note the endurance run produced, binding on the T4 task on every column:**

1. **The T4 recorder must fail loudly on arena contact**, not record a plateau. In all six
   endurance runs the *world* ended the run, not the policy: `x` froze at 12.90 m against a wall
   at x = 13.0 and stayed there for 200 s while the robot kept locomoting sideways. A grader that
   did not know where the walls were would have read those runs as *"the walk degrades to
   0.037 m/s after ~110 s"* — a false capability finding about a robot that was simply pressed
   against a wall. Arena contact is a **termination**, recorded as `termination_cause:
   arena_geometry`, and a T4 cell whose recorder cannot see it is `support_attestation`-style
   incomplete rather than quietly graded.
2. **Every column's T4 task needs a free run-up of at least 1.5 × the bar** — ≥ 15 m of
   unobstructed floor ahead of the start for a 10 m bar — **stated in the task**, and OmniSim's
   own column needs a world authored to it: the shipped puppet arena (`floorSize 26 14`) clears
   10 m by only 29 % and fails the rule outright at any bar of 15 m or more.
3. This also bears on §9 Q2: our `external_support` attestation is real and per-channel
   (`fx/fy/fz/tx/ty/tz`, per tick, from the applied wrench itself), which is a strong position
   for the assertion T3.4 and T4 both rest on — **but only if the run it attests is not silently
   geometry-bound**, which is what rule 1 exists to guarantee.

**The support decision, made here and stated plainly.** The task **permits** a support harness,
and every cell measures the support instead of asserting its absence.

Why permit it: forbidding support would make this tier a task we already know we fail on
OmniSim, which is uninformative in the opposite direction from rigging. The shipped G1
locomotion demos run on a **weight-bearing puppet rig** — `HARNESS_KZ=2000` N/m upward-only
clamped at **700 N** against the G1's 34.1 kg ≈ 335 N, i.e. **up to ≈ 2× body weight carried**,
plus a ±350 N·m attitude spring and a lateral catch, all at **λ = 0.9**, with **no G1 walk
champion ever shown at λ = 0**
([`rl-current-state.md`](rl-current-state.md), the **THE HARNESS** block). A durable
free-standing humanoid walk is an **open problem** here, and [`AGENTS.md`](../../AGENTS.md)
binds every statement about a G1 result to disclose the harness.

Why measuring beats a boolean: "harnessed / unharnessed" hides the difference between a
fingertip and a crane.

**How it is graded and published — two cells, never one:**

| cell | condition |
|---|---|
| **`T4-unsupported`** | peak non-gravitational, non-contact force on the base **≤ 0.02 × m·g** and **≤ 2 N·m** — numerically nothing |
| **`T4-supported`** | anything larger, with the **measured** peak force as a multiple of body weight, the peak applied torque in N·m, and the **fraction of the walk window during which support was non-zero**, printed inside the cell |

A published `T4-supported` cell reads, literally:
`achieved 1/3 (supported: peak 2.09 × body weight, 348 N·m, 100% of window; reuse_class: assembled)`.
**The support number is in the cell, not in a footnote**, and §8 forbids any T4 sentence that
omits it. Where a simulator cannot attest applied forces, the cell is
`achieved (support unverified)` and is excluded from comparison with an attested cell.

**Cost class.** GPU, and the highest engineering cost in the ladder.

**Can OmniSim do this agent-unassisted today? — no for `T4-unsupported`, and `T4-supported` is
uncertain for a reason worth stating.** `T4-unsupported` is `not_achieved(no_physics_capability
→ open research)` by our own canonical status, and that cell should be published as ✗ with the
blocker named, in our own column, in the first pass.

`T4-supported` **was** expected to be the harder half, and the reason has since been measured
and refuted. The prediction, kept visible: *"the flagship decent walker measures 0.120 m/s on
the exact puppet world over a 15.04 s scored window; ten metres at that speed is ≈ 83 s of
continuous walking, and no continuous G1 bout of that length is recorded anywhere in this tree
that I found — so even the supported cell may come back `not_achieved(no_physics_capability)`
on distance."* §9 Q1 required that be measured before the freeze rather than assumed.

**It was measured, and the prediction was wrong** ([`g1-endurance-2026-08-01.md`](g1-endurance-2026-08-01.md),
`86c6522e`): 10.0 m at **t = 82.9 s** — within 0.5 % of the arithmetic — **6 / 6 runs, zero
falls**, 12.9 m before the arena wall, 29.5 m in a longer one. So the *distance* blocker is not
the one to expect on our own column; what remains uncertain is everything the ladder actually
asks, which is whether an **agent** gets there from one sentence. Two caveats bind any use of
that measurement: it is on the λ = 0.9 weight-bearing balance harness (0 N vertical applied on
this run, up to **69.2 N·m** of attitude authority, non-zero **100 %** of the window — decisively
a `T4-supported` cell, 5.5 × over the force threshold and 34.6 × over the torque one), and the
six runs are **reproducibility checks, not six independent samples** — `t` to 10 m was 82.88 s in
every one of them, so a "3/3" here says the artifact is reliable, not that we sampled a wide
distribution's good tail. `T4-unsupported` is untouched by all of this and is still expected to
publish as ✗ in our own column.

---

## 3. What goes in a cell

**A bare ✗ is forbidden, and a bare ✓ is barely better.**

A ✗ is an unfalsifiable claim about someone else's product. It silently merges four different
findings with four different owners — *their simulator cannot do this*, *their documentation
did not say how*, *our scaffolding was inadequate*, *the agent gave up* — and one of those
owners is us. A named blocker is checkable by a maintainer in an afternoon; a ✗ is not
checkable at all. The precedent is not hypothetical:
[`simulator-comparison.md`](simulator-comparison.md) §9.2 lists **eight** things this project
previously got wrong about the field, and at least two are exactly this shape — asserting that
no simulator shipped a first-party agent-facing scene-control API (ROS 2
`simulation_interfaces` is an Apache-2.0 standard implemented natively by three of them), and
asserting Newton had no conservation tests (it ships
`newton/tests/test_physics_verification.py`). Both were absences we were confident about and
wrong about.

### 3.1 The three cell values

| value | meaning | requirements |
|---|---|---|
| **`achieved`** | every tier assertion held in phase B on **≥ 1 of 3** runs | the deliverable, the recorder trace, the phase-B re-run, and every sub-label below |
| **`not_achieved(blocker=<code>)`** | the outcome was not measured, and we say why | a blocker code from §3.3, **plus** the evidence for it: a named trace excerpt and the artifact as the agent left it; all three runs' blockers listed, the modal one in the cell |
| **`NOT_EXPRESSIBLE`** | the tier has no honest formulation on this simulator | [SPEC](../../tests/benchmarks/agentbench/SPEC.md) §6.4's rules verbatim — a written justification citing **that simulator's own docs or API listing** showing the required verb or property is absent, review during the 30-day window, and **a single credible counter-example flips the label and the cell gets run**. Additionally: the justification must cite one of the `REQUIRED_EVIDENCE` lines in [`adapters/__init__.py`](../../tests/benchmarks/agentbench/adapters/__init__.py) — the channel the simulator cannot supply. `NOT_EXPRESSIBLE` is excluded from every aggregate, is reported as its own count, and **may never be rendered in a failure's colour in any chart** |

### 3.2 The fields every cell carries

**Cost** — per [SPEC](../../tests/benchmarks/agentbench/SPEC.md) §3.1, with unmeasured fields
`null` and never `0.0`:

`tool_calls` · `turns` · `t_agent_s` **and** `t_total_s` (both printed; `t_agent_s` leads) ·
`tokens_in` / `tokens_out` / `tokens_cache_read` (separately, **never summed**) · `gpu_hours` ·
`usd` at list price on the run date · `interventions` · `metrics_source` (naming which figures
are the instrument's self-reported ones — Claude Code reports its own cost and turns; §6) ·
`measured_under_concurrency` (a flagged row's **time columns are excluded from every latency
statement**, per [`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.3).

**`interventions` is structurally 0** ([SPEC](../../tests/benchmarks/agentbench/SPEC.md) §3.4):
there is no human channel, and anything that *would* have needed a human makes the cell
`INVALID` with the cause named. One scoped exception, stated rather than smuggled: **the
operator may provision the venue** (a pod, an install) before the agent's first message — the
analogue of §4.1's "the simulator is installed in the image" — and the provisioning is recorded
in the cell's `venue` field. After the agent's first message, an operator touching the run
makes it `INVALID`.

**Provenance sub-labels** — these are what keep the grid honest:

| field | values | why it exists |
|---|---|---|
| **`reuse_class`** | `authored` (the behaviour was produced during the run) · `assembled` (the agent composed shipped assets it did not modify) · `mixed` | our demos are human-months. "The agent ran our shipped champion" and "the agent produced a walking policy" are different findings and the grid must not let them merge. **A tier where every column is `assembled` is published labelled as measuring shipped assets, not agents.** No prose sentence about a cell may omit this word (§8) |
| **`method`** | e.g. `learned_policy` · `model_based` · `scripted` | a walk is a walk; how it was reached is a fact readers want and is never a pass condition |
| **`hold_mechanism`** (T2) | `friction` · `suction` · `attachment` · `unknown` | §2 T2 |
| **`external_support`** (T4) | peak force / m·g, peak torque N·m, fraction of window | §2 T4 |
| **`distance_to_termination_m`** + **`termination_cause`** (T4) | the distance actually reached, un-clipped, and what ended the run (`fell` · `arena_geometry` · `time_limit` · `controller_stopped` · `unknown`) | §2 T4. **Measurements, never pass conditions.** The bar stays 10.0 m as pre-registered; these publish the headroom without letting it become a bar, and they keep "it ran out of floor" from being read as "it degraded" — which is exactly how the endurance runs would have been misread |
| **`support_attestation`** | `attested` · `unverified` | a cell that cannot prove nothing held the robot up is not comparable to one that can |
| **`venue`** | machine id, GPU, or pod id | [`AGENTS.md`](../../AGENTS.md)'s standing rule: a result that does not name its box is not a result. `python projects/policies/common/env_fingerprint.py` produces the id |

### 3.3 The blocker taxonomy

| code | meaning | who owns it |
|---|---|---|
| `no_importer` | the robot description could not be brought into the simulator by any documented path | the simulator (refutable in the window) |
| `no_physics_capability` | the physics did not produce the outcome (grip slipped, gait fell, distance unreachable) | the simulator, or the state of the art |
| `no_measurement_surface` | the outcome may have happened; nothing exposed a way to measure it (this is the code that most often should have been `NOT_EXPRESSIBLE` — check §3.1's rules before using it) | the simulator |
| `agent_lost` | the agent had a viable path and did not find it | the **product's docs, defaults and error messages**, which is the interesting case |
| `budget_exhausted` | turn / token / timeout cap hit ([SPEC](../../tests/benchmarks/agentbench/SPEC.md) §2: timeout = 3 × par, 60 turns, 400k in / 60k out) — **a `FAIL`, not `INVALID`** | shared |
| `resource_limit` | the machine could not hold the run (VRAM, disk, batch size) | us, or the hardware floor |
| `install_failed` | the simulator did not come up | our scaffolding, until proven otherwise |
| `scaffolding_defect_ours` | we broke it | **us, and it is published as ours** |
| `unknown` | not diagnosed | us |

**`unknown` is capped.** A column in which more than **one third** of blockers are `unknown` is
published as **"not diagnosed"** rather than as a set of findings, because a taxonomy that
hides behind `unknown` is a bare ✗ with extra syllables.

### 3.4 How a cell gets corrected

Publication order is fixed and matches [SPEC](../../tests/benchmarks/agentbench/SPEC.md) §6.2:
**scaffolding, prompts, graders and the empty grid go public before any number.** Then:

1. **30-day correction window**, with an explicit invitation to each simulator's community.
2. A **non-OmniSim reviewer runs each competitor column at least once** (§6.2.4) and their
   notes are published verbatim, criticisms included, before the headline. **If no reviewer can
   be found for a column, that column is not published** — an unreviewed competitor cell is not
   run late, it is not run.
3. Three kinds of correction, each with a committed consequence:
   - **a better scaffold** → it **replaces ours as the published cell** (§6.2.5), and our
     original stays visible with its date rather than being deleted;
   - **a working recipe for a `not_achieved` cell** → the cell is re-run and flipped;
   - **a counter-example to a `NOT_EXPRESSIBLE`** → the label flips and the cell gets run
     (§6.4).
4. Every correction re-runs the affected cell; the revision history stays public; superseded
   cells are struck through with a date, never removed.
5. **No row, no result** ([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md)
   §0.2, standing rule): a ladder cell that exists only in a commit message, an operator's
   note or an agent's summary is quotable nowhere, this document included. Cells live under
   [`tests/benchmarks/agentbench/results_published/`](../../tests/benchmarks/agentbench/results_published/)
   or they did not happen.

---

## 4. The roster, and what each column costs

**Evidence tiers**, per [`simulator-comparison.md`](simulator-comparison.md) §0: 📊 measured by
us · ✅ verified against a primary source · ◐ primary-source extraction, not audited · ⚠️ vendor
claim or contested · ⊘ self-attested against this checkout · **unverified** = we are guessing
and it must be confirmed from their docs before publication.

**Rule, binding: no capability assertion about a competitor may be published at tier
"unverified".** Every one below that carries it is a *task*, with the check named.

### 4.1 OmniSim — local, scaffolding mostly exists ⊘

| tier | native path | status |
|---|---|---|
| T1 | `URDFRobot` import; harness (`:6789`) or a Python controller; `run-headless --fail-on-runaway` for the physics verdict | ⊘ exists; the agent-reachability is the open question |
| T2 | an arm controller with `newtonSolver "mujoco"` pinned; `/sim/grips` for inferred grips | ⊘ exists, **fragile** — §2 T2 |
| T3 | [`projects/policies/training/run_quad_walk_rl.sh`](../../projects/policies/training/run_quad_walk_rl.sh) → `quad_walk_recipe.py`, or `python -m omnisim policy train/run <skill>` | ⊘ verified skill exists; `reuse_class` decides what the cell means |
| T4 | the skill library + the λ = 0.9 balance harness | ⊘ supported only; unsupported is open research |

**Scaffolding to author:** a T1–T4 recorder in the shape
[`adapters/omnisim/`](../../tests/benchmarks/agentbench/adapters/omnisim/) already uses;
graders for the four tiers, **each born with the negative fixtures §5c requires**; a T3/T4
applied-force probe (the `external_support` channel does not exist yet and is the single
largest new build in this programme). Staging reuses
[`cc_lane/stage_workspaces.py`](../../tests/benchmarks/agentbench/cc_lane/stage_workspaces.py)
unchanged.

### 4.2 Upstream Webots R2025a — local, install verified ✅

Stood up for Phase W in WSL2 (`\\wsl$\Ubuntu-22.04\opt\upstream-webots\R2025a`;
[`adapters/webots/BRINGUP.md`](../../tests/benchmarks/agentbench/adapters/webots/BRINGUP.md)),
with a launcher, a Supervisor-side recorder
([`adapters/webots/webots_lane/controllers/agentbench_webots_recorder`](../../tests/benchmarks/agentbench/adapters/webots/webots_lane/controllers/)),
and a grading adapter. **⚠️ Never install it on the dev box** — the Windows installer writes
machine-scope `WEBOTS_HOME` with no uninstall cleanup and shares our QSettings key
([`webots-control-baseline.md`](webots-control-baseline.md) §1).

| tier | native path | evidence |
|---|---|---|
| T1 | first-party `urdf2webots` conversion + a robot controller | ◐ documented by upstream; the conversion was used in Phase W |
| T2 | a hand-written controller; ODE contact parameters | **unverified** — check upstream's `projects/` for a shipped gripper sample and its contact-parameter docs |
| T3/T4 | **no first-party RL training path is known to ship** | **unverified** — check upstream's repo for any RL sample, and whether the documented path is an external stack. Expect `not_achieved(resource_limit / no training path)`, **not** `NOT_EXPRESSIBLE`: the physics can express a walking quadruped, so the absence is of a pipeline, not of a capability |

Upstream is **decaying** — 48 commits in 12 months, last tag R2025a 2025-02-04, a ~28× decline
over four years ✅ ([`simulator-comparison.md`](simulator-comparison.md) §3.1). A Webots column
that reaches a rung we do not is very damaging to us; a Webots column that does not is only the
minimum bar.

### 4.3 Gazebo Jetty 10.0.0 LTS — local, container, free compute

| tier | native path | evidence |
|---|---|---|
| T1 | SDF/URDF via `sdformat`; `ros_gz` + `gz` CLI + ROS 2 `simulation_interfaces` (`SpawnEntity`, `SetEntityState`, `GetEntityBounds`, `StepSimulation`, …) | ◐ implemented natively per `ros_gz/src/gz_simulation_interfaces/` — **confirm the service list from their `.srv` files before publication** |
| T2 | DART contacts + `ros2_control` (hosted first-party by ros-controls ✅) | **unverified** — check whether a first-party gripper/grasp example ships |
| T3/T4 | **unverified** — no first-party GPU-batched RL is known. Check whether Gazebo's documented locomotion path is an external RL stack, and whether that stack is installable in the container | **unverified** |

**Scaffolding to author:** a ROS 2 + `ros_gz` container, a `simulation_interfaces` client with
tool definitions generated **from their `.srv` files** (never paraphrased —
[SPEC](../../tests/benchmarks/agentbench/SPEC.md) §4.2), SDF initial states, a pose recorder, a
hashed docs corpus. Order **2–4 engineer-weeks**. Free in dollars.

### 4.4 Isaac Sim — cloud only for us

**Hardware floor verified ✅:** RTX 4080 / 16 GB VRAM minimum, and **GPUs without RT cores
(A100, H100) are not supported** — the local 3060 is out, so this column runs on a **RunPod
4090**. Planning figures to **verify before committing budget** (quoted, not re-fetched):
~13 GB install + ~80 GB assets; order **$20–40** compute at ~$0.7/hr.

| tier | native path | evidence |
|---|---|---|
| T1 | the URDF importer extension; `isaacsim.ros2.sim_control` (19 services + 1 action ◐) and the Kit Python API | ◐ — **confirm the importer's current name and the service list from NVIDIA's docs** |
| T2 | Isaac's manipulation examples / Isaac Lab manipulation environments | **unverified** — check what ships as a first-party pick-and-place |
| T3/T4 | **Isaac Lab RL** — the column most likely to reach the top rungs | **unverified** — check Isaac Lab's shipped locomotion environments and whether a one-sentence path to a trained walker exists. Note Isaac Lab is **still self-described beta** ✅ and its licence is BSD-3 ✅ while Isaac **Sim**'s is NOASSERTION ✅ |

**Scaffolding to author:** the pod image and bring-up, a USD-side recorder, the tool bridge
generated from their published definitions, the docs corpus. Order **2–3 engineer-weeks**.

### 4.5 MuJoCo / MJX — local, free, and the strongest adversary above T2

| tier | native path | evidence |
|---|---|---|
| T1 | MJCF authoring; URDF ingestion via MuJoCo's own loader; drive via actuators + `mj_step` | ◐ URDF support documented — **confirm the current loader's URDF coverage and mesh handling** |
| T2 | MuJoCo contact + an actuated gripper | **unverified** — check the shipped manipulation examples |
| T3/T4 | **MuJoCo Playground** — quadruped joystick locomotion in ~5 min; Unitree G1 walking in under 30 min on 2× RTX 4090; zero-shot transfer to **six** physical platforms ✅ | ✅ per [`simulator-comparison.md`](simulator-comparison.md) §5.3 — **confirm the exact commands and the hardware the published times assume** |

[SPEC](../../tests/benchmarks/agentbench/SPEC.md) §6.1 already says raw MuJoCo is the strongest
adversary on authoring and that *"we include it because it may beat us."* On T3 and T4 that is
not a possibility, it is the **expected outcome**, and §1's adverse outcome 3 pre-registers it.

### 4.6 Genesis — noted, not a column in pass 1

Apache-2.0, genuinely active (v1.2.3, 952 commits/12 mo ✅), broadest GPU-vendor support ✅.
Excluded from the first pass for one reason stated rather than implied: **its throughput claims
are contested** — a 43M FPS README claim added 2024-12-18 and removed 2026-05-27 with no
explicit retraction ◐, an independent re-run ~150× lower ◐, and a live "10–80× faster" claim
with no stated methodology ⚠️ — and a maintainer disclaims physics-matching guarantees ◐. None
of that says the ladder would go badly for Genesis; it says we would have to author its
scaffolding with no reliable baseline to sanity-check against. **It becomes a column the moment
someone from that community volunteers to author or review it** (§3.4 rule 2), and its absence
is stated in every published grid.

---

## 5. The three traps, as binding rules

### 5a. We author the tasks, and we could pick our own demos

**The outcome-not-feature rule.** A tier statement may not name a file format, an endpoint, a
node type, a solver, a training method, a checkpoint, or any product's proper noun. **The
test**, applied by the reviewer at freeze: *a roboticist who has never used any of these
simulators must be able to read the tier and say what the robot did, and must not be able to
tell which simulator wrote it.* A tier that fails the test is rewritten before freeze or
dropped.

**Where the rule is under strain, admitted:** T2 permits attachment-based grasping and T4
permits a support harness — both are techniques **we personally rely on**. Each is a lever we
gave ourselves. The guard in both cases is that the technique is **measured and printed inside
the cell** (`hold_mechanism`, `external_support`), and that §8 forbids any prose that omits it.
That is a weaker guard than not having the lever, and it is recorded as such.

**The external-task-source commitment.** At least **one tier's exact wording** must come from
outside this project — solicited from a competitor community, lifted from a published
benchmark, or taken from a real user request — before any grid is *published*
([SPEC](../../tests/benchmarks/agentbench/SPEC.md) §8.3; V4 in
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §1.1). **If the external
wording disagrees with ours, the external wording is the one that runs.** Until an external
source exists, every grid carries the sentence *"tiers authored by the simulator's own
maintainers"* in its header.

**Freeze and no-escape.** The four tier statements, their thresholds, their prompts, their
graders, the roster and the n are frozen together and the manifest hash published in
[`tests/benchmarks/agentbench/preregister/`](../../tests/benchmarks/agentbench/preregister/)
before the first cell. **Adding a tier, removing a tier, re-wording a tier, moving a threshold,
or dropping a simulator because its column went badly, each voids the pass** and forces a
version bump and a full re-run
([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.4's no-escape clause,
applied here verbatim).

### 5b. Competence asymmetry — we know our tree and barely know Isaac

This is the trap most likely to produce a false result, and it cuts entirely one way: we have
years of tacit knowledge of one column and none of the others.

**Binding mitigations**, all from [SPEC](../../tests/benchmarks/agentbench/SPEC.md) §6.2 and
none optional:

1. **Scaffolding published before any number**, in a PR-able location, with the empty grid.
2. **30-day correction window** with an explicit invitation to each community.
3. **A non-OmniSim reviewer runs each competitor column**, notes published verbatim. **No
   reviewer, no published column.**
4. **A maintainer's better scaffold becomes the published cell.**
5. Tool definitions generated from **their** published service/API definitions, never
   paraphrased by us (§4.2).

**And one this programme adds, because the others do not measure the asymmetry:**

6. **The effort-parity ledger.** For every column we publish the **engineer-hours spent
   building its scaffolding** and the number of debug iterations it took to get the first cell
   to run. "We spent three weeks on ours and two days on theirs" is the asymmetry, and it is
   measurable rather than merely confessable. **Any column whose scaffolding effort is below
   half the OmniSim column's is labelled `under-invested` in the grid and in every prose
   sentence that mentions it** (§8).

### 5c. Our own demos are human-months, and the programme must be willing to publish ✗ in our own column

Three mechanisms, all pre-committed:

1. **The pre-registered expectation table.** Before any cell runs, we write down what we expect
   **each OmniSim cell** to be, and the published report prints expectation beside outcome.
   This document already contains the first draft of it (§2's honest notes, collected in §6.4),
   and it is pessimistic on purpose: T2 expected to fail, `T4-unsupported` expected to fail,
   T3 expected to pass only as `assembled`, T1 genuinely uncertain. A rosy expectation that
   comes true is visible as a possible design artifact; a pessimistic one that comes true is
   evidence the design was not built to flatter.
2. **The red-evidence rule**, standing and binding
   ([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §5.5): **no assertion
   enters a ladder cell until it has been observed FAILING on a deliberately wrong artifact,
   with that negative fixture named in the assertion's record.** A green assertion is not
   evidence that the assertion works — for weeks half of `A1.3` could not fail, because
   `ContactPoint.node_id` returned the queried solid's own id and every contact pair was keyed
   `(id, id)`. **A `null` agent turning every assertion red does not satisfy this rule.** Each
   tier's graders are born with their fixtures or they do not enter the freeze: a teleported
   arrival for T1.3, a welded prop for T2.4, a crawling robot for T3.3, a robot on a crane for
   T3.4/T4.
3. **The answer-key ruling, and what it deliberately leaves in.** Cells run in staged
   workspaces excluding the benchmark's own answer key — `tests/benchmarks/agentbench/**`, this
   document, and anything revealing thresholds or task internals — with the staging manifest
   and every redaction published as a before/after diff
   ([`cc_lane/stage_workspaces.py`](../../tests/benchmarks/agentbench/cc_lane/stage_workspaces.py);
   [`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.7 Amendment 2).
   **Capability documentation stays** — the skill library, `AGENTS.md`'s locomotion routing,
   the harness reference are *product*, and doctoring them would void the comparison. The
   consequence is faced rather than hidden: an OmniSim agent can legitimately reach T3 by
   invoking a shipped champion, which is why `reuse_class` exists and why §8 forbids quoting a
   T3/T4 cell without it.

---

## 6. Sequencing and cost

**Cheapest-decisive-first**, and the first decisive cell is one of ours.

### 6.1 The order

| # | step | venue | cost | gate |
|---|---|---|---|---|
| 0 | **Freeze.** Tier wording, thresholds, prompts, graders (**born with negative fixtures**, §5c), the roster, n = 3, the expectation table. Publish the manifest hash in `preregister/` | free | days of engineering | nothing runs before this |
| 1 | **T1 × {OmniSim, Webots}** — both installs already exist | local, CPU | tokens only | 🚦 **if an agent cannot reach T1 on OmniSim from one sentence, publish that and stop the pass** |
| 2 | **T2 × {OmniSim, Webots}** | local, CPU | tokens only | the tier we expect to lose in our own column |
| 3 | **+ MuJoCo column at T1, T2** | local, CPU | 1–2 engineer-weeks + tokens | the strongest authoring adversary, free to run |
| 4 | **T3 × OmniSim on M1 (RTX 3060 laptop)** | local, GPU | tokens; likely `resource_limit` | run it *because* the likely outcome is a resource blocker — that is a real cell and it is free |
| 5 | **T3, T4 × {OmniSim, MuJoCo} on RunPod 4090** | 🔒 **cloud — owner approval required** | compute order **$5–20** (see 6.2); engineering is the real cost | no cloud spend without §6.3 |
| 6 | **+ Gazebo column** | local container | 2–4 engineer-weeks, free compute | publish scaffolding + window before any number |
| 7 | **+ Isaac column** | 🔒 **cloud — owner approval required** | order **$20–40** compute (verify); 2–3 engineer-weeks | as above, and a non-OmniSim reviewer must exist first |

Steps 1–4 are free in dollars and are the entire first pass. Steps 5–7 each require the owner's
per-campaign approval separately; approval for one is not approval for another.

### 6.2 What the cloud actually costs, and what we do not know

**T3/T4 compute is cheap; the engineering and the tokens are not.** The anchor from this tree:
three RunPod sessions producing three quadruped results — a certified turn skill, a falsified
saturation hypothesis, and a foldability sweep — cost **≈ $2.7 total**
([`rl-current-state.md`](rl-current-state.md), 2026-07-17 banner). The shipped Go2 champion's
config is recorded (K = 16384, 400 iterations, 727,583 env-steps/s cumulative on a RunPod
4090 — [`skill.json`](../../projects/policies/skills/quadruped/go2_shadow_walk/skill.json)),
but **its wall clock is not recorded in this tree**, so any pod-hour figure here is an
assumption. **The first action of any T3 campaign is to measure one run's wall clock and
re-derive the budget from it**, exactly as
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §3.1 requires of the token
arithmetic.

**Tokens.** The ladder's first pass is at most 4 tiers × 3 repeats × 5 simulators = **60
cells**, far fewer than Phase W's 70 per simulator. At the only figure we hold — **80k–350k
tokens per run**, an *operator observation, not an instrumented figure*
([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §3.1) — that is **4.8 M to
21 M tokens**, a factor-of-four spread that is an artifact of never having recorded tokens
properly. T3/T4 cells involve long-running training and their token counts are unknown and
probably higher. Every cost figure in this section is a **planning figure to be replaced by
the first ten instrumented cells**, not a budget.

**Pod discipline is not ours to improvise** ([`AGENTS.md`](../../AGENTS.md)): **arm the
delete-watchdog before anything else**, batch and detach, write results to the network volume
because a pod can vanish mid-run, **TERMINATE rather than stop**, and confirm
`GET /v1/pods` returns `[]` at the end. Two traps that have each cost a whole experiment: the
GPU wheels must land in **both** the engine-linked interpreter **and** the `python3` that
spawns **controllers** (a missing `onnxruntime` there silently runs a zero-residual baseline
and exits 0 — **assert the `ONNX loaded:` line, never the exit code**), and the volume is NFS
root_squash so tar needs `--no-same-owner`.

### 6.3 The cloud gate

> **No cloud spend happens without the owner's explicit, per-campaign approval.** The approval
> names the campaign, the date, and a **dollar ceiling**; it is recorded in the campaign record
> alongside the pre-registration hash. **A campaign that reaches its ceiling stops** — it does
> not continue and reconcile afterwards. Approval for step 5 is not approval for step 7.
> Cloud is never the agent's idea; local is the default venue.

### 6.4 The instrument constraint, and the expectation table

**The instrument.** Ladder cells run **Claude Code**, headless, pinned to a CLI version and a
model id recorded in every row — the same product-level instrument Phase W used (Claude Code
2.1.179 at `claude-fable-5`, per `124ae7f3`), for the same reason: the question is about the
product an agent actually meets. The stated weakening rides along unchanged
([`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §5.11): Claude Code is a
moving product we cannot byte-hash, so **replication means "at this version"**, cost and turns
are the instrument's **self-reported** figures (`metrics_source`), and generalising beyond the
pinned instrument is forbidden (§8).

**The quota is the scarcest resource in this programme — scarcer than dollars.** The pinned
model's weekly quota **resets 2026-08-07**, and Phase W already lost three `A1` cells to
exactly this: *"the model's weekly quota expired mid-campaign and switching models would have
voided the comparison, so the lane stopped rather than substituting an instrument"*
(`124ae7f3`). Binding rules, inherited:

- **A cell may never be run on a substitute model to beat the quota.** A quota exhaustion is a
  **pause**, recorded as a deferred attempt and retried on the same cell (the recogniser exists
  in [`cc_lane/concurrency.py`](../../tests/benchmarks/agentbench/cc_lane/concurrency.py)), and
  it is **never** a failed run.
- **Spend the quota cheapest-decisive-first.** Step 0 is the only step that can proceed before
  the reset; step 1 is where the first quota should go.
- A version drift mid-campaign voids the affected cells.

**The expectation table (pre-registered; §5c).** Written before any cell runs:

| tier | OmniSim expectation | reasoning |
|---|---|---|
| **T1** | **genuinely uncertain** — I would not bet on 3/3 | capability present; Phase W's authoring control was 0/5 on our arm, on relative asset paths |
| **T2** | **`not_achieved`** expected | the free-object pinch is unfixable by parameter here; the working grasp needs a solver pin an agent is unlikely to find; the shipped success needed a purpose-built suction tool |
| **T3** | **`achieved`, `reuse_class: assembled`** expected | a verified champion ships and has a CLI front door; producing one from scratch in a session is not plausible |
| **T4-unsupported** | **`not_achieved`** — a stated ✗ in our own column | free-standing durable humanoid walk is open research here |
| **T4-supported** | **`achieved`, `reuse_class: assembled`** expected — *revised 2026-08-02, before the freeze and before any cell ran* | the distance was measured, not assumed (§9 Q1): 10.0 m at t = 82.9 s, 6/6 runs, zero falls, on the harness (`86c6522e`). **The prior pre-registration, kept visible rather than deleted:** *"uncertain, leaning `not_achieved` on distance — flagship measures 0.120 m/s; 10 m ≈ 83 s of continuous walking, longer than any G1 bout recorded in this tree."* That expectation was refuted by measurement; **the 10.0 m bar it was written against did not move** (§2 T4) |

---

## 7. What this programme does not claim

**It is a capability map, not a head-to-head score.** It cannot rank the simulators, it cannot
say one is better, and it cannot be converted into one. Specifically:

- **It says nothing about cost-to-outcome on a fair task set.** That is E1's question and Phase
  W's lane, and this programme's tiers deliberately violate the fairness constraint that lane
  requires (a tier may be reachable on one simulator and not another — that is the *point*
  here and would be *rigging* there).
- **The Phase W head-to-head result stands as its own narrow finding, cited and not
  superseded.** 67 Claude Code cells across two products, one pinned instrument, each simulator
  staged as shipped with the answer key quarantined: upstream Webots led completion on `B1`
  (4/5 vs 3/5) and `B2`, OmniSim led `B3` (2/5 vs 1/5), the two debug tasks **tied 5/5** with
  mirrored cost profiles (upstream answers `C1` in half the calls, OmniSim proves `C2` in half),
  the `A1` authoring control was **0 on both arms**, and three `A1` cells remain unrun on the
  quota (`124ae7f3`; rows under
  [`results_published/`](../../tests/benchmarks/agentbench/results_published/)). **A favourable
  ladder grid does not soften that, and if one appears, both must be quoted in the same
  paragraph.**
- **It says nothing about physics fidelity.** That is OmniBench
  ([`omnibench-2026-07-24.md`](../benchmarks/omnibench-2026-07-24.md)), and a reachability cell
  is not evidence a simulation is right. A robot can walk ten metres in a wrong world.
- **It says nothing about whether the endpoints exist.** That is lane 3c
  ([DRIVEABILITY.md](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md), 10/10 on OmniSim
  📊) — a different and already-measured question.
- **It does not generalise past the pinned instrument.** Every cell is one shipped coding agent
  at one version. A reader comparing simulators through this grid is also comparing how well
  each product fits that one agent.
- **`achieved` does not mean reliable.** At n = 3 with a ≥ 1-of-3 bar, `achieved` means *reached
  at least once*. Ladder cells are `exploratory` and are barred from any published pass rate
  ([SPEC](../../tests/benchmarks/agentbench/SPEC.md) §3.5).

**The Lane C quarantine remains in force where it was written.** Promoting the frontier into
this programme does not license quoting Lane C numbers inside a Phase W report; the two
artifacts stay separate, and
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md) §2.1's quarantine binds that
document unchanged.

---

## 8. The forbidden sentences

Exact sentences we may not write, not sentiments to avoid.

**About exclusivity — every one of these is refutable by a maintainer with one recipe:**

- ❌ "the only simulator where an agent can make a robot walk" — or *any* sentence of the form
  "the only simulator that…"
- ❌ "OmniSim is the only column that reaches T3" (or T2, or T4, or any tier)
- ❌ "no other simulator can do this" — the honest form is *"no other column in this grid
  reached this rung, at this instrument, with the scaffolding we published, before the
  correction window closed"*, and if that sentence is too long to use, use no sentence
- ❌ anything with **"first"**

**About the cells:**

- ❌ a bare ✗ or ✓ anywhere — in a table, a chart, a slide, a README
- ❌ any capability negation about a competitor without a cited blocker **and** a closed
  correction window
- ❌ rendering a `NOT_EXPRESSIBLE` in a failure's colour, in any chart
- ❌ quoting a **T4** cell without its measured support number
- ❌ quoting a **T4** cell without its `termination_cause` — a run that stopped because it ran
  out of floor may never be described as a walk that degraded, a stall, or a fall
- ❌ quoting a **T3 or T4** cell without its `reuse_class`
- ❌ quoting a **T2** cell without its `hold_mechanism`
- ❌ describing a G1 result without the balance-harness disclosure ([`AGENTS.md`](../../AGENTS.md))
- ❌ mentioning a column labelled `under-invested` without that word in the same sentence
- ❌ quoting a cell's cost or turn figures without the `metrics_source` caveat
- ❌ quoting a ladder cell as a **pass rate** — n = 3, `exploratory`, `achieved` = at least once

**About aggregation:**

- ❌ any **cross-column count or ratio** — "OmniSim 3 of 4, Gazebo 1 of 4" — because it implies
  the tiers are of commensurate difficulty and hides `reuse_class` and the support labels.
  Per-column enumeration *by tier name* is allowed: *"on OmniSim the grid records T1 and T3
  (assembled) achieved, T2 and T4-unsupported not achieved."*
- ❌ mixing ladder cells into any Lane A/B aggregate, or into any AgentBench score
- ❌ "the capability ladder shows agents get more done on OmniSim" — the ladder measures
  reachability, not throughput, and that sentence is E1's, unmeasured
- ❌ presenting a ladder result as evidence for or against E1's conjunct (i) or (ii), in either
  direction

**About what a run proves:**

- ❌ "the robot walks" from a run in which the recorder did not integrate the trajectory — an
  exit code is not evidence, and a policy that never loaded exits 0
- ❌ any number that exists only in a commit message, an operator's note, or an agent's summary
  — **no row, no result**

---

## 9. Open questions, recorded rather than resolved

1. ~~**Is 10 m the right distance for T4, and is it reachable even harnessed?**~~ **CLOSED
   2026-08-02 — measured, and the bar did not move.** The question as posed: *"at the flagship's
   measured 0.120 m/s that is ≈ 83 s of continuous walking, and no bout of that length is
   recorded in this tree. Measure it before the freeze. If it is unreachable, publish that as the
   tier's finding — do not lower the bar after seeing a cell."* It was measured
   ([`g1-endurance-2026-08-01.md`](g1-endurance-2026-08-01.md), `86c6522e`): **10.0 m at
   t = 82.9 s, 6/6 runs, zero falls, 29.5 m demonstrated in a longer arena**, on the λ = 0.9
   weight-bearing balance harness. The bar **stays at 10.0 m verbatim** (§2 T4) — a
   pre-registered threshold that moves after measurement is worthless whichever way it moves, and
   raising it to a distance we had just watched our own robot clear would be selecting the axis
   after the fact. The headroom is published as a per-cell **measurement**
   (`distance_to_termination_m`, `termination_cause`) instead, and the tier's resulting weakness
   — 10 m is about a third of our artifact's demonstrated reach, so most walking columns will
   clear it — is stated in §2 T4 rather than quietly carried.
2. **Does the `external_support` channel exist on any simulator but ours?** It is the largest
   new build in the programme and the assertion T3.4/T4 both rest on. If most columns can only
   answer `support_attestation: unverified`, the T3 "nothing held it up" assertion becomes
   nearly unenforceable cross-column, and the tier needs re-thinking before freeze, not after.
3. **Is `reuse_class` cleanly decidable?** "The agent composed shipped assets it did not
   modify" has a fuzzy boundary — an agent that edits three env vars in a shipped recipe is
   what? Proposed rule for the freeze: `authored` requires that the deliverable's *behaviour*
   would not exist without work done during the run; anything else is `assembled` or `mixed`,
   and the reviewer arbitrates. Not yet good enough.
4. ~~**Should T1 really ship raw URDF on every column?**~~ **CLOSED 2026-08-02 — yes, with the
   meshes, and T2 diverges on purpose.** The question as posed: *"it makes the importer part of
   the measurement (§2 T1), which is right for a capability map and wrong for a cost comparison.
   The risk is that one column fails T1 on a mesh-path quirk and the grid reads as a capability
   verdict."* Decided: **T1 ships the real robot description unmodified, meshes and
   `package://` references intact**, because ingesting a real robot description is what that tier
   measures, and the MuJoCo bring-up demonstrated those meshes navigable through that column's
   own first-party settings. **T2 ships primitive geometry and not one mesh**, because that tier
   is decided by collision geometry and grasp physics and a mesh-decoding gap there would be a
   pure confound. The countermeasure below stays available and is deliberately **not**
   pre-emptively applied — applying it everywhere would delete the measurement. Both halves are
   recorded in the task files and in §2 T1 / §2 T2. Original countermeasure text, retained:
   publish `not_achieved(no_importer)` **and** re-run that column's T2–T4 with a pre-converted
   robot, so an importer defect does not silently truncate a whole column.
5. **Who reviews the Isaac and Gazebo columns?** §3.4 rule 2 says an unreviewed column is not
   published. We have no reviewer for any column today. This gates steps 6 and 7 entirely, and
   it should be started *now*, in parallel with step 0, because finding one takes longer than
   building the scaffolding.
6. **Does the answer-key ruling survive contact with T3?** Leaving the skill library in the
   workspace is right (it is product), but it means the OmniSim T3 cell is close to
   "can the agent read `AGENTS.md` and run one command." That is a legitimate product finding
   and a nearly uninformative capability finding. `reuse_class` records it; whether the tier
   should *also* carry a from-scratch variant is undecided, and the from-scratch variant would
   be expensive and probably fail on every column.
7. **What happens if the ladder is favourable?** The temptation to quote it as vindication of
   the positioning sentence Phase W did not support will be strong, and §7 plus §8 are the only
   things standing in the way. Decide before the first grid ships whether a favourable grid is
   published at all before the external-task column (§5a) exists. Current answer: **no** — the
   header sentence *"tiers authored by the simulator's own maintainers"* is not a fig leaf we
   should be willing to publish a headline behind.

---

**When this file and the code disagree, the code wins — and update this file in the same
change.** When this file and a marketing sentence disagree, this file wins.
