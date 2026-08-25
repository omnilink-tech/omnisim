# BuildBench — the capability suite

**Suite id: `buildbench/v0.1`.**

**Status: DECLARATION ONLY.** Five tasks are registered. **Zero have a world, a
grader, an oracle or a null.** No cell has been run, no number exists, and
nothing in this suite may be quoted as a result. The one thing that has been
*measured* is a negative: B2's subsystem does not work (§6.2), and that
measurement is recorded here rather than allowed to become a task.

---

## 0. What this suite is, and what it is not

BuildBench asks one question:

> **What robotics work can each simulator express?**

That is a different question from the one
[`tests/benchmarks/agentbench/`](../agentbench/SPEC.md) asks, and BuildBench is
a **sibling** of AgentBench, never a replacement for it. AgentBench is where the
freeze discipline, the oracle/null gate and this tree's only credible
cross-simulator results live — including upstream Webots passing R1 cleanly and
R4's oracle going 9/9. **That record is the evidence that we do not rig
outcomes, and it must survive.** Nothing in BuildBench renames, moves, converts
or supersedes it.

| suite | asks | verdict |
|---|---|---|
| `omnibench` lanes 1–2 | is the physics right, how fast | numeric error / throughput |
| `omnibench` lane 4 | does *this* engine do X, measured in physical units | `works` / `partial` / `broken` |
| `agentbench` | given one sentence and no human, did an agent get the job done | task completed / not |
| **`buildbench`** (this file) | **what work can each simulator express at all** | expressible / not, with evidence |

### 0.1 BuildBench is a capability suite, NOT a fairness benchmark

This distinction is the whole contract and it is stated first because everything
downstream depends on it.

**A fairness benchmark** — AgentBench — holds the agent, the prompt, the budget
and the tool floor identical across simulators so that a *difference in outcome*
is attributable. It earns the right to say "A did better than B". It pays for
that right with pre-registration, a bare-shell control condition, a 30-day
correction window, reviewer-run cells, and a standing promise to adopt a
maintainer's better scaffold as the published headline.

**A capability suite** — BuildBench — makes no such comparison and earns no such
right. It asks whether a piece of robotics work has an honest formulation on a
given simulator at all. Its output is a **matrix of expressibility with
citations**, not a leaderboard.

Consequences, binding:

- **BuildBench produces no score, no ranking, and no "wins".** There is no
  aggregate, no percentage, and no chart in which simulators are ordered.
- **A `NOT_EXPRESSIBLE` verdict is not a defeat for that simulator**, and may
  never be drawn in the same colour as a failure — the same rule AgentBench
  §6.4 and §11 already carry.
- **BuildBench may not be used to make a comparative claim.** If a comparative
  claim is wanted, the task belongs in AgentBench under AgentBench's discipline.
- Where a task *is* expressible everywhere, that is the expected and unremarkable
  case. It is recorded exactly as plainly as the other kind.

### 0.2 The four rules that keep it honest

**Rule 1 — every task must be genuine robotics or simulation work.**

A task enters this suite only if a practitioner would recognise it as work worth
doing, stated **independent of who can run it**. Every task therefore carries a
mandatory `why_this_is_real_work` field that must stand on its own merits: it
must not mention a simulator, a competitor, or a capability gap. If the
justification only makes sense once you know who cannot do it, the task is
reverse-engineered from a competitor's gaps and it does not belong here.

This is the rule most likely to be violated by accident, because the tempting
task is exactly the one we can do and they cannot. The test
[`test_declarations.py`](test_declarations.py) enforces the mechanical half
(no simulator names in the justification); the judgement half is on the author
and the reviewer.

**Rule 2 — `NOT_EXPRESSIBLE` is a first-class verdict and must carry evidence.**

Adopted wholesale from AgentBench §6.4 and tightened. The label requires:

- the **specific** missing capability named — a verb, a device, a node type, a
  service — not "it can't really do this";
- a **citation** to that simulator's *own* documentation, API listing or source;
- a `verification_status` recording how well established the claim is (§2);
- and the standing rule that **a single credible counter-example flips the label
  and the task gets run**.

An assertion without a citation is not a finding. It is marked `UNVERIFIED` and
it stays that way until someone does the work, and the matrix shows it as
unverified wherever it is displayed.

**Rule 3 — where a competitor CAN express a task, record it plainly.**

Stated positively because the failure mode is silence, not lying. A suite in
which we express everything and the competitors express nothing is not credible
and should not be believed — including by us. If the matrix ever comes out that
way, the correct response is to assume the task set is bad and go looking for
the bias, not to publish it.

Corollary: the suite is required to carry tasks that competitors express and we
do not, or cannot yet. Two are already known and are named in §5 as candidates
rather than left for a critic to point out.

**Rule 4 — the oracle/null gate (AgentBench §7.1) applies unchanged.**

> **A task nobody can demonstrably complete is not a capability claim.**

Before a task may report anything about any simulator, that `(task, simulator)`
cell needs an **oracle** that demonstrably completes it and a **null** that
demonstrably fails it. Without the oracle the "capability" is a hypothesis;
without the null the grader may be passing everything, including doing nothing.

The gate is **per `(task, simulator)`**, not per task. Our own arm is not
exempt, and is in fact where this bites first: a task we declare
`EXPRESSIBLE` on OmniSim and cannot demonstrate on OmniSim is a claim about our
marketing, not our engine.

---

## 1. What is reused from AgentBench, and what is deliberately not

**Reused — do not fork these:**

| machinery | where | why |
|---|---|---|
| `Verdict` / `Assertion` / `Falsifier`, the progress ordinal, the outcome vocabulary | `agentbench/graders/verdict.py` | assertions in physical units, the vacuous-clause detector, `PASS` == every assertion |
| the oracle/null gate | `agentbench/preregister/run_oracles.py` | §7.1, quoted above as Rule 4 |
| row writer, machine attribution, fingerprinting | `omnibench/common/results.py` | one machine-id mechanism for the whole tree |
| the grader-owned recorder pattern | `agentbench/controllers/agentbench_recorder/` | measure from a recorded run, never from a harness poll |
| physical-unit graders + per-sim adapters | `agentbench/graders/`, `agentbench/adapters/` | a grader that reads an OmniSim-shaped response is a bug |

**Not reused, because BuildBench does not make the claim they defend:**

- the `shell` / `shell+tools` split — that is a fairness control for an
  attributable comparison, and BuildBench does not compare;
- the pre-registration freeze — BuildBench has no scored campaign to protect
  from post-hoc reshaping. **If BuildBench ever runs a scored campaign, the
  freeze comes with it**, and this line is what gets deleted;
- per-cell budgets, ceilings, token accounting, `pass@1` — there is no agent in
  the loop of a capability declaration.

---

## 2. `verification_status` — the field that stops this becoming marketing

Every expressibility claim, on every simulator including ours, carries one of:

| status | means | what it takes |
|---|---|---|
| `UNVERIFIED` | **the default.** Somebody's belief. Nobody has checked. | nothing — this is where every claim starts |
| `CITED` | supported by a citation to that simulator's own docs, API listing or source | a resolvable reference, quoted |
| `MEASURED` | established by a run on a named machine, with an evidence record | an evidence file under `evidence/`, naming the machine, the binary and the date |
| `REFUTED` | a credible counter-example flipped it | the counter-example, recorded; the label changes with it |

Two rules that make the field load-bearing rather than decorative:

1. **A claim at `UNVERIFIED` may not be published, quoted, or drawn in a
   matrix without the word `UNVERIFIED` beside it.** Not in a footnote.
2. **`MEASURED` is the only status that licenses a statement about behaviour.**
   `CITED` licenses a statement about a documented interface, which is a
   materially weaker thing — a device that appears in an API reference and
   silently does nothing is precisely the failure mode
   [OmniBench lane 4](../omnibench/lane4/README.md) exists to catch, and we have
   found it in our own engine more than once.

**Every competitor claim in this file is currently `UNVERIFIED`**, including the
ones that feel obvious. Several are recorded with an explicit **challenge**
(§4): a reason to suspect the claim is wrong, written down before anyone runs
anything, so the suite cannot quietly keep a convenient error.

---

## 3. Layout

```
tests/benchmarks/buildbench/
  SPEC.md              <- this file (the contract)
  tasks.py             <- the task registry: declarations as CODE, not prose
  tasks/<id>/
    meta.json          <- the declaration, serialised from tasks.py
    prompt.txt         <- the exact agent-facing prompt
  evidence/            <- measurement records; the only thing that makes a
                          claim MEASURED
  test_declarations.py <- enforces the honesty invariants mechanically
  README.md            <- orientation for a reader who lands here first
```

The registry is **code, not prose** — the same decision, for the same reason, as
AgentBench's `sims.py`: a runner, a matrix renderer and a report cannot disagree
about what was declared if there is only one declaration.

---

## 4. The five registered tasks

Full declarations, including per-simulator reasons and challenges, are in
[`tasks.py`](tasks.py) and mirrored per task into `tasks/<id>/meta.json`. The
summary:

| id | capability demonstrated | build status |
|---|---|---|
| **B1** `trained_locomotion_deploy` | train a locomotion policy in-engine, then deploy it to complete a sensor-guided navigation course | declaration only |
| **B2** `granular_traversal` | interact with granular/regolith media in a graded way | **BLOCKED — subsystem does not work (§6.2)** |
| **B3** `robustness_distribution` | report a success *distribution* over ~1000 randomised draws, not a single run | declaration only |
| **B4** `multi_robot_radio` | ~20 robots coordinating over a radio device model, collision-free | declaration only |
| **B5** `procedural_generalization` | train across seeded procedural worlds, grade on an unseen seed | declaration only |

Three simulators are declared per task: `omnisim`, `webots` (upstream R2025a),
`mujoco`. They are the three AgentBench has adapters for or is closest to having
— extending the matrix to Gazebo, Isaac, CoppeliaSim or Genesis without an
adapter would be adding unverified rows to an already unverified matrix.

### 4.1 The challenges, recorded before anyone runs anything

These are reasons to think our own declarations are wrong. They are written into
the registry as `challenges` and surfaced by the matrix renderer.

- **B1 / B2 / B3 — "MuJoCo has no Lidar or Camera device model" is probably
  false as stated.** MuJoCo ships a `rangefinder` sensor and offscreen rendering
  (`mjr_render`), and MuJoCo-based embodied-AI stacks routinely produce camera
  observations. The defensible narrower claim is about *device model plus
  per-robot controller processes as a packaged simulator concept*, not about
  whether ray casts and pixels can be obtained at all. Until someone cites
  MuJoCo's own documentation either way, the row stays `UNVERIFIED` and the
  broad phrasing is not to be repeated.
- **B4 — "Webots has Emitter/Receiver but ODE throughput collapses at scale" is
  a claim about a *different engine version than the one it names*.** Upstream
  Webots R2025a is not this tree's deleted ODE path, and its scaling has not
  been measured by us at 20 robots. Recorded as `UNVERIFIED`; the honest
  formulation is a measurement, not an inherited belief.
- **B4 — the task collides with a defect in OUR OWN engine and would measure
  it.** See §6.1, risk 2.
- **B1 / B3 / B5 — our own side may not be expressible either.** See §6.1,
  risk 1: if cameras do not render in the batched GPU path, the sensor half of
  these tasks does not exist on OmniSim, and the declaration is wrong about
  **us**.

---

## 5. Tasks the suite still owes, so the matrix is not all one colour

Rule 3 requires tasks competitors express and we do not. Two are already known
and are named here rather than waiting to be pointed out:

- **`ros2_control` integration.** ⚠️ This entry used to say OmniSim had no ROS
  bridge at all and that this was a declared non-goal. That was reversed on
  2026-08-17: OmniSim implements `simulation_interfaces` plus a live topic
  surface as a sidecar ([`packages/omnisim-ros2/`](../../../packages/omnisim-ros2/)),
  so a ROS 2 *scene-control* task is now EXPRESSIBLE on OmniSim. What is still
  honestly `NOT_EXPRESSIBLE` is narrower and should be registered as such: a
  **`ros2_control` / MoveIt / Nav2** task, which needs a
  `hardware_interface::SystemInterface` plugin OmniSim does not have
  ([docs/developer/ros2-integration.md](../../../docs/developer/ros2-integration.md)).
- **Per-entity spawn / delete / set-pose as first-class services.** AgentBench
  §6.1 already records this as the tier where we expect to lose: Gazebo has
  `SpawnEntity` / `DeleteEntity` / `SetEntityState` / `GetEntityBounds` as
  services. (OmniSim has since grown `POST /scene/spawn`, `/scene/delete` and
  `/scene/set_pose` on the harness, so this one needs re-checking rather than
  assuming either answer.)

Neither is registered yet. **The matrix may not be published until at least one
task that OmniSim does not express is registered and graded**, because a
one-sided matrix is not evidence, it is a brochure.

---

## 6. Risk register — assumptions that threaten the declarations

These are recorded because they are **assumptions, not measurements**, and each
one, if wrong, invalidates specific tasks.

### 6.1 Open risks

**Risk 1 — do sensors (camera / lidar) work in the batched `mujoco_warp` GPU
path?** Suspected: cameras do **not** render under batching. **Threatens B1, B3,
B5** — all three assume a policy trained in the batched path can consume, or be
graded against, sensor data. Status: `UNVERIFIED`. This is the highest-value
check in the register: it decides whether three of five tasks are expressible
**on OmniSim**, i.e. it is a risk to our own claims first.

**Risk 2 — B4 collides with a known engine defect and would measure our own
overflow.** `WorldInfo.newtonNjmax` / `newtonNconmax` default to **256** and
overflow **silently** on `mujoco_warp`: the constraint vector is truncated and
contact solving degrades with no error a user will ever see (the only warning is
a `wp.printf` from inside the solver kernel, discarded entirely on Windows where
`omnisim-bin.exe` is a GUI-subsystem binary). A wheeled robot resting on four
wheels contributes ~32 constraint rows, so ~20 robots is far past the default.
**B4 must set the per-world fields deliberately** — or it is not measuring
multi-robot radio coordination, it is measuring an unset default. Note that
OmniBench lane 4b currently **cannot reproduce** the overflow on generated
rovers and reports `cliff_detector_validated: false`, so the threshold is
unconfirmed on the present runtime in *either* direction; that uncertainty does
not license leaving the fields unset. Status: `UNVERIFIED` as a threshold,
`CITED` as a hazard.

**Risk 3 — B2 needs a robot to interact with granular media in a graded way.**
**RESOLVED, negatively.** See §6.2.

### 6.2 Risk 3, resolved: the granular subsystem does not support a graded task

Measured 2026-08-11 on machine `9722d23d12a3` (RTX 3060 Laptop, driver 596.36)
with `msys64/mingw64/bin/omnisim-bin.exe` (built 2026-08-10). Evidence record:
[`evidence/2026-08-11-granular.md`](evidence/2026-08-11-granular.md).

**Verdict: B2 is `BLOCKED`. It is not a viable task and no task was authored
around it.** Four independent blockers, any one of which would be sufficient:

1. **Particles do not simulate at all on the shipped binary.** Loading
   `tests/cuda/granular_group_load.omniworld` logs
   `WARNING: GranularGroup is inert: CUDA is not available on this build/box.`
   The box has an RTX 3060 and CUDA 12.1, but the binary was built with
   `OMNISIM_WITH_CUDA=OFF` (the flag auto-detects `nvcc` on `PATH` at build
   time). There is **no CPU fallback** — the node is inert, and the world still
   loads and still reports the run as fine.
2. **Even with CUDA on, the robot↔particle collider list is always empty.**
   `OmGranularGroup::collectColliders` skips any Solid whose `body()` is `NULL`;
   `OmSolid::body()` returns the ODE body, and ODE was deleted in `bdc02139`, so
   it is now `NULL` for **every** Solid. The kernel's collider loop runs zero
   iterations. Robots and particles are mutually invisible — **not one-way, not
   two-way, not at all.**
3. **The reverse force is written into an empty function.**
   `OmSolid::addForceAtPosition` (`src/omnisim/nodes/OmSolid.cpp:4221`) has an
   empty body, with the tree's own comment saying so: *"the granular-group
   coupling (OmGranularGroup) that call them apply nothing."*
4. **There is no way to read particle state.** OmniBench lane 4 already records
   this: *"this lane has NO way to read particle state through the supervisor
   API, so 'the particles are simulated' is UNMEASURED."* A grader could not
   measure displaced mass even if 1–3 were fixed.

**Two findings from the same measurement that correct existing documents:**

- **The 294-sphere `granular_sand_demo.omniworld` is NOT frozen**, contrary to
  `docs/developer/codebase-audit-2026-07.md` §12.6 (an internal audit, not in the
  public snapshot), which claimed that after the ODE deletion "the sand has no
  physics … the demo shows a frozen pile". It registers **294 dynamic Newton bodies** and the grains fall
  (z 1.593 → 0.048) and land. It never used `GranularGroup` — it is ordinary
  rigid spheres — so the ODE deletion did not freeze it.
- **It is, however, physically unsound as it stands.** The grains escape a pit
  whose walls sit at ±1.4 m and roll outward without bound: at t ≈ 123 s one
  grain reads (−34.6, −80.2) m and another (−70.4, −6.5) m, drifting at a
  roughly constant ~0.7 m·s⁻¹ with no sign of arrest. Whether that is wall
  tunnelling during pile collapse, absent rolling resistance, or both, is
  **not diagnosed** — it is reported as observed. It also runs at μ = 1.0, not
  the authored 0.65, because `WorldInfo.contactProperties` is the ODE-path
  declaration and Newton does not read it (the engine warns).

Consequently the "use ordinary rigid spheres instead of `GranularGroup`" fallback
is **not** a shortcut to a viable B2 either: it would grade a pile that does not
currently behave like granular media, and the world it would be built from
contains no robot.

**This is recorded as a capability finding, not as a task.** Inventing a graded
task around a subsystem that does not work is precisely the failure mode this
suite is built to avoid, and B2 stays registered at `BLOCKED` — visible, with
its evidence — rather than being quietly dropped or quietly built.

---

## 7. Honesty rules

The OmniBench and AgentBench rules carry over unchanged: report losses as
prominently as wins; deviations are results; every number carries its machine id;
never present unlike things as like-for-like; never "first". BuildBench adds:

1. **No LLM judging, anywhere.** Same rule as AgentBench §8.1.1.
2. **A declaration is not a result.** Nothing in `tasks.py` is evidence of
   anything until it has an oracle, a null and a graded run. The build status is
   printed beside every task in every rendering of the matrix.
3. **We are the author and a contestant** (AgentBench §8.2), and a capability
   matrix is *more* vulnerable to that conflict than a benchmark is, not less —
   because a task list is a claim about what matters, and we wrote the list.
   Rules 1, 3 and §5 are the mitigations; they are weaker than AgentBench's
   pre-registration, and that is stated rather than glossed.
4. **An engine defect discovered while building a task is published as a
   defect**, on the same day, whether or not the task survives. §6.2 is the
   first instance.
5. **A `BLOCKED` task is never silently deleted.** Deleting it destroys the
   record of the thing we could not do, which is the most useful row in a
   capability matrix.

---

## 8. Open questions, recorded rather than resolved

1. **Is a capability matrix publishable at all by a vendor?** AgentBench's
   answer to the author-and-contestant problem is procedural (pre-registration,
   correction window, reviewer cells, adopting a maintainer's better scaffold).
   BuildBench inherits none of that machinery yet, and a matrix is easier to
   bias than a benchmark because the bias lives in task *selection*. Unresolved.
   Provisional answer: not publishable until §5 is satisfied.
2. **What is the honest unit of "expressible"?** A capability that exists in an
   API and silently does nothing is not expressible in any useful sense — that
   is lane 4's whole thesis, and §6.2 is an instance of it in our own tree. The
   `CITED` / `MEASURED` split in §2 is the current answer and it is probably not
   sufficient.
3. **How many simulators?** Three are declared because three are reachable.
   A matrix of three, two of which we have adapters for, is a narrow claim and
   should be labelled as one.
