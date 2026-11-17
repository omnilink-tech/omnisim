# AgenticSimBench — the agent-task benchmark

*(formerly **AgentBench**. Same suite, same graders, renamed at v0.3 when the
comparator set expanded from 5 to 7 and the platform programme landed. The
public name is **AgenticSimBench**; `agentbench` remains the Python package
name and the directory, and is not worth churning.)*

**Suite id: `agenticsimbench/v0.3`.** Rows carrying the predecessor id
`agentbench/v0` are a **different measurement and must never be pooled with
these** — see the version notice below.

**Status:** v0.3 contract, **partially implemented**. Six of the 15 tasks
(`A1_husky_swarm_10`, `B1_overlap_audit`, `B2_subject_in_frame`,
`B3_measure_and_report`, `C1_parse_error_fix`, `C2_fall_through_floor`) have
graders, oracle/null coverage and a runner; the other nine remain
contract-only. Two of the seven primary comparators (**OmniSim**, **upstream
Webots**) have adapters; the other five are **declared and unbuilt** — they are
in the frozen design and named in [`sims.py`](sims.py) with their blockers, so
the gap between what this file claims and what the tree can run stays visible
rather than being quietly absent.

This file remains the thing implementations must match. If a number, a
threshold, a prompt or a grader assertion is written here, the code must use
*this* value; if the code needs a different one, change this file in the same
commit and say why.

## Version notice — v0.3 supersedes v0, and nothing may be pooled

v0.3 changes three frozen things at once, and **each one alone** would already
forbid pooling old rows with new:

| | `agentbench/v0` (superseded) | `agenticsimbench/v0.3` |
|---|---|---|
| model | `claude-fable-5` (inherited from the local CLI default, not pinned) | **`claude-opus-5`, pinned in code** (§4.4) |
| wall-clock budget | a single global 45 min; the per-task `timeout_s` was recorded but **never enforced** | per-task, hard-capped and **actually enforced** — at 15 min when v0.3 was frozen, **45 min since 2026-08-10** (§2.4.1 has the full history and the non-comparability rule) |
| runs per cell | n = 5 (n = 10 for A1) | **one**, since 2026-08-10 (§3.5) |

⚠️ **v0.3 is not internally uniform, and §2.4.1 / §3.5 say where the seams
are.** The ceiling moved twice inside this suite id and the repeat count moved
once, so a v0.3 row is comparable with another v0.3 row only when both carry
the same `protocol` block (§5). This is recorded as a **freeze amendment**
rather than a further suite bump because **no scored run existed** under
v0.3 at the time — all 14 v0.3 rows on record are single exploratory
cells, barred from claims by the very rule being amended. See
`preregister/FREEZE.md`.
| comparators | 2 (OmniSim, Webots) | 7 primary + 3 extended, declared in `sims.py` (§6.1) |

The predecessor grid — 35 scored cells, run 2026-08-01 — is **preserved, not
deleted**, under its own suite id. It is quotable only as what it is: a
different model, on a budget three times longer, against two simulators. It is
also **partly invalidated on its own terms**, which we record here rather than
in a footnote:

- **C2's 5/5 on both arms was scoring nothing.** The floor was authored at
  z = 0, so the engine's implicit z = 0 ground plane held the crate at a rest
  height inside C2.5's 0.15 m tolerance — the **unfixed** world passed 5/5
  (measured 2026-08-08, fixed in `187a9baab`, pinned by
  `test_c2_discriminates.py`). Those C2 rows measured the tolerance, not the
  agent.
- **Four frozen worlds silently drifted after the freeze.** `9db0e1628` and
  `187a9baab` (both 2026-08-08, seven days after the 2026-08-01 freeze) edited
  hash-frozen task worlds, and three doc commits edited the frozen plan §2,
  with no amendment recorded. `preregister/test_freeze.py` had been failing on
  that drift ever since. The freeze guard worked; nobody was reading it. §8.4
  is the rule that closes this.

Neither fact was discovered by the campaign. Both were found by re-reading the
guard output while re-baselining for v0.3, which is the argument for doing the
re-baseline rather than continuing the old grid.

**Relationship to the rest of the suite**

| suite | asks | system under test | verdict |
|---|---|---|---|
| `omnibench` lanes 1–2 | is the physics right, how fast | the engine | numeric error / throughput |
| `omnibench` lane 3c (`DRIVEABILITY.md`) | **does the endpoint exist** | the agent-facing API | 10 capability probes |
| **`agentbench` (this file)** | **did the agent get the job done** | the simulator *as an agent works it* | task completed / not |
| `omnilink_tasks` | does the OmniLink **agent product** operate a robot fleet honestly | the LLM engine, sim held fixed | pose + trace |

Driveability asks *"is there a `/scene/tree`?"*. AgentBench asks *"given one
sentence, did an LLM with no human help end up with a running 10-Husky scene?"*
The first is a property of the API. The second is a property of everything —
API, file format, docs, error messages, defaults, and how forgiving the engine
is when the agent guesses wrong.

---

## 0. The thesis, stated so it can lose

> **H1.** An LLM agent, given one sentence and no human in the loop, completes
> more of a fixed task set, in less wall-clock time and fewer tokens, on OmniSim
> than on upstream Webots, Gazebo, Isaac Sim, MuJoCo, CoppeliaSim or Genesis.
>
> **H2 (v0.3).** That advantage does not require workstation hardware: OmniSim
> reaches its Minimum Productive Platform (§10.8) at a lower tier of the P0–P6
> ladder than the comparators do.

H2 is the platform claim and it is falsifiable in an unusual direction — it can
lose by our *winning* everywhere, if every comparator also runs fine on P0, in
which case accessibility is simply not a differentiator and we should stop
saying it is.

H1 is falsifiable and we expect to lose parts of it. Specific ways it can lose,
written down *before* any run so we cannot retro-fit them into wins:

- **The model has read far more SDF, MJCF and USD than `.wbt`.** Gazebo and
  MuJoCo have a large training-data advantage on the authoring tier. AgentBench
  may show that advantage dominating our transport advantage.
- **Gazebo and Isaac implement ROS 2
  [`simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces)
  natively** — `SpawnEntity`, `DeleteEntity`, `SetEntityState`, `GetEntityBounds`,
  `StepSimulation`, `LoadWorld`. **OmniSim's harness has no spawn/delete/set-pose
  verb at all** (verified against `scripts/harness/omnisim_harness.py`: the routes
  are load / diagnostics / state / tree / node / viewpoint / visible / frame /
  orbit / look_at / render_stats / screenshot / step / reset / robots / joints /
  devices / contacts / grips / events / damage — and nothing that creates a node).
  On OmniSim, authoring **is** text editing plus reload. If per-entity spawn
  services beat file-and-reload, we lose the authoring tier and that is the
  finding.
- **Our own measured latencies are bad in places.** Driveability recorded
  `/sim/step` at **~0.7–0.9 s per 16 ms step**, a plain `/scene/tree` at ~1.4 s
  on a 37-node world (~50 s cold on a 149-node one), and a **broken-world load
  taking ~243 s to resolve**. Tasks C1/C2/C3 walk straight into that last one.
  AgentBench will make it a published number.

H1 is **not** "OmniSim is the best simulator", it is **not** a physics claim, and
it is never to be phrased as "the first" anything (see §8.1.6).

---

## 1. Layout and identifiers

```
tests/benchmarks/agentbench/
  SPEC.md                <- this file (the contract)
  sims.py                <- the comparator registry: 7 primary + 3 extended,
                            each implemented|declared with its blocker named
  tasks/                 <- one dir per task: prompt.txt, initial/, grader.py, meta.json
  graders/               <- sim-agnostic assertion library (physical units only)
  adapters/              <- per-sim ground-truth readers (omnisim/, gazebo/, isaac/, webots/, mujoco/)
  agent/                 <- the sim-agnostic agent loop, tool plumbing, trace writer
  controllers/
    agentbench_recorder/ <- grader-owned pose recorder (see §4.5.1)
  images/                <- pinned container definitions, one per simulator
  preregister/           <- frozen spec hashes + the pre-registration record (§8.3)
  run_all.py
  results/               <- JSONL rows, gitignored except committed campaign summaries
```

Suite id: **`agenticsimbench/v0.3`**. Task ids are stable and file-name safe
(`A1_husky_swarm_10`). Machine attribution, fingerprinting and the row writer are
**reused from `tests/benchmarks/omnibench/common/results.py`** — do not fork it.

### 1.1 Why this is a new suite and not an extension of `omnilink_tasks`

`omnilink_tasks` is not the same measurement and must not be absorbed:

- **Opposite variable.** `omnilink_tasks` holds the simulator fixed (four Huskies
  in one pre-authored world) and varies the *LLM engine* to grade the OmniLink
  product. AgentBench holds the agent fixed and varies the *simulator*.
- **Different artifact.** `omnilink_tasks` grades robot pose in a world someone
  else authored. AgentBench grades a world **the agent produced**, plus how it
  behaves when run standalone.
- **Different failure of interest.** Theirs is honesty and tool selection; ours
  is completion under a tool surface.
- It is also under heavy active development in a parallel lane (16 tasks across
  capability / tool_selection / delegation / honesty / safety). Colliding with it
  would be gratuitous.

**What AgentBench reuses from it, deliberately:** the five-outcome taxonomy, the
rule that *a grader may believe measured state and the recorded trace and nothing
else*, the invariant that **no task may be passable by doing nothing** (enforced
by a test, §7.1), "unmeasured cost is `null`, never `0.0`", and its cost /
provenance modules if and when they land. If they have not landed, re-implement
the rules — the rules are the load-bearing part.

---

## 2. The task set

**15 tasks: 12 cross-simulator "core" + 3 OmniSim-only "annex".**

The annex tasks probe properties that have no honest analogue elsewhere. They are
**never** included in any cross-simulator aggregate, and any chart that mixes them
with core tasks is a bug.

Every task ships: `prompt.txt` (the **exact** one sentence, byte-identical across
simulators except for a path), `initial/` (the starting files), `grader.py`
(programmatic, end-state assertions only, **no LLM judging anywhere in AgentBench**),
`par_s`, and the capability it stresses.

### 2.4 The budget, and the hard ceiling

**`timeout_s = min(3 × par_s, 2700 s)`.** **Max turns = 60.** **Max tokens =
400k in / 60k out.** Any budget exhausted → `FAIL` (not `INVALID`) — running
out of budget is a result.

**No cell may exceed 45 minutes of wall clock.** The ceiling
(`tasks.TASK_HARD_CEILING_S = 2700`) exists so campaign cost is bounded
*before* the campaign runs: `cells × 2700 s` is the worst case that can ever
be billed, whatever the agent does. It is enforced in three places so no
single edit can lift it:

1. `Task.timeout_s` clamps whatever `meta.json` declares — a task cannot buy
   itself a longer run.
2. `run_cell` gates the child process on `min(--timeout-s, task.timeout_s)`, so
   the CLI flag can only ever *tighten* a budget.
3. Each task's `meta.json` carries a `budget` block recording the rule, the
   resulting `ratio_to_par`, and whether the ceiling truncated it.

**A cell finishing well inside the ceiling is not a problem.** The ceiling is
a cap on the worst case, not a target and not a par time; `par_s` is the
difficulty estimate and it is reported beside every result.

At 2700 s the 3× rule holds for every task and **nothing is truncated**:

| task | par | timeout | ratio |
|---|---|---|---|
| B2, B3 | 240 s | 720 s | 3.00× |
| B1 | 300 s | 900 s | 3.00× |
| C1 | 480 s | 1440 s | 3.00× |
| C2, R1, R2, R3 | 600 s | 1800 s | 3.00× |
| A1 | 720 s | 2160 s | 3.00× |

`par_s` is deliberately **left at its measured difficulty estimate** rather
than being rewritten to make the ratios come out. A1's estimate stands even
though A1 scored 0/17 at a *45-minute* budget on the superseded v0 grid.

A run killed at the ceiling records `budget_exhausted: true` alongside its
`progress` ordinal (§3.2), so "ran out of time at level 3" stays
distinguishable from "finished and was wrong".

#### 2.4.1 The ceiling has moved twice, and cells scored under different ceilings may not be pooled

`TASK_HARD_CEILING_S` is **global** — one number rewrites every task's budget
at once — so it is also a property of the *experiment*, not of a task. Its
history, kept because a reader comparing two campaigns needs to know which
ceiling each ran under:

| value | when | why |
|---|---|---|
| 900 s (15 min) | freeze v3, 2026-08-09 | the original cost cap |
| 1800 s (30 min) | 2026-08-09, owner | measured: **every** robotics-tier cell exhausted 15 minutes with the goal unreached (R1: 2 of 2 on omnisim, `budget_exhausted`), so the suite was measuring the cap rather than the simulator — and a cap that truncates every cell on every arm cannot discriminate between arms |
| **2700 s (45 min)** | 2026-08-10, owner | **cost control**, as one half of a trade: the suite stopped running repeats (§3.5) in the same decision. Repeats were the dominant token cost, so a single longer run is far cheaper than five shorter ones — A1's worst case per (task, arm) falls from `10 × 1800 = 18000 s` of agent time to `1 × 2160 = 2160 s` |

**⚠️ Cells scored under different ceilings are different measurements and may
never be pooled, averaged, or placed in one column.** Every row records the
budget it ran under (`timeout_s`) and, from 2026-08-10, the whole protocol
(`protocol`, §5). **A row with no `protocol` block predates the single-run
protocol** — that absence is the machine-readable fence.

Note that this is *not* visible from `timeout_s` alone for four of the six
core tasks: B1 (900 s), B2/B3 (720 s) and C1 (1440 s) sit below every ceiling
the suite has used, so their per-task budget is genuinely unchanged and their
rows look identical across eras. Only **A1** (1800 → 2160 s) and **C2**
(900 → 1800 s) moved. The protocol block is what distinguishes the rest.

#### 2.4.2 The cell wall bound

`timeout_s` bounds the agent's **session**. The cost of a *cell* is larger:
workspace staging, an authenticated preflight, port/process sweeps, R1's
grade-time placement, the real engine grading launch, teardown — and, if the
CLI refuses with a usage limit, a backoff and a retry. Without a bound on the
whole cell, the deferral loop multiplied the budget: 12 retries × (30 min +
15 min backoff) is about **nine hours for one cell**, while this file claimed
`cells × ceiling` was the campaign's worst case. That claim was false, and a
webots R1 cell was measured running 58 minutes on 2026-08-09.

The bound is therefore **`task budget + a fixed allowance`**
(`run_cc_cell.cell_wall_bound_s`, allowance `CELL_WALL_ALLOWANCE_S = 900 s`).
It is **additive, not a multiple**: the allowance pays for work that does not
scale with the agent's budget. It was `budget × 2.0`, sized when a rate-limited
cell needed room for a second full-length try; carried onto a 45-minute
ceiling that would have licensed a **90-minute** cell.

900 s is sized from measurement, not taste: over the 90 committed cell reports
in `results/` (machine `9722d23d12a3`), the non-session part of a cell ran a
median of **28 s**, p90 **225 s**, max **838 s**. It also equals one
rate-limit backoff, so one deferral still leaves the retry its full budget
instead of silently shortening it — and when a session *is* shortened, the
row says so (`protocol.session_budgets_s`, `session_budget_curtailed`).

Resulting bounds: A1 3060 s, C2/R1/R2/R3 2700 s, C1 2340 s, B1 1800 s,
B2/B3 1620 s. `readiness.py` prints the bound for any task.

### 2.1 Core tasks

| id | tier | one-sentence prompt (verbatim) | initial state | stresses | par |
|---|---|---|---|---|---|
| **A1** `husky_swarm_10` ⭐ | author | *"Build me a scene with 10 Huskies in it and let them move randomly, and show me proof they actually moved."* | empty scratch dir; repo read-only | multi-robot authoring, controller wiring, solver choice, self-verification | 12 min |
| **A2** `warehouse_racks_12` | author | *"Make me a 30 by 18 metre warehouse with twelve pallet racks in three rows and a mobile robot parked at the loading dock."* | empty scratch dir | scene layout from prose, asset discovery, no-overlap placement | 10 min |
| **A3** `arm_on_table` | author | *"Put a robot arm on top of this table so it is standing on the surface, not floating and not sunk into it."* | world with a table, arm asset available | geometric bounds reasoning, settling under gravity | 8 min |
| **B1** `overlap_audit` | inspect | *"Are any two robots in this scene overlapping? Answer yes or no and name the pair."* | world with exactly one interpenetrating pair among 6 robots | bounds retrieval, pairwise reasoning, honest reporting | 5 min |
| **B2** `subject_in_frame` | inspect | *"Point the camera at the red cylinder and prove to me it is in frame."* | world with red + blue + green cylinders | camera control with numeric verification | 4 min |
| **B3** `measure_and_report` | inspect | *"How far apart are the two Huskies centre to centre, and which one is taller?"* | 2 Huskies, one on a 0.4 m plinth | reading real geometry rather than guessing | 4 min |
| **C1** `parse_error_fix` | debug | *"This world will not load. Fix it."* | `.wbt`/`.sdf`/`.usda`/`.xml` with an unbalanced brace **and** an undefined node type | structured diagnostics, error-message quality | 8 min |
| **C2** `fall_through_floor` | debug | *"The robot falls through the floor. Fix it and prove it stays up."* | floor authored with no collision surface | contact debugging, verification loop | 10 min |
| **C3** `gripper_drops_box` | debug | *"The gripper closes on the box but the box slips out. Fix it and prove it holds for ten seconds."* | grasp world with friction/force set too low | contact/friction tuning, timed verification | 12 min |
| **D1** `too_dark_fix` | iterate | *"This scene is too dark. Fix the lighting and prove it with numbers, not a screenshot I have to squint at."* | correct geometry, one weak light | quantitative render feedback loop | 6 min |
| **D2** `settle_the_stack` | iterate | *"These eight boxes explode the moment the world loads. Make the stack sit still for five seconds."* | box tower + a bad solver config | solver-parameter iteration with measurement | 12 min |
| **E1** `drive_to_wall` | multi-step | *"Drive the Husky to the far wall without touching anything on the way, then tell me how far it travelled."* | corridor world with obstacles | closed-loop control + contact monitoring + honest measurement | 10 min |

### 2.2 Annex tasks (OmniSim only — never in a cross-sim number)

| id | prompt | why annex | par |
|---|---|---|---|
| **N1** `skid_steer_jam` | *"This Husky's wheels spin but the robot never moves. Fix it."* | The failure is XPBD-specific: soft positional contact has no bounded lateral wheel slip, so a skid-steer pivot jams all four wheels. The fix is `newtonSolver "mujoco"` / `"mujoco_warp"`. There is no faithful analogue on a DART or PhysX stack. | 12 min |
| **N2** `prove_the_backend` | *"Prove to me which physics backend actually drove this run — don't guess from the config."* | Tests the `<log>.newton.json` verdict sidecar. No competitor ships an equivalent race-free attestation, so scoring them would be scoring an absence we invented the test for. | 5 min |
| **N3** `five_edit_loop` | *"Move the robot 2 m north, then 2 m east, then 2 m south, then 2 m west, then back to the middle — verify the pose after each move."* | Measures hot-reload-without-relaunch loop cost specifically. Gazebo's `LoadWorld` and Isaac's `SetEntityState` are real but structurally different, so the number would not be comparable. | 6 min |

### 2.3 ⭐ A1 — the flagship, in full

**Prompt (verbatim, the only text the agent gets beyond the standard system
prompt and its tool list):**

> Build me a scene with 10 Huskies in it and let them move randomly, and show me
> proof they actually moved.

**Initial state.** An empty scratch directory, writable. The repository mounted
read-only. No world is pre-loaded. No harness is pre-started — starting it is part
of the task.

**What already exists in the tree, and what does not** (checked, so the grader is
not accidentally scoring a copy-paste):

- `projects/samples/demos/worlds/physics/newton_husky_swarm_drive.wbt` has **8**
  Huskies, not 10, and the file's own comment explains why: *"at 10 articulated
  bodies on Newton 1.1 + XPBD we hit a structural scaling limit — one or more
  huskies' wheel actuator targets stop being satisfied."* They run
  `drive_forward` — straight lines, not random.
- `projects/robot_combat/worlds/tests/newton_husky_10.wbt` **does** have 10, also
  on `drive_forward`, also straight lines, and it is a combat test world.
- `projects/samples/demos/worlds/showcase/warehouse_husky.wbt` has **one** Husky
  on the `husky_random` controller, and needs `newtonSolver "mujoco_warp"`,
  `newtonStatics TRUE`, `newtonRobotColliders TRUE` and an **even**
  `newtonSubsteps` to move at all.
- `projects/default/controllers/husky_random/husky_random.py` random-walks, but
  only takes the useful branch when the robot is `supervisor TRUE`; it seeds its
  RNG from `hash(robot.getName())`, which is **not stable across processes**
  unless `PYTHONHASHSEED` is pinned.

So the ingredients exist and no single file is the answer. Copying the 8-Husky
world fails the count; copying the 10-Husky combat world fails the randomness
assertion; using `husky_random` without `supervisor TRUE` and without a
skid-steer-capable solver produces ten robots that sit still. Good task.

**Grader — exact assertions.** Two phases. Phase A runs against whatever the agent
built, live. Phase B re-runs the artifact **standalone**, cold, with no harness
supervisor, because a world that only works under the harness is not a world.

Constants, fixed here: **N_ROBOTS = 10**, **SETTLE_S = 1.0 s**,
**N = 30.0 s of simulated time**, **X = 2.0 m**, **PATH_MIN = 4.0 m**.

*Phase A — structure and placement (harness, `GET /scene/tree?bounds=1`, `GET /robots`, `GET /sim/contacts`):*

| # | assertion | pass condition |
|---|---|---|
| A1.1 | **exactly ten Huskies** | exactly 10 nodes with `type ∈ {URDFRobot, Robot}`, `num_joints ≥ 4`, and a URDF/proto reference whose basename is `husky.urdf` (read from the artifact text). The grader's recorder node is excluded by DEF. |
| A1.2 | **ten distinct robots** | 10 distinct non-empty `name` values, 10 distinct scene-tree node ids. Ten references to one node is a fail. |
| A1.3 | **none interpenetrating at t=0** | for every pair, the world-space AABBs from `?bounds=1` must **not** overlap on all three axes, with ≥ 0.05 m clearance on at least one axis; **and** `GET /sim/contacts` reports no contact whose two participants are both robots during the first 10 basic timesteps. |
| A1.4 | **all ten are controlled** | every one of the 10 has a non-empty `controller` field, and the engine log shows 10 controller processes started. |

*Phase B — motion and clean exit (standalone `run-headless`, grader-injected recorder):*

| # | assertion | pass condition |
|---|---|---|
| A1.5 | **all ten moved ≥ X** | after SETTLE_S, every robot's **net displacement** over the next **N = 30.0 s of sim time** is **≥ 2.0 m**. `min` over robots, not mean. |
| A1.6 | **they actually drove, not slid** | every robot's integrated **path length** over the same window is **≥ 4.0 m** (recorder integrates every basic timestep). |
| A1.7 | **nobody fell through or tipped over** | for every robot, `\|z(t_end) − z(t0)\| ≤ 0.30 m` and `z(t_end) > −0.10 m`, at every recorded sample. |
| A1.8 | **the motion is random, not a parade** | let θᵢ be the bearing of robot i's net displacement; the circular resultant `R = \|Σ e^{iθᵢ}\| / 10` must be **< 0.70**. Ten identical headings give R = 1.0; ten uniform random headings give E[R] ≈ 0.32, and P(R ≥ 0.70) ≈ 0.008 — so the false-fail rate is under 1 % and a `drive_forward` copy fails deterministically. |
| A1.9 | **the run exited cleanly** | the recorder calls `simulationQuit(0)` at t = SETTLE_S + N; the process exits **0** *and* the engine log contains **zero** `ERROR`-class lines *and* the log shows the world reached finalize. Exit code alone is not accepted — a failed load still exits 0 in 2–4 s. |
| A1.10 | **backend attribution recorded** | the row records which backend drove Phase B, from the `<OMNISIM_LOG_PATH>.newton.json` sidecar (`{backend, solver, degraded}`) when Newton, or the explicit `physicsBackend "ode"` pin when ODE. **Not a pass/fail gate** — either backend passes — but a run with no attribution is `INVALID`, because an unattributed physics result is not a result. |

All ten must hold. **PASS = 10/10.** Nine of ten is `FAIL`, recorded with the
`progress` ordinal from §3.2 and the failing assertion named.

**Deliberately not asserted:** which backend, which controller, whether the agent
wrote a new controller or reused `husky_random`, world size, spawn pattern, and
whether the "proof" the agent shows the user is a table, a log or a screenshot.
The prompt asks for proof because self-verification is the behaviour we want to
observe; the *grader* does its own measuring regardless, and the trace records
whether the agent verified before declaring success. That is reported as
`self_verified: true|false` and is **not** part of the pass criterion — it is the
most interesting secondary metric in the suite.

---

## 3. Scoring

### 3.1 What is recorded per run

| field | definition |
|---|---|
| `outcome` | one of six, §3.3 |
| `t_agent_s` | wall clock from *simulator ready* to the agent's final message |
| `t_total_s` | wall clock from container start (includes sim boot — Isaac's is minutes) |
| `turns` | assistant messages containing ≥ 1 tool call |
| `tool_calls` | total tool invocations |
| `tokens_in` / `tokens_out` / `tokens_cache_read` | reported separately, never summed into one "tokens" figure |
| `usd` | cost at the model's list price **on the run date**, pinned in the row. Unmeasured cost is `null`, never `0.0`. |
| `interventions` | structurally 0 — see §3.4 |
| `self_verified` | did the trace show the agent checking its own work before claiming success |
| `progress` | the ordinal in §3.2 — reported, never scored |

### 3.2 Half-done: defined, reported, never awarded

Completion is **binary**. There is no partial credit, and no aggregate anywhere
in AgentBench may be a weighted sum of progress states. But "how far did it get"
is the most useful diagnostic in the suite, so it is recorded as a strict ordinal:

```
0 no_artifact           nothing was written
1 artifact_invalid      written, does not parse / does not load
2 artifact_loads        loads, but the graded behaviour is absent
3 artifact_runs         loads and steps, some assertions pass
4 graded_pass           every assertion passes
```

Only level 4 scores. Levels 0–3 are all `FAIL` and are reported as a distribution
per task, because "OmniSim reached level 3 and Gazebo reached level 1" is a real
finding that a pass rate hides — and it is a finding we must publish even when the
arrow points the other way.

### 3.3 Outcomes

| outcome | meaning | in the pass rate? |
|---|---|---|
| `PASS` / `FAIL` | the agent's result, including budget exhaustion | **yes** |
| `NOT_EXPRESSIBLE` | the task has no honest formulation on this simulator (§6.4) | **no** — reported as its own count |
| `INVALID` | the stack broke: sim crash unrelated to the agent's edit, pod died, port collision, missing attribution | no |
| `SKIPPED` | no credential / no hardware for this cell | no |
| `ERROR` | the benchmark harness itself failed | no |

`INVALID` runs are re-run up to twice. A cell with > 20 % `INVALID` after re-runs
is published as unreliable rather than published as a score.

### 3.4 Human interventions must be structurally zero

There is no human channel. The container has no TTY the operator can reach, no
GUI, and the agent loop has no "ask the user" tool. So `interventions` is 0 by
construction, and anything that *would* have required a human — a modal dialog, a
manual port cleanup, an operator restarting a wedged sim — makes the run
`INVALID` with the cause named. We do not count interventions; we make them
impossible and void the run when they were needed anyway.

### 3.5 The measurement protocol: ONE run under a ceiling, and what that forbids

**The protocol is a single run per `(task, simulator, condition, model)`,
bounded by the §2.4 wall-clock ceiling.** Owner's decision, 2026-08-10:
*"we are not doing n times anymore. We will do it so that we set a maximum
time that the agent should finish before. If the agent finishes way before it
or a little bit before it, then it is not a problem."* The reason is cost:
repeats were the dominant token spend (Lane B at n = 5, A1 at n = 10 is 35
cells for one arm's core set), and the suite could not afford them.

It replaces the previous rule — `n = 5` per cell, `n = 10` for A1 — which is
recorded here rather than deleted, because rows produced under it exist.

**What a single run buys, and what it does not.** It buys an outcome under a
stated budget on a stated machine. It buys **no variance estimate whatsoever**.
This is not a matter of a wide interval: with one sample a pass fraction, a
`pass@1`, and a confidence interval are **undefined**, and the variance is
**unmeasured** — which is a different claim from *small*. Temperature 0 is not
determinism (§4.4.1), so the run-to-run spread is real and we have chosen not
to measure it.

The rules that follow are therefore binding, not stylistic:

1. **A single observation is never reported as a rate.** Not `pass@1`, not
   "0/1", not "100 %", not a bar in a chart whose axis is a percentage. The
   honest form is the outcome and its cost: *"passed once, in 148 s, under a
   2700 s ceiling, on machine `<id>`"*.
2. **No confidence interval, no error bar, no significance claim** is
   attached to a single-run cell — including a Wilson interval on 1/1 or 0/1,
   which is arithmetically expressible and epistemically empty.
3. **Every row carries its own sample count** (`protocol.samples`,
   `protocol.runs_per_cell`, `protocol.variance_measured: false`, §5). A
   consumer that aggregates reads those fields; one that finds
   `variance_measured: false` may not print an interval.
4. **Every published table states the protocol in its header** — one run per
   cell, the ceiling in force, and *"variance is unmeasured"* — in the header,
   not a footnote.
5. **Counts across tasks are still legitimate and are the primary reading**:
   *"OmniSim completed 4 of 6 core tasks, Webots 5 of 6, one run each"* is a
   fair sentence. It is a count of tasks, not a rate over repeats. The
   `progress` histogram (§3.2) does the same work and is more informative.
6. **A difference of one task is not a result.** With one run per cell the
   suite cannot separate a real difference from a single coin flip in either
   direction, and no comparative claim may rest on a margin this design cannot
   resolve. Anything that would be quoted goes to §9.3's re-run clause first.

Other aggregate rules, unchanged:

- **`solved_at_least_once`** is now identical to the outcome and is not
  reported as a separate statistic.
- **`t_agent_s` per PASS** — *and* the time of every failing run alongside it,
  so a fast simulator cannot look fast by failing quickly. At one run per cell
  these are individual measurements, never medians.
- **Tokens and USD per PASS**, and per attempt.
- **`progress` histogram** per (task, sim).
- No cross-tier averaging into a single "AgentBench score". There is no single
  number. If someone asks for one, the answer is the per-task table.

**⚠️ The frozen withdrawal rule F is unevaluable under this protocol.** F
(plan §2.4, quoted verbatim in `preregister/FREEZE.md`) is defined on `n = 5`:
its completion channel needs `Δpass ≥ +2` out of 5, and its cost channel is
defined only on tasks where both conditions passed **≥ 3 of 5** runs. At one
run per cell `Δpass ∈ {−1, 0, +1}` and the definedness floor is unreachable,
so **both channels are structurally unevaluable and F would withdraw every
time** — which is a property of the arithmetic, never evidence about the
claim. The plan's §2 is frozen content and is **not** amended here: until its
owner re-derives F for a single-run design, no F verdict may be computed,
quoted, or read as a withdrawal.

---

## 4. The agent harness

### 4.1 The core design decision: two conditions, one identical baseline

Every simulator is run in **two conditions**:

- **`shell`** — the agent gets a POSIX shell, `read_file`, `write_file`,
  `list_dir`, and nothing else. This tool set is **byte-identical for every
  simulator.** The simulator is installed in the image; the agent figures the
  rest out.
- **`shell+tools`** — the same, plus that simulator's own first-party agent-facing
  surface exposed as native tool definitions (§6.1).

This is the design that defends the whole benchmark. The most damaging attack on a
vendor-run comparison is *"you gave our simulator a worse interface"*. Under
`shell`, we provably did not: the interface is a shell, and it is the same shell.
The `shell+tools` − `shell` delta then **measures** the value of each project's
agent surface instead of asserting it — including the possibility that OmniSim's
delta is small, which would be a real and publishable result about our own harness.

### 4.1.1 When a simulator ships no packaged tool surface

The design above quietly assumes every simulator *has* a first-party surface to
expose, and at least one does not: upstream Webots's §6.1 row — "extern
controllers on TCP:1234, `--stream`, the Supervisor API" — is a function
reference, not a packaged toolset, and upstream publishes no service definitions
to generate one from. For any such simulator, the `shell+tools` condition is a
**tool bridge wrapping that simulator's entire published API function
reference**, authored by the benchmark operator — that is, we write our
competitor's tools, and every incentive in §6.2 cuts against us doubled: a
subtly lame bridge manufactures the completion win, a subtly chatty one
manufactures the cost win. The bridge is therefore bound by the full §6.2
machinery — published before any number (§6.2.3), the 30-day correction window,
reviewer-run cells (§6.2.4), a maintainer's better bridge superseding ours as
the published condition (§6.2.5) — and additionally by the **oracle-based
granularity and distinctness guards** and the **no-deletion consequence** for a
non-distinct bridge (degeneracy downgrades the delta to descriptive; it never
removes a comparison), as defined in the validation programme:
[`docs/developer/agent-edge-validation-plan.md`](../../../docs/developer/agent-edge-validation-plan.md)
§2.2. Any exclusion from the function reference is listed and justified in the
pre-registration and countersigned by the §6.2.4 reviewer — a bridge faithful in
every included verb but curated by omission is the abuse the fidelity rules
alone cannot catch.

### 4.2 Tool plumbing, and an integrity property worth having

The agent loop (`agent/loop.py`) is sim-agnostic: system prompt → tools → messages
→ tool results → repeat, until a final message, a budget, or the timeout.

For OmniSim, the `shell+tools` tool definitions are **generated from the shipped
first-party MCP server's own registry** (`packages/omnisim-mcp` `TOOLS`, 18 tools:
`harness_status`, `load_world`, `get_scene_tree`, `get_scene_node`,
`get_viewpoint`, `frame`, `orbit`, `visible`, `look_at`, `screenshot`,
`render_stats`, `sim_step`, `sim_reset`, `get_events`, `list_robots`,
`get_robot_joints`, `get_contacts`, `get_diagnostics`). A test asserts the
benchmark's tool names, descriptions and schemas are **equal** to the shipped ones.
We therefore cannot quietly hand-tune our own tool descriptions for the benchmark:
whatever helps the benchmark agent is what ships to users, and vice versa.

The same rule binds every competitor: their tool definitions are generated from
*their* published service/API definitions, not paraphrased by us.

### 4.3 Isolation between runs

One container per `(task, simulator, condition, repeat)`. Nothing is reused.

- Fresh container from a **pinned image digest**; fresh checkout at a pinned repo
  sha; repo mounted **read-only** except a per-run scratch dir. A task cannot leave
  an artifact that helps the next task.
- Per-run unique ports. For OmniSim: harness `--port P`, `--supervisor-port P+1`
  (the defaults 6789/6790 collide — the harness self-detects, but a benchmark must
  not rely on that), and a **per-run `OMNISIM_LOG_PATH`**, or parallel runs clobber
  the shared `omnisim_log.txt`.
- Teardown reaps every simulator process, asserts the ports are free, and fails the
  run `INVALID` if anything survives.
- **No network.** Otherwise the agent's `pip install` or doc-fetch makes runs
  irreproducible and the result depends on the internet on the day.
- Documentation is a **read-only mounted corpus** per simulator, snapshotted and
  hashed — see the asymmetry discussion in §6.3.

### 4.4 The model, pinned

**The benchmark model is `claude-opus-5`, driven headless through the Claude
Code CLI.** It is pinned as `cc_lane/run_cc_cell.DEFAULT_MODEL` and applies to
**every** benchmark in this suite; `--model` overrides it only for deliberate,
recorded experiments.

It is pinned *in code* rather than inherited from whatever the local CLI
defaults to, because that inheritance is precisely how the superseded v0 grid
came to be scored on a different model than anyone intended: `run_cell` read
`preflight["default_model"]` and pinned that. Rows now record the pinned id,
the CLI's own default, and which of the two won (`model_pin_source`), so a
machine whose CLI defaults elsewhere is visible in the row instead of silently
changing the experiment.

**This is not a hypothetical risk — it was live on the benchmark machine.** The
first v0.3 cell (C1 / webots, 2026-08-09, exploratory) recorded
`cli_default_model: "claude-opus-4-8[1m]"` against
`pinned_model: "claude-opus-5"`, `model_pin_source: "DEFAULT_MODEL"`. Under the
old code that cell would have run on **Opus 4.8** and been filed as an Opus 5
result, with nothing in the row to reveal it. The three-field record is what
makes that detectable rather than merely fixed: if a future row shows
`model_pin_source: "DEFAULT_MODEL"` and a `cli_default_model` that is not the
pinned id, the pin is doing real work on that machine and the campaign is still
sound.

Because the CLI and the model behind an alias both move, replication is scoped
**"at this CLI version and this model id"** (§9, open question 5), and both are
recorded per row. Changing the model changes the measurement: it forces a suite version
bump, never a re-run inside the same suite id (see the version notice).

### 4.4.1 Agent nondeterminism

- Temperature pinned (0 where the provider allows) — and the report states plainly
  that **temperature 0 is not determinism**; identical prompts still diverge.
- **One run per cell (§3.5), so agent nondeterminism is present and
  UNMEASURED.** It used to say "`n` repeats per cell; variance is reported,
  never averaged away" — under the single-run protocol there is no variance to
  report, and the honest statement is that the spread exists, is not bounded by
  anything we measured, and is disclosed in every report header. This is the
  largest cost of the protocol change and it is stated where it can be seen,
  not buried in a caveat list.
- The prompt is fixed verbatim and identical across simulators. **No per-simulator
  prompt tuning, ever** — that is the mechanism by which vendor benchmarks cheat,
  and it is banned outright rather than discouraged.
- The system prompt is one shared file; the only per-sim text is a factual
  "here is your tool surface" block generated from §4.2.
- Same model, same scaffold sha, same budgets for every cell of a comparison. A
  comparison that mixes models is not published as a comparison.

### 4.5 Trace

Every run writes an append-only `trace.jsonl` — every message, every tool call with
full arguments, every tool result (truncated only in the transcript view, never in
the file), timestamps, token counts — plus the simulator log, the `.newton.json`
sidecar when present, every artifact the agent wrote, the grader's full assertion
record, and a `manifest.json` of image digest / repo sha / sim version / machine
fingerprint. All of it is archived per run and **published for every scored run**.
A number whose trace is not published is not a published number.

### 4.5.1 The grader-owned recorder

Phase-B motion measurement does **not** poll the harness. The harness free-runs
between RPCs, `/scene/tree` costs seconds, and `/sim/reset` does not restore node
state — all measured in driveability. Instead the grader copies the agent's world,
appends its own `Robot { controller "agentbench_recorder" supervisor TRUE }`, and
runs it headless. The recorder samples every robot's pose **every basic timestep**
in sim time, integrates path length, writes `%.17g` CSV, and calls
`simulationQuit(0)` at the target sim time. This is the
`omnibench/lane3/controllers/lane3_recorder` pattern — reuse it, with one change:
enumerate robots by **node type + name**, not by DEF, because the agent has no
reason to add DEFs and a grader that requires them is grading our conventions
instead of the task.

---

## 5. Results schema

JSON Lines, one row per `(task, sim, condition, model, repeat)` — and under
the single-run protocol (§3.5) there is exactly **one row per (task, sim,
condition, model)**, with `repeat` always `0`:

```json
{"suite": "agenticsimbench/v0.3",
 "task": "A1_husky_swarm_10", "tier": "author",
 "sim": "omnisim", "condition": "shell+tools", "repeat": 0,
 "protocol": {"id": "single-run-under-ceiling/2026-08-10",
              "runs_per_cell": 1, "samples": 1,
              "variance_measured": false, "is_rate": false,
              "hard_ceiling_s": 2700, "task_budget_s": 2160,
              "cell_wall_bound_s": 3060,
              "session_budgets_s": [2160.0],
              "session_budget_curtailed": false,
              "note": "ONE observation ... an OUTCOME, not a rate ..."},
 "agent": {"model": "...", "temperature": 0, "scaffold_sha": "...", "system_prompt_sha": "..."},
 "outcome": "PASS", "progress": 4, "self_verified": true,
 "assertions": {"A1.1": true, "A1.2": true, "A1.8": true, "...": true},
 "failed_assertion": null,
 "metrics": {"t_agent_s": 0.0, "t_total_s": 0.0, "turns": 0, "tool_calls": 0,
             "tokens_in": 0, "tokens_out": 0, "tokens_cache_read": 0, "usd": null},
 "par_s": 720, "timeout_s": 2160, "interventions": 0,
 "sim_version": {"name": "omnisim", "sha": "...", "binary_sha256": "...",
                 "backend": "newton", "solver": "mujoco_warp", "degraded": false},
 "image_digest": "sha256:...",
 "artifacts": {"trace": "...", "world": "...", "engine_log": "...", "recorder_csv": "..."},
 "deviations": ["..."],
 "machine": {"...": "..."},
 "utc": "2026-07-26T00:00:00Z"}
```

`sim` ∈ `omnisim`, `gazebo`, `isaac`, `webots`, `mujoco`.
`condition` ∈ `shell`, `shell+tools`.
`machine` is the trimmed `env_fingerprint.py` block via `omnibench/common/results.py`.

**The `protocol` block is mandatory on every new row and is the fence between
experiments.** It carries the sample count explicitly so that no consumer has
to infer it, and states in the row's own words that a single observation is
not a rate. Two rules follow:

- A row whose `protocol.id` differs from another's ran under a different
  ceiling and/or repeat count. They are different experiments. **Do not pool.**
- **A row with no `protocol` key at all was not produced by the campaign lane
  under this protocol** and states nothing about the budget or repeat count it
  ran under. Every row in `results_published/` as of 2026-08-10 is of this
  kind: **87 rows, none carrying a `protocol` block** — 73 under the
  superseded `agentbench/v0` id, and **14 under `agenticsimbench/v0.3`** (the
  11 `v03_pilot_*` cells plus 3 R1 cells, all single exploratory runs at the
  900 s and 1800 s ceilings; R1's own `meta.json` carries `status: "NOT READY
  FOR A PUBLISHED NUMBER"`). They are quotable only as what they are, and
  never in the same column as a row that carries the block.

---

## 6. The competitor baseline

This is the section that decides whether AgentBench is a benchmark or marketing.

### 6.1 Each simulator gets its best available surface

Surfaces are taken from each project's **own** documentation and API definitions.
Nobody is routed through an inferior path to make a point.

The registry is [`sims.py`](sims.py) — code, not prose, so the runners, the
campaign driver and the report cannot disagree about who is in the comparison.
`status` there is either **implemented** (an adapter exists, the oracle/null
gate is green, cells can be scored) or **declared** (in the frozen design, no
adapter yet, blocker named). Only implemented arms may appear in a published
comparison; declared ones are named in every coverage table so the distance
between the design and the tree is never invisible.

**Primary leaderboard (7).** The public headline comparison is these and only
these.

| sim | version | status | `shell+tools` surface |
|---|---|---|---|
| **OmniSim** | pinned repo sha | **implemented** | 18 MCP tools from `packages/omnisim-mcp`, generated from the shipped registry |
| **Webots (upstream)** | R2025a | **implemented** | operator-authored bridge over the published Supervisor / controller API reference (§4.1.1) |
| **MuJoCo** | pinned | declared | the `mujoco` Python API and viewer as tools |
| **Gazebo** | **Jetty 10.0.0 LTS** (LTS to May 2031) | declared | ROS 2 `simulation_interfaces` via `ros_gz/src/gz_simulation_interfaces` — `SpawnEntity`, `DeleteEntity`, `GetEntities`, `Get/SetEntityState`, `GetEntityBounds`, `StepSimulation`, `Reset/Get/SetSimulationState`, `LoadWorld`, `GetSimulatorFeatures`, `GetSpawnables`, `GetNamedPoses` — plus the `gz` CLI and `ros2 topic/service` |
| **Isaac Sim** | 6.0 | declared | `isaacsim.ros2.sim_control` (19 services + 1 action) **plus** the Kit Python API as a tool, because that is how Isaac users actually script it |
| **CoppeliaSim** | pinned | declared | the ZeroMQ remote API / integrated scripting and the headless CLI path. **Pin one physics backend per release and disclose it** — it ships several, and an undisclosed switch makes the row incomparable with itself |
| **Genesis** | 1.2.1 | declared | the Pythonic simulation interface (CPU and GPU backends) |

In every condition the `shell` arm is the same POSIX shell + `read_file` /
`write_file` / `list_dir`, byte-identical across all seven (§4.1).

**Extended research set (3, published as a second table).** Never merged into
the primary leaderboard, because these are not the same product category — a
physics engine and a full robotics simulator answer different questions and
averaging them produces a number about neither.

| sim | status | why tracked |
|---|---|---|
| **PyBullet** | declared | historically important lightweight Python baseline; the natural low-resource arm |
| **SAPIEN** | declared | embodied-AI / articulated-object manipulation and RL |
| **Newton** | declared | GPU physics on Warp. ⚠️ **Reported as its own row and never conflated with OmniSim's**, because Newton is also OmniSim's only physics backend — a reader who mistakes one row for the other would draw exactly the wrong conclusion |

Three of these matter more than they look:

- **Upstream Webots is the control experiment.** Same file format, same base
  engine, no OmniSim harness. Any OmniSim − Webots delta is *exactly* the surface
  we added, with physics and format held constant. It is the cheapest and most
  informative baseline in the suite and it runs on a laptop. **If OmniSim does not
  clearly beat upstream Webots, the entire agent-native claim is dead**, and that
  result must be published as prominently as a win.
- **Raw MuJoCo is the strongest adversary on the authoring tier**, and it is nearly
  free to run: MJCF is compact, well-represented in training data, and an LLM
  writing `<body>` blocks plus a 20-line Python driver may simply beat a `.wbt`
  plus a harness. We include it *because* it may beat us.
- **Gazebo is the strongest adversary on the *authoring-verb* tier**, for a
  reason we should state before measuring rather than after: it has
  `SpawnEntity` / `DeleteEntity` / `SetEntityState` / `GetEntityBounds` as
  first-class services, and **OmniSim's harness has no spawn-or-delete verb at
  all** — on OmniSim, authoring *is* text editing plus reload. If per-entity
  services beat file-and-reload, we lose that tier, and the finding is that we
  should build the verb.

### 6.2 Who writes the competitor scaffolding, and how we avoid crippling it

We write it first — there is no one else — and then we do all of the following
before publishing a single competitor number:

1. **Pre-register** the task set, prompts, graders, par times, thresholds, scoring
   model, and this fairness plan. Freeze the file, publish its SHA-256 in
   `preregister/`, and timestamp it publicly. Only *then* run competitors. Any
   post-hoc change forces a suite version bump and re-runs everything.
2. **The `shell` condition is the fairness floor** (§4.1). Its tool set is
   byte-identical across simulators, so the "you crippled us" attack has to be made
   against a POSIX shell.
3. **Publish the competitor scaffolding before the numbers**, in a public,
   PR-able repo, with a **30-day correction window** and an explicit invitation to
   the Gazebo, Isaac, Webots and MuJoCo communities. Accepted corrections re-run the
   affected cells and the revision history stays public.
4. **A non-OmniSim reviewer runs each competitor cell once** — ideally a regular
   user of that simulator — and their notes are published verbatim, criticisms
   included, before the headline.
5. **If a maintainer produces a better scaffold, their number becomes the
   published headline for that simulator.** Committed in writing, here.
6. **Sim-agnostic graders, per-sim adapters.** Every assertion is stated in
   physical units (metres, seconds, contacts, exit codes). A grader that reads an
   OmniSim-shaped JSON response is a bug. Adapters are published with the
   scaffolding.
7. **Budget parity**: same model, temperature, turn cap, token cap, timeout,
   and the same number of runs per cell — **one** (§3.5), for every simulator
   including ours. Isaac's minutes-long boot is excluded from `t_agent_s` for
   everyone and included in `t_total_s` for everyone; both are published.
8. **Reproduce or drop, for anything published as a comparison.** No
   competitor number is published as a headline unless the cell reproduces on
   two independent runs. **This survives the single-run protocol and is not
   softened by it**, and the two are not in conflict: one run per cell is how
   the *campaign* is scored under a cost bound (§3.5), and this is an extra
   gate that a *published comparative claim* must clear on top. Deleting it
   because the routine protocol got cheaper would be trading away a fairness
   commitment we made publicly, to buy tokens. In practice it means a
   comparative headline costs its cells twice, and that cost is budgeted as
   part of publishing rather than as part of measuring — the same discipline
   as §9.3's re-run clause, which it now shares a mechanism with.
9. **Version and hardware pinning**, published: image digests, sim versions, ROS
   distro, GPU, driver, CPU, machine id.

### 6.3 The asymmetry we cannot design away: documentation

Our agent reads `AGENTS.md`, which is an unusually good agent-facing manual and
which literally hands over the non-obvious answers — `newtonSolver "mujoco"` for
skid-steer, the canonical lighting recipe, the `.newton.json` sidecar, the harness
loop. That is a genuine product advantage, and it is also a confound.

We do **not** solve this by withholding docs (unrealistic, and it would make the
competitor cells unrealistically hard too). We solve it by measuring it:

- Every simulator gets a **hashed, published, read-only docs corpus**: its own
  official docs snapshot plus its repo. Contents listed in the report.
- **Ablation cell: OmniSim with `AGENTS.md` and `docs/developer/` removed**, run at
  full `n` on the core set. If OmniSim's advantage collapses without them, the
  honest headline is *"our documentation is the moat"* — still true, still useful,
  and materially different from *"our API is the moat"*. We publish whichever one
  the data supports.
- The bias in the **other** direction is stated with equal prominence: the model
  has seen vastly more SDF, MJCF, USD and ROS 2 than `.wbt`, and `.wbt` worlds in
  training data are Webots' rather than OmniSim's. We do not know the sign of the
  net documentation-plus-priors effect until we measure it.

### 6.4 "Not expressible" is a finding, never a failure

A task is `NOT_EXPRESSIBLE` on a simulator when it has no honest formulation there
— not when it is merely hard, and not when we could not be bothered.

Requirements for the label: a written justification citing that simulator's **own**
docs or API listing showing the required verb or property is absent; review during
the 30-day correction window; and a standing rule that **a single credible
counter-example flips the label and the cell gets run.**

`NOT_EXPRESSIBLE` cells are excluded from every aggregate and reported as their own
count. They may never be rendered in the same colour as a failure, in any chart.

Expected, subject to review: annex N1–N3 on all competitors (that is why they are
annex). `D1 too_dark_fix` requires a quantitative render-statistics read-back; if a
simulator has no programmatic exposure metric, the honest handling is
`NOT_EXPRESSIBLE`, **not** a fail — and we should expect to find that at least one
competitor exposes something we did not know about.

---

## 7. Phasing

### Phase 0 — the graders, with no LLM in the loop (free, days)

#### 7.1 The oracle/null gate

Two scripted agents run every task: an **oracle** that performs the known-good
solution, and a **null** agent that does nothing. Required outcomes: oracle PASSes
every task; null agent PASSes none — **no task may be passable by doing
nothing**. This is `omnilink_tasks`' rule 6 made structural, and it must be
green before a single token is spent.

Also in Phase 0: a **fake-simulator dry run** so the harness, isolation, trace
writer and results schema can be exercised without a sim, a GPU or a network.

### Phase 1 — OmniSim only, local (~$200–600 in tokens, ~2 machine-days)

All 15 tasks × 2 conditions × **one run per cell** (§3.5) on the local
machines, both attributed by machine id per the AGENTS.md rule. Plus the §6.3
documentation ablation. Compute is free; the cost is tokens.

**What Phase 1 licenses us to say:**
> "On OmniSim, on machine `<id>`, a `<model>` agent given one sentence and no
> human help completed *k* of 15 tasks, **one attempt each**, under a 45-minute
> per-cell ceiling — *t* seconds and *T* tokens per success. Variance is
> unmeasured: each cell was run once. Here is the per-task table, the progress
> histogram, and every trace."

Note what left that sentence with the repeats: there is no `pass@1`, no
interval, and no *p*. *k* of 15 is a count of tasks and is fair; a percentage
over one attempt each is not.

**What we may NOT say after Phase 1 — these exact sentences are forbidden:**
- ❌ "faster than Gazebo / Isaac / any other simulator"
- ❌ "gets further than other simulators"
- ❌ "the most agent-driveable simulator"
- ❌ "the only simulator an agent can drive end to end"
- ❌ anything with "first"
- ❌ any sentence in which the word "than" is followed by a competitor's name

Phase 1 measures **us**, not a comparison. A comparative adjective before Phase 2
is a false claim, regardless of how confident the numbers look.

### Phase 2 — CPU competitors: Gazebo, Webots, MuJoCo (~$600–1800 tokens; 2–4 engineer-weeks)

Docker on Linux/WSL2; no GPU required. Dollar cost is tokens; the real cost is the
scaffolding — `ros_gz` bring-up, a `simulation_interfaces` client, per-sim graders
and adapters, three docs corpora — plus the §6.2 review and correction window.

**Licenses:** comparative claims against **Gazebo Jetty, Webots R2025a and MuJoCo,
on the 12 core tasks, at the pinned versions, on the stated machine, with the
`shell` / `shell+tools` split reported separately.** Isaac is absent and must be
named as absent in every statement. The Webots control result (§6.1) is published
whichever way it goes.

### Phase 3 — Isaac Sim on a pod (~$30–60 compute; 2–3 engineer-weeks)

Isaac's verified floor is **RTX 4080 / 16 GB VRAM minimum**, and **GPUs without RT
cores (A100, H100) are not supported** — so the 6 GB laptop 3060 is out and this
must run on a **RunPod 4090**. Compute is cheap (order tens of dollars at ~$0.7/hr
for ~40 pod-hours including image pull, bring-up, 12 × 5 runs and re-runs); the
engineer time is not.

Pod discipline is not optional and is not ours to improvise: arm the delete
watchdog **before anything else**, batch and detach rather than sitting on an idle
billed pod, write results to the network volume because a pod can vanish mid-run,
**TERMINATE** rather than stop, and confirm `GET /v1/pods` returns `[]` at the end.

**Licenses:** the full H1 statement, bounded to the tested task set, models,
versions, machines and conditions — and to the caveat list in §6.3 stated in the
same paragraph, not in a footnote.

---

## 8. Honesty rules

The OmniBench rules carry over unchanged (report losses as prominently as wins;
deviations are results; every number carries its machine id; never present unlike
things as like-for-like). AgentBench adds these.

### 8.1 Benchmark-specific

1. **No LLM judging, anywhere.** Every verdict is a programmatic assertion on
   measured end state. If a task cannot be graded programmatically, it does not
   enter the suite.
2. **Ground truth, never narration.** An agent that *says* the Huskies moved scores
   `FAIL` if the recorder says they did not — mirroring the failure `omnilink_tasks`
   was built around, where an agent reported completed moves with confident
   coordinates while all four robots had travelled 0.000 m.
3. **One run is an outcome, never a rate.** This rule used to read *"`n = 1`
   is a demo, not a measurement; marked `exploratory`, barred from claims"* —
   and the protocol it assumed is gone (§3.5, owner 2026-08-10: repeats cost
   more tokens than the suite has). Deleting the rule outright would have been
   the dishonest move, because the epistemics it encoded did not change when
   the budget did: **a single observation still estimates no variance.** So it
   is replaced rather than removed, and it now binds the *reporting* instead
   of the *running*:
   - a single-run cell may be published, as an **outcome with its cost and its
     ceiling**, never as `pass@1`, a percentage, or a point with an error bar;
   - every table says "one run per cell, variance unmeasured" in its header;
   - **no comparative claim may rest on a margin this design cannot resolve**
     (§3.5 rule 6) — a one-task difference between two simulators is within
     the noise of a single attempt, and goes to §9.3's re-run clause before it
     is quoted;
   - a cell that *is* re-run for variance says so on its rows
     (`protocol.runs_per_cell > 1`), so a deliberate experiment and the routine
     protocol can never be confused after the fact.
4. **Publish the traces or do not publish the number.**
5. **Failures of our own surface are first-class results.** The 243 s broken-world
   path, the 0.7–0.9 s `/sim/step`, the seconds-long `/scene/tree`, the missing
   spawn verb — AgentBench will price all four, and those prices get published
   whether or not we have fixed them by then.
6. **Never "first."** Per the standing rule, the positioning word is
   *agent-native*, and even that only after Phase 2.

### 8.2 We are the author and a contestant

This is exactly the conflict that discredited **SimBenchmark**, which was run by
RaiSim's own developers, and it is the single largest credibility risk in the
design. Naming it is not a mitigation. These are the commitments:

1. **Pre-registration with a published hash** before any competitor runs (§6.2.1).
   The task set cannot be reshaped once the results start arriving.
2. **We publish the whole result, including our losses — and we say so here, in
   the frozen spec, before we know the answer**, so that quietly abandoning the
   benchmark is itself visible evidence.
3. **Everything is re-runnable by a third party**: pinned image digests, published
   scaffolding, published prompts and graders. The Phase 1–2 cells run on a laptop
   with no paid hardware, so an independent check costs a critic tokens and an
   afternoon.
4. **Adversarial review before publication** (§6.2.3–6.2.5), including the standing
   commitment to adopt a maintainer's better scaffold as the published headline.
5. **The graders are written from the physical specification before any scaffolding
   is written**, so that no grader can be shaped around what our stack happens to
   make easy.
6. **The `shell` baseline is published alongside every `shell+tools` number.** If
   our advantage exists only when we hand the agent our own tools, that is visible
   in every table.
7. **A hostile summary is required in the report**: a section, written by us,
   stating the strongest honest case that AgentBench is unfair to competitors —
   kept up to date with the best criticisms received.
8. **External tasks** (§8.3) — the deepest mitigation, because the others all
   assume the task set itself is fair.

### 8.3 Task-selection bias — the risk none of the above fixes

We chose the tasks. Pre-registration stops us reshaping them *after* seeing
results, but it cannot make a self-authored task set representative, and a
comparison decided by task selection is worthless no matter how clean the
scaffolding is.

Commitments:

- **A second, externally sourced task set.** Tasks derived from sources we do not
  control — real user requests, issues on competitor repos, and **two tasks
  solicited from each competitor community as representative of their users' work**
  — scored and published as a **separate column**, never merged into ours.
- **If the external column disagrees with ours, the external column leads the
  report.**
- The 10-Husky flagship is explicitly labelled what it is: the **owner's demo
  task**, chosen because it is the thing we want to show, and reported separately
  from the aggregate so it cannot silently inflate it.

### 8.4 A freeze nobody reads is not a freeze

The pre-registration hashes 68 files and `preregister/test_freeze.py`
recomputes every one. Between 2026-08-01 and 2026-08-08 five of them changed —
four task worlds and the plan's §2 — through commits doing unrelated and
individually correct work (retiring `physicsBackend "ode"` pins after ODE was
deleted; repairing C2's geometry). **The guard caught all of it and failed on
every run. The failure was simply not acted on**, and the campaign kept its
"frozen" description for a week while its frozen content moved underneath it.

The mechanism was sound; the process around it was not. So:

1. **The freeze test is a release gate, not a test.** A red
   `test_freeze.py` blocks scoring a cell, publishing a number, or quoting a
   grid — in that state the suite has no valid pre-registration and anything it
   produces is `INVALID` by construction.
2. **Frozen paths are amended, never edited.** Any change to a hashed file —
   even an obviously-correct repair, even one driven by an engine change
   elsewhere — lands as a numbered amendment in `FREEZE.md` recording *what*
   changed, *why*, and **how many scored runs existed at amendment time**. That
   last number is what decides whether the change is a legal pre-run edit or a
   suite version bump.
3. **A benchmark artifact is not collateral damage.** A tree-wide sweep
   (retiring a field, renaming a node, reformatting worlds) must treat
   `tests/benchmarks/**` as opt-in, not swept-by-default. A frozen world is an
   *instrument*; correcting it mid-campaign is like recalibrating a scale
   halfway through weighing.
4. **Drift is disclosed even when it changes nothing.** If a re-hash shows a
   frozen file moved, that fact is published with the affected grid whether or
   not we believe it altered the result — "we checked and it did not matter" is
   a claim readers are entitled to check for themselves.

This section exists because the failure was ours and it was invisible for a
week. Recording it is cheaper than the alternative, which is discovering it
from someone else's re-run.

---

## 9. Open questions, recorded rather than resolved

1. **Is `t_agent_s` the right clock?** Excluding sim boot flatters Isaac and
   penalises nothing; including it flatters us. We publish both and have not
   decided which leads. Fix before pre-registration.
2. **Can A1's randomness assertion (A1.8) be gamed** by ten straight-line robots
   authored on ten different headings? Yes — R would be low. That is arguably a
   legitimate reading of "move randomly", so A1.6's path-length floor and the
   `self_verified` metric carry the weight instead. Revisit before freezing; do not
   patch it with an LLM judge.
3. **Repeat count — now the suite's sharpest open question, because the
   answer is currently "one".** The economics were always the problem: n = 20
   across 5 sims × 2 conditions × 15 tasks is 3000 runs and a five-figure
   token bill, and even n = 5 could not separate 0.6 from 0.8. On 2026-08-10
   the owner resolved the cost side by removing repeats entirely (§3.5) in
   exchange for a 45-minute ceiling. That resolves the bill and **not the
   statistics**: at one run per cell the suite has no variance estimate at
   all, so it can report what happened and cannot report how reliably it
   happens.
   Current answer, stated as the trade it is: **one run per cell for the
   campaign; re-runs are bought deliberately, per cell, for the specific
   claims that need them.** A cell must be re-run before it is quoted when
   (a) it is a comparative headline (§6.2.8's reproduce-or-drop gate, which
   still binds), or (b) the claim would turn on a margin of one task (§3.5
   rule 6). The old "re-run at n ≥ 20 if it lands within a CI width" trigger
   no longer applies as written — there is no CI to be within — and needs
   re-deriving for a single-run design. **Two consequences are unresolved and
   are recorded rather than papered over:** the frozen withdrawal rule F is
   arithmetically unevaluable at n = 1 (§3.5), and the MPP-HW definition in
   §10.8 was written around a pass rate.
4. **`PYTHONHASHSEED`.** `husky_random` seeds from `hash(robot.getName())`, which
   is randomised per process. Graders must therefore be threshold-based, never
   trajectory-based, and A1 must not assume run-to-run reproducibility of the walk.
   Decide whether the benchmark pins `PYTHONHASHSEED` (more reproducible, less
   representative of what a user gets) — currently: **do not pin**, and say so.
5. **Model coverage.** One model produces a benchmark about that model. At least
   two model tiers should be run before any published comparison, or the result is
   "this model, on these simulators", which is a weaker claim than it will read as.

---

## 10. Platform accessibility and resource scaling

Agentic productivity is only useful if the simulator can be installed and run
on the machine the developer actually owns. This programme therefore treats
platform accessibility as a **first-class measured dimension**, not a footnote:
a simulator gets no free pass because its normal target is a workstation, and
inability to run on a constrained tier is itself a result.

It also answers the concrete question this project keeps meeting in practice:
**can an autonomous agent build a useful robot simulation on an RTX 3060-class
laptop with 16 GB of RAM, or is workstation hardware required?**

### 10.1 Three separate questions

| question | what is measured | why it matters |
|---|---|---|
| **Can it exist on this machine?** | installability, dependency resolution, disk footprint, driver/OS compatibility, first successful process start | the access barrier before any robotics work begins |
| **Can it become usable?** | time to ready state, first valid frame / state step, peak RAM & VRAM, crash/OOM behaviour | separates "installed" from "practically runnable" |
| **Can an agent build with it?** | task outcomes (one run per cell, §3.5 — a count of tasks completed, never a rate), TTWS, actions, tokens, restarts, resource pressure on representative tasks | whether limited hardware destroys the agentic workflow itself |

### 10.2 Outcome taxonomy: never hide a non-run

Platform outcomes are **disjoint from** the run outcomes in §3.3 and are
recorded in their own field. A cell that never launched has **no task
outcome at all** and must never be rendered as a failed attempt at the task.

| code | meaning | handling |
|---|---|---|
| `PASS` | installs, launches, completes the stage | metrics reported normally |
| `PASS-BELOW-MIN` | works although the hardware is below the vendor's published minimum | **success**, with a visible below-minimum flag |
| `DEGRADED` | runs but misses a declared usability threshold (swap thrashing, extreme RTF, repeated recoverable renderer failure) | metrics reported, marked degraded |
| `UNSUPPORTED-OS` | vendor does not support this OS/arch | coverage gap, not a quality failure; any experimental attempt reported separately |
| `BELOW-VENDOR-MIN` | machine is under a published minimum *before* execution | a **condition flag, not a verdict** — attempt the run anyway |
| `INSTALL-FAIL` | canonical install cannot complete in the setup budget | failure; exact logs preserved |
| `LAUNCH-FAIL` | installed runtime never reaches ready state | failure |
| `GPU/DRIVER-BLOCK` | required GPU feature, driver or graphics API unavailable | failure, with root-cause class |
| `OOM-RAM` / `OOM-VRAM` | killed by host / GPU memory exhaustion | failure |
| `TIMEOUT` | no ready state or task success inside the frozen budget | failure |

**The fairness rule that makes this honest: `BELOW-VENDOR-MIN` is not
`LAUNCH-FAIL`.** If a vendor publishes a 16 GB VRAM minimum and the thing
nonetheless runs on a 6 GB laptop GPU, that is `PASS-BELOW-MIN` and it counts
in the vendor's favour. Conversely a failure is recorded because it was
**observed**, never inferred from a spec sheet. Isaac Sim 6.0's published
minimum (32 GB RAM, RTX 4080, 16 GB VRAM, no-RT-core GPUs unsupported) puts
half our ladder below its floor; those cells are still attempted and their real
behaviour recorded.

**Our own red cells, predicted before running so they cannot look like a
surprise:** OmniSim's macOS build is untested, so OmniSim expects
`UNSUPPORTED-OS` on the Apple Silicon row of its own benchmark; and the Newton
runtime's CPU-only path has never been exercised on a machine with no discrete
GPU in this tree, so P0/P1 are genuinely unknown for us. Both publish whichever
way they land.

### 10.3 The platform ladder

Real machines, because VRAM capacity, mobile power limits, graphics APIs and
driver support cannot be faithfully emulated by throttling a workstation.

| id | class | CPU / RAM | graphics | purpose |
|---|---|---|---|---|
| **P0** | constrained office / mini-PC | 4C/8T, 8 GB | integrated, no discrete GPU | the accessibility floor; CPU/headless viability |
| **P1** | low-cost developer laptop | 6C/12T, 16 GB | integrated or 4 GB discrete | ordinary non-RTX developer hardware |
| **P2** | legacy gaming laptop | 6C/12T mobile 35-45 W, 16 GB | RTX 3060 Laptop, 6 GB | **the critical cell** — OmniSim's own development machine |
| **P3** | mainstream developer PC | 8C/16T, 32 GB | 8 GB RTX-class | typical current midrange |
| **P4** | high-end consumer workstation | 12C+, 64 GB | RTX 4080-class, 16 GB | approximately Isaac Sim's declared minimum |
| **P5** | flagship desktop | 16C+, 64-96 GB | RTX 5090-class, 32 GB | consumer ceiling |
| **P6** | professional workstation | 24C+, 128 GB | pro RTX, 48 GB+ | large-scene / high-sensor ceiling |

**What we actually own** is P2 (RTX 3060 laptop, machine `9722d23d12a3`) and a
laptop 5070 Ti, plus rentable RunPod 4090s at roughly P4/P5. Every other rung
is **declared and unrun**, and the coverage table shows it as such. Per the
machine-attribution rule no result is ever recorded as "local" — it names its
machine id, and `env_fingerprint.py` is what produces it.

### 10.4 Controlled resource ablations

Physical tiers confound several variables at once, so selected headless tasks
repeat on **one** host with a single resource constrained at a time.

| ablation | levels | metrics |
|---|---|---|
| CPU cores | 2, 4, 8, 16 | TTWS, startup, RTF, tool latency, saturation |
| System RAM | 8, 16, 32, 64 GB hard limits | install/launch outcome, peak RSS, swap, OOM, pass rate |
| GPU / VRAM | none, 4, 6, 8, 16, 24/32, 48 GB **real devices** | launch status, peak VRAM, sensor throughput, pass rate |
| Storage | SATA vs NVMe; 30/60/120/500 GB free | install time, footprint, cold-start penalty |
| Power mode | battery-balanced vs plugged in | wall time, RTF, throttling (secondary only) |

**A software memory cap does not reproduce a smaller GPU.** VRAM ablations
require real partitions or real devices; anything else is a simulation of a
constraint, and is reported as such or not at all.

### 10.5 Operating-system matrix

| platform | status | rules |
|---|---|---|
| Ubuntu 24.04 LTS x86-64 | mandatory | pin kernel, driver, image, repos |
| Windows 11 x86-64 | mandatory | **native only — WSL is never substituted for a Windows result** |
| macOS on Apple Silicon | mandatory where officially supported | unsupported systems get `UNSUPPORTED-OS`, never an invented port |
| Linux aarch64 | optional | only where real native packages exist |
| WSL2 | separate optional row | never combined with native Linux or native Windows |

### 10.6 Vendor requirement annotation

Before running a frozen simulator version, archive its official
system-requirements page and attach a machine-readable annotation
(`sims.py` gives each comparator a `vendor_minimum`) distinguishing: (a)
supported and meets minimum, (b) supported OS but below minimum hardware, (c)
unsupported OS/arch, (d) no published quantitative minimum. **This is metadata,
never a substitute for testing** — it decides how a result is *labelled*, never
whether the run is attempted.

### 10.7 The platform gate: staged execution

Stop early on a machine that cannot launch, but preserve the exact point of
failure. **Charts show the highest stage reached, never a binary
supported/unsupported label.**

| stage | budget | pass contract |
|---|---|---|
| **G0** requirement classification | 2 min automated | machine profile captured; vendor flags assigned from frozen metadata |
| **G1** install | 30 min | canonical vendor-supported install completes; footprint recorded |
| **G2** cold launch | 10 min | ready state from cold cache; no fatal errors; first valid step |
| **G3** warm launch | 5 min | second launch reaches ready state; warm time recorded |
| **G4** minimal scene | 10 min | ground + one dynamic body advances 10 simulated seconds with finite state |
| **G5** representative agentic build | normal task budget (§2.4) | the frozen representative task set passes |
| **G6** integrated application | the §2.4 ceiling (45 min) | attempted only on cells that passed G4; failure is a valid platform result |

G5 and G6 inherit the §2.4 ceiling **and the single-run protocol** — the
platform programme gets no longer budget and no extra attempts of its own, or
the cost bound in §2.4 would be meaningless.

### 10.8 Resource-efficiency metrics

| metric | definition |
|---|---|
| **MVP-HW** (minimum viable platform) | ⚠️ same defect as MPP-HW below: "at least 80 % of representative agentic trials" presumes repeated trials. Unresolved (§9.3) |
| **MPP-HW** (minimum productive platform) | ⚠️ **needs re-deriving for the single-run protocol and is not usable as written.** It reads "lowest tier where representative `pass@1` is 80 %+ **and** median TTWS is within 2x that simulator's P4 TTWS" — an 80 % threshold on a pass rate the suite no longer measures, and a median over repeats it no longer has. Recorded as unresolved (§9.3) rather than silently reinterpreted as "4 of 5 tasks passed once", which is a different and much weaker statement |
| Platform coverage rate | passed mandatory (OS x hardware) cells divided by attempted applicable cells; unsupported-by-vendor cells shown separately |
| Cold / warm launch time | process start to ready state, from cleared vs normal caches |
| Install footprint | simulator + required dependencies + benchmark cache, before task assets |
| Peak RAM / VRAM | process-tree host and GPU memory during the task |
| Resource-normalised TTWS | TTWS reported **beside** its hardware tier — never used to pretend unlike machines are equivalent |

### 10.9 Platform tasks

Four platform-specific prompts, byte-identical across simulators like every
other prompt (§4.4.1), sharing the same fixed system prompt:

| id | prompt intent | verifier |
|---|---|---|
| **SETUP-01** | install the target simulator from its official path on a clean machine and launch it once | pinned version, process launch, ready-state signal, first valid step, footprint, host/driver controls unchanged |
| **PLATFORM-01** | smallest valid dynamic simulation: ground + one 0.20 m, 1.0 kg cube dropped from z = 1.0, at least 10 simulated seconds, comes to rest | first dynamic motion, finite state, rest height, no prohibited state write, wall time, RTF, RAM, VRAM |
| **PLATFORM-02** | the BUILD-02 differential-drive motion sequence, **without escaping the resource limits** | the BUILD-02 contract, plus unchanged host resource controls |
| **PLATFORM-03** | 640x480 at 15 fps camera + 180-sample 10 Hz LiDAR, 60 simulated seconds collision-free, both streams consumed | sensor cadence/resolution, read counts, dropped frames, RTF, CPU/RAM/GPU/VRAM |

`PLATFORM-02` results are reported as a **platform** task and never merged into
the normal-hardware BUILD-02 score. A headless pass never erases a GUI platform
limitation: the two are reported separately.

**Status: this whole section is DESIGN. No platform cell has been run.** It is
written now so the ladder, the taxonomy and the gates are frozen before any
result exists to shape them — the same reason the task set was pre-registered.

---

## 11. Platform-aware reporting

The headline public artifact is a **platform heatmap**, then task success/time
charts within each hardware tier.

- **Never collapse "cannot launch" into an infinite TTWS bar.** Use the explicit
  reason code from §10.2. A missing bar and a failed bar are different claims.
- Cell display: `PASS` with TTWS and peak memory; `PASS-BELOW-MIN` with a
  warning marker; then the failure codes verbatim.
- `UNSUPPORTED-OS` and `NOT_EXPRESSIBLE` (§6.4) may **never** be drawn in the
  same colour as a failure. They are absences, not defeats.
- For side-by-side video, print the machine spec above each simulator and start
  from the same frozen prompt. If one cannot launch, the recording stops at the
  real error and that panel **stays on screen** while the others continue.
  **Never substitute a stronger machine for one simulator inside a comparison
  panel.**
- Publish per-OS leaderboards **and** the coverage heatmap. A simulator that
  excels on Ubuntu but cannot serve a constrained Windows laptop should show
  that plainly — including when it is us.

---

## 12. What v0.3 took from the AgenticSimBench v0.2 draft, and what it did not

v0.3 is a **merge**, and the parts that were declined are recorded here so the
decision is auditable rather than silent.

**Adopted** (sections 10-11 above): the platform-accessibility programme in
full — the three questions, the never-hide-a-non-run taxonomy, the P0-P6
ladder, resource ablations, the OS matrix, vendor annotation, the G0-G6 staged
gate, the efficiency metrics, the platform task prompts and the heatmap
reporting rules. Also adopted: the comparator expansion to seven primary
systems with a separate extended research set (§6.1).

**Deliberately kept from the predecessor spec, because the draft dropped them
and each is load-bearing:**

| kept | why the draft's version was not enough |
|---|---|
| **`shell` vs `shell+tools`** (§4.1) | the draft runs one condition per track. The bare-shell arm is the only defence against *"you gave our simulator a worse interface"* — under it the interface **is** a shell, and the same one. It also turns the value of an agent surface from an assertion into a **measurement**, including when the answer is unflattering: our own first A/B had bare shell tie full-surface at 10/10 with **fewer** tool calls |
| **The documentation ablation** (§6.3) | the draft's OFFLINE track allows local files, and `AGENTS.md` is a local file that hands over the non-obvious answers. Unmeasured that is a confound; measured, "our documentation is the moat" is a true and useful headline, and materially different from "our API is the moat" |
| **Author-and-contestant machinery** (§8.2) | the draft cites SimBenchmark but not its lesson: it was discredited *because its authors were a contestant*. Freeze rules alone do not fix that. The 30-day correction window, reviewer-run cells, adopting a maintainer's better scaffold as the published headline, and the required hostile summary all stay |
| **Externally sourced tasks** (§8.3) | pre-registration stops us reshaping tasks after seeing results; it cannot make a self-authored set representative. The external column stays, and still **leads the report** when it disagrees with ours |
| **The `progress` ordinal** (§3.2) | an outcome plus failure counts hides "we reached level 3 and they reached level 1" — a real finding, and one we must publish when it points the other way |
| **`NOT_EXPRESSIBLE`** (§6.4) | stronger than the draft's "predeclared capability exclusion": it needs a justification from the competitor's **own** docs, and a single credible counter-example flips it and the cell gets run |
| **The oracle/null gate** (§7.1) | cheap, catches a broken grader before tokens are spent, and enforces *no task may be passable by doing nothing* — exactly the check C2 needed and did not have |
| **No single composite score** | the draft keeps ADS/PAS as secondary. We keep the harder line: there is no one number, and the answer is the per-task table |

**Declined outright:**

- **The draft's scale.** 15 tasks x 7 simulators x 20+ trials x 2 doc tracks is
  4,200 trials before the platform matrix, the real-GPU ablations and a
  mandatory Apple Silicon row. That is a specification for a funded benchmark
  lab. v0.3 keeps it as the north star and runs the slice in §7.
- **The draft's token budgets** (100k total for an S task). Too tight for a
  modern agent once a system prompt and tool results are counted, and it does
  not say whether cache reads count against the cap — an ambiguity that decides
  which runs die of budget exhaustion. We keep 400k in / 60k out per task and
  report cached and uncached separately.
- **Integrity instrumentation asymmetry, left unstated.** The draft requires
  `direct_pose_writes_after_t0` and access audits "where the simulator exposes
  them". Net effect: the simulator we control gets policed while a bare script
  that teleports may not be caught. Where an adapter cannot see a class of
  cheating, **that gap is published per simulator** rather than left implicit.
- **An unscoped determinism sidecar.** Determinism is a **per-solver** property
  here — bitwise on CPU `newtonSolver "mujoco"`, refuted on the GPU
  `mujoco_warp` path, cross-machine untested. The pinned solver is declared in
  the manifest or the row is not published.
