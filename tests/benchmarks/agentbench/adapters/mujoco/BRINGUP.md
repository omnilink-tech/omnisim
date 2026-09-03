# MuJoCo-lane bring-up record

**2026-08-09.** What is installed, how the arm was proven end to end, and the
exact numbers the bring-up runs produced. Companion to
[`SPEC.md`](../../SPEC.md) §6.1–6.4 (the fairness contract this arm is written
to) and to the sibling record [`adapters/webots/BRINGUP.md`](../webots/BRINGUP.md).
**The numbers below are bring-up telemetry, not benchmark results** — two
fixture scenes, no conditions, no *n*, no agent.

Machine: **`9722d23d12a3`** — RTX 3060 laptop GPU, AMD64 16-core, Windows 11,
CPython 3.12.9. (No GPU is used by this arm; it is recorded because every
number in this tree names the box that produced it.)

---

## 1. The install — **nothing was installed**

| item | value |
|---|---|
| package | `mujoco` **3.8.1** (`mj_versionString()` `3.8.1`, `mujoco.__version__` `3.8.1`) at bring-up; the pinned runtime is now `mujoco` 3.11.0 / `newton` 1.5.0 / `warp-lang` 1.16.0 (`scripts/packaging/newton_runtime_pins.py`) |
| interpreter | the system CPython 3.12.9 at `C:/Users/<user>/AppData/Local/Programs/Python/Python312/python.exe` |
| how it got there | **it was already present as a Newton dependency.** OmniSim's only physics backend resolves `newton` / `warp` / `mujoco` out of the system interpreter (AGENTS.md), and `python -c "import mujoco"` reported 3.8.1 before this arm existed. `machine_fingerprint()` lists it in the repo's own stack: `newton 1.2.0`, `warp_lang 1.13.0`, **`mujoco 3.8.1`**, `mujoco_warp 3.8.0.3`, `torch 2.5.1+cu121` |
| pip actions taken | **none.** Disturbing the system interpreter would break physics for the whole repo, and there was nothing to fix: the version present is current and is now pinned in [`sims.py`](../../sims.py) |
| interpreter selection | `$AGENTBENCH_MUJOCO_PYTHON`, else the parent's own `sys.executable`. Every launch probes it once (`launcher.probe_python`) and publishes the result on the row, so **a missing library reads as a missing library and never as a simulator that failed to simulate** |
| GL / display | `MUJOCO_GL` is deliberately **not** set by the launcher. A driver that opens a viewer or a renderer fails on a headless box, and that is the honest outcome of a benchmark that is headless for every arm; pinning a software backend would be the grader changing what the agent's program does |

If a campaign wants this arm isolated from the engine's interpreter, point
`$AGENTBENCH_MUJOCO_PYTHON` at a venv with `mujoco` in it — the child is an
ordinary Python process and does not care. That was **not** needed here and was
not done.

## 2. The pieces (all new, all under this directory)

| file | role |
|---|---|
| `recorder.py` | the grader-owned **observer**. Wraps `mujoco.mj_step` / `mj_step1` / `mj_step2` before the agent's driver runs, takes the frozen t=0 scan on the first observed step, samples every body's pose per step, reads `data.ncon`/`data.contact` per step, and **owns termination** (MuJoCo has no `--duration`; it raises a `BaseException` out of `mj_step`, which an ordinary `except Exception:` cannot swallow) |
| `runner_main.py` | the child process: compiles the agent's MJCF **on its own first** (so "did the world load" is answered independently of whether the driver works), installs the recorder, runs the driver unchanged as `__main__` with `sys.argv[1]` = the model path and cwd = the model's directory, and **always** writes the artifact set in a `finally` |
| `launcher.py` | the parent: resolves the interpreter and probes it, finds the deliverable's driver, runs the child under a timeout, writes `process.json`. Everything except the `subprocess.run` is a pure function |
| `recording.py` | reads one run off disk. Imports neither `mujoco` nor the neutral schema, so the mapping is auditable on a machine with no simulator |
| `evidence.py` | the mapping into `graders/evidence.EvidenceBundle`, plus `build_view_evidence` for the camera channel |
| `mujoco_lane/bringup_scene.{xml,py}` | the bring-up fixture: plane, two named walls, a falling box, an unactuated crate, and a 2-joint wheeled body an actuator drives. The driver loops **without an end condition on purpose**, so the grader's clock is what stops it |
| `mujoco_lane/r1_probe.{xml,py}` | the R1 **instrument** probe: the 10 m arena, four walls, and the five obstacles at the exact poses `benchmark_assets/obstacles.json` publishes — driven **blind** at the goal so it collides. Not a solution and not an oracle (§5) |
| `mujoco_lane/r1_oracle.xml` | the R1 **oracle scene** and the graded artifact of the deliverable pair: same arena and same five published obstacle poses, the rover at the declared start pose (yaw 0), plus a `lidar` mount site on a mast |
| `mujoco_lane/r1_oracle.py` | the R1 **oracle driver** — casts a 181-beam `mj_ray` fan, maps what comes back, A\*s over the map it built, and drives. PASSes 6/6 (§3.3) |
| `mujoco_lane/r1_null.py` | the **null** half: steps the same scene and commands nothing |
| `mujoco_lane/r1_hardcode.py` | the **negative control that passes**: the oracle's planner and control law fed a map READ from `obstacles.json` instead of sensed. It exists to measure where R1's evidence stops (§3.3) |
| `test_launcher.py`, `test_mujoco_adapter.py` | 66 tests that need **no MuJoCo at all** |
| `r1_perturbation_sweep.py` | the anti-hardcode sweep tool: catch rate vs oracle pass rate vs layout legality, per `PERTURBATION_MAX_M` (§3.5) |
| `test_r1_discriminates_mujoco.py` | the SPEC 7.1 oracle/null gate for R1 on this arm, as 13 real MuJoCo runs (§3.3, §3.5) |
| `test_mujoco_end_to_end.py` | 16 tests that build a tiny MJCF, run it for real, and assert the neutral bundle. Skipped (not failed) without `mujoco` |

Artifact set produced per run — exactly what `recording.read_run` parses:
`trajectory.json` + `trajectory.csv`, `roster.json`, `contacts.json`,
`completion.json`, `model_info.json`, `model_load.json`, `process.json`,
`stdout.log`, `stderr.log`.

## 3. End-to-end proof (bring-up telemetry — **NOT** benchmark results)

### 3.1 `bringup_scene.xml`, 10 s window, two runs

- exit code **0**, `timed_out=false`, stopped by the **grader's clock**
  (`stopped_by: "duration"`) — the driver had no end condition
- **5,000 steps**, `recorded_s = 9.998` simulated seconds
- wall clock: **1.13 s / 1.09 s** for the whole launch, of which the child's
  own run was **0.406 s / 0.390 s** (the rest is process start + the one-off
  `import mujoco` probe)
- rover: path length **9.883 m**, net displacement **2.692 m** (it arcs)
- dropped box: z **0.900 → 0.1499 m** — landed, half-extent above the floor
- contacts: `supported=true`, **54,927** observed over 5,000 steps,
  **54,927** naming two distinct participants, deduped to **3** pairs
  (`floor/rover`, `floor/parked_crate`, `floor/dropped_box`)
- inventory: **6** roster entries (3 top-level bodies + 3 world geoms),
  **8** in the t=0 inventory (+2 wheel links), every one with a world AABB
- attribution: `mujoco` / `mjSOL_NEWTON` / `mjINT_IMPLICITFAST` /
  `mjCONE_PYRAMIDAL`, read from the compiled model
- `adapters.check_bundle`: **zero findings**
- state resets: **0**, MuJoCo warnings: **none**

**The two runs were identical to every digit printed** — same path length, same
net displacement, same contact counts. That is *n = 2, one machine, one
process shape*: it is a sanity signal, not a determinism claim, and nothing in
this tree has measured MuJoCo determinism across machines.

### 3.2 `r1_probe.xml`, 12 s window — the R1 instrument

Launch wall **1.12 s**, child **0.359 s**, **6,000 steps**, 11 roster entries /
13 in the t=0 inventory, **12,297** contacts observed and **12,297** naming two
distinct participants.

`graders/r1_core.grade` on the resulting bundle, unmodified:

| assertion | outcome | measured |
|---|---|---|
| R1.1 the run is clean | **PASS** | exit 0, 0 error-class events, driver completed |
| R1.2 one drivable robot | **PASS** | 1 robot-class body, 1 with ≥ 2 joints (`rover`) |
| R1.3 the obstacles are intact | **PASS** | **5 of 5** matched **by geometry**; blocking the straight line: `OBSTACLE_1`, `OBSTACLE_2`, `OBSTACLE_3` |
| R1.4 the goal is reached | FAIL | final `(-1.771, -2.072)`, 8.377 m from the goal |
| R1.5 nothing was hit | FAIL | 1 robot–obstacle contact, first hit **`OBSTACLE_2/rover`**, travelled 3.514 m |
| R1.6 drove around, did not teleport | FAIL | path 3.514 m (floor 11.5), largest step **0.0032 m**, start error 0.000 m |

Outcome **FAIL**, progress 3 — **and that is the grader working, not a score.**
The probe's driver is deliberately blind: it reads the forward rangefinder,
prints it, and ignores it. Three things this establishes that nothing else
could:

1. **R1.3 is decidable on this arm.** All five published obstacles were matched
   by position and footprint from *measured* world AABBs, and the
   straight-line blockage the whole sensing argument rests on was re-derived
   from that geometry. On the OmniSim and Webots arms this same assertion
   currently reports an **INSTRUMENT GAP** because their bounds scan is
   name-keyed; here the t=0 scan bounds every body and every world geom without
   being handed a list, so an agent that calls its boxes "crate A" is as
   visible as one that uses the published names.
2. **R1.5 is falsifiable.** A robot-vs-obstacle contact was named. "Nothing was
   hit" is worth nothing until a hit can be reported, and it can.
3. **R1.6's teleport witness is real.** 0.0032 m per sample at one sample per
   2 ms step.

### 3.3 The oracle/null gate for R1 — **run 2026-08-09, on the same box**

SPEC 7.1's gate is two halves and is worth nothing with one of them: an
**oracle** that performs the known-good solution must PASS, and a **null** that
does nothing must not. §5 used to say this arm had only the failing half. It
now has both. Four programs, **one scene** (`r1_oracle.xml`), the unmodified
`graders/r1_core.grade`; holding the world fixed and varying only the program
is the design, so a verdict difference can only be the program.

| program | R1.1 | R1.2 | R1.3 | R1.4 | R1.5 | R1.6 | outcome |
|---|---|---|---|---|---|---|---|
| `r1_null.py` — steps, commands nothing | PASS | PASS | PASS | FAIL | FAIL | FAIL | **FAIL** |
| `r1_probe.py` — drives blind at the goal | PASS | PASS | PASS | FAIL | FAIL | FAIL | **FAIL** |
| `r1_hardcode.py` — a map read off `obstacles.json` | PASS | PASS | PASS | PASS | PASS | PASS | **PASS** |
| `r1_oracle.py` — senses with `mj_ray`, then goes | PASS | PASS | PASS | PASS | PASS | PASS | **PASS** |

**The oracle, measured.** Final xy **(3.9062, 4.0018)** → **0.0938 m** from the
goal (tolerance 0.30); **0** robot–obstacle/wall contacts over **14.272 m**
actually travelled; integrated path **14.272 m** (floor 11.5, straight line
11.31 and blocked by OBSTACLE_1/2/3); largest single-sample step **0.0022 m**
(bound 0.25 — a drive, not a hop); start error 0.000 m; 5 of 5 obstacles
matched by geometry. It arrives at **19.8 s** of a 60 s window, exit 0,
`stopped_by: "driver_returned"`, 9,901 steps, child wall **1.28 s**. Three
consecutive runs were identical to every digit printed (*n* = 3, one machine,
one process shape — a sanity signal, not a determinism claim).

**How it senses:** `mujoco.mj_ray`, the primitive MuJoCo's own `<rangefinder>`
is built on — a **181-beam** planar fan over **180°** to **6.0 m**, cast from
the `lidar` site every control tick (20 Hz), `flg_static=1` so the static
obstacle and wall geoms are visible. Measured on the graded run: **384 scans /
69,504 beams**, and the map the planner uses starts empty — its 278 occupied
cells were all put there by a beam. The MJCF alternative (181 `<rangefinder>`
sensors on replicated sites) was considered and rejected: it costs 181 casts on
every one of 30,000 integration steps instead of on the 20 ticks a second that
consume them.

**Margin, checked independently of the contact channel.** Minimum distance from
the robot's centre to any published obstacle surface over the whole run:
**0.505 m** (OBSTACLE_4), against the **0.30 m** radius that bounds the robot's
own footprint — **0.205 m** of true clearance. This is the second oracle: the
first, planned at a 0.5 m inflation, passed with **0.264 m** centre-to-surface
at OBSTACLE_5's corner, i.e. *inside* its own bounding radius. It did not
collide, but only because the corner fell where the footprint is narrow, and an
oracle that passes on 2 cm has not shown a task is passable. The planner's
inflation is 0.7 m for that reason.

**The path length is not sampling noise.** Integrated at one sample per 2 ms
step it is 14.272 m; decimated 1-in-50 it is 14.251 m and 1-in-250 it is
14.170 m. 0.7 % of the number is sampling, so the 11.5 m floor is cleared by
geometry rather than by counting jitter.

**Sensing, proven by moving the obstacles.** The oracle was re-run against
**20** layouts produced by `r1_core.seeded_offsets` (every obstacle displaced up
to `PERTURBATION_MAX_M` = 0.6 m, seeds 1–20): **20/20** reached the goal with
**zero** contacts, worst-case true clearance **+0.081 m**, paths 11.91–18.98 m.
It cannot be a memorised route. (R1.3 correctly FAILs on those runs — the world
deliberately is not the published one.)

**⚠ What this gate does NOT establish, measured rather than assumed.**
`r1_hardcode.py` is the oracle's planner and control law with the map **read
from `benchmark_assets/obstacles.json`** before the wheels turn and **not one
beam cast** — and it passes **6/6** on the published layout (13.60 m, 0.098 m
from the goal, 0 contacts). So a green R1 cell means "the agent got a robot to
the goal around the obstacles without touching anything", **not** "the agent
built a perceiving robot". That is what `tasks/R1_lidar_nav/meta.json`'s
`anti_hardcode` and `status` already declare in prose; this is the measurement
under the words. The declared fix is the seeded perturbation, and on the same
20 seeds it catches the hardcoded path **3 times out of 20 (15 %)** — seed 10
by a named collision (`wall_west/rover`), seeds 17 and 18 by failing to arrive.
**At *n* = 1 graded run per cell a memorising agent survives the perturbation
~85 % of the time**, so the perturbation as specified (≤ 0.6 m) is a weak
filter against a path planned with clearance to spare, not the clean
discriminator the prose implies. **§3.5 sweeps the displacement to find out
whether that is a tuning problem** — it is not — **and finds two further
defects that would fire the day the perturbation is wired.** Read it before
changing `PERTURBATION_MAX_M`.

### 3.4 Failure modes, each proven to be a measurement rather than a crash

| case | what the arm records |
|---|---|
| MJCF does not compile | `model_load.compiled=false`, an `MJCF compile error:` error line, `live_load_ok=false`, **no attribution** (nothing was ever stepped) |
| the driver raises | exit code 1, `driver_error` carries the exception, `complete=false`, **no** engine error line (the agent's program is not MuJoCo speaking), `live_load_ok=true` (the model was fine) |
| no driver at all | `driver_rule` says "a MuJoCo scene does not move on its own", 0 steps, `reached_finalize=false`, `behaviour_starts={}` |
| the driver un-hooks the recorder | `hook_intact=false` and a `TAMPER:` note on the verdict |
| the solve diverges | see §4.3 — this one found a real bug |

Tests: `pytest tests/benchmarks/agentbench/adapters/mujoco -q` → **95 passed
in 27 s** (66 without MuJoCo, 16 with, and 13 more in
`test_r1_discriminates_mujoco.py`, which are real runs — the gate costs ~19 s
of the total and needs no GPU, no network and no model quota).

### 3.5 The perturbation sweep — can the anti-hardcode defence be tuned into working?

§3.3 measured the hole: `r1_hardcode.py` passes 6/6 by memorising the published
layout, and the declared defence (`PERTURBATION_MAX_M` = 0.6 m) fires 3 times
in 20. This section asks whether a larger displacement fixes it. Tool:
[`r1_perturbation_sweep.py`](r1_perturbation_sweep.py) — free, MuJoCo only.
**510 graded runs**, machine `9722d23d12a3`, 2026-08-09.

"Caught" is the **behavioural** pair R1.4/R1.5 — arrived, collision-free —
because R1.3 matches against the *published* spec and therefore fails for
everyone on a perturbed layout. "Genuine oracle failure" excludes R1.6's path
floor, for the reason in finding 2.

| max_m | seeds | memoriser caught | oracle genuine fails | oracle path-floor fails | fallback | fatal layouts | fused |
|---|---|---|---|---|---|---|---|
| **0.60** (current) | 20 | 3/20 = **15 %** | 0/20 | 0 | 0 % | 0 % | 35 % |
| 1.00 | 20 | 8/20 = 40 % | 0/20 | 0 | 0 % | 0 % | 55 % |
| 1.40 | 20 | 9/20 = 45 % | 0/20 | 0 | 5 % | 0 % | 55 % |
| **1.80** | 80 | 45/80 = **56 %** | **0/80** | 1 | 2 % | **0 %** | 70 % |
| 2.20 | 80 | 48/80 = 60 % | 0/80 | 2 | 4 % | 2 % | 75 % |
| 2.60 | 20 | 17/20 = 85 % | **1/20 = 5 %** | 2 | 5 % | **20 %** | 55 % |

Layout health over **1000** seeds per level (`--layouts-only`, instant). *Fatal*
= a box through the arena wall, on the start, on the goal, or no collision-free
route at all; *fused* = two boxes interpenetrating, which is cosmetic (MuJoCo
does not collide two world-welded geoms and the layout stays solvable) but does
mean five obstacles become four effective ones.

| max_m | fallback | fatal | unsolvable | outside arena | on start | fused |
|---|---|---|---|---|---|---|
| 0.60 | 0 % | 0 % | 0 % | 0 % | 0 % | 25 % |
| 1.00 | 0 % | 0 % | 0 % | 0 % | 0 % | 49 % |
| 1.40 | 2 % | 0 % | 0 % | 0 % | 0 % | 59 % |
| **1.80** | 5 % | **0 %** | 0 % | 0 % | 0 % | 61 % |
| 2.20 | 8 % | 2 % | 0 % | 2 % | 0 % | 59 % |
| 2.60 | 12 % | 11 % | 2 % | 9 % | 1 % | 55 % |
| 3.00 | 15 % | 25 % | 4 % | 22 % | 3 % | 48 % |

**Finding 1 — the honest navigator is not the constraint, up to 2.2 m.** Across
all **240 oracle runs** at every level the oracle recorded **zero**
robot-obstacle contacts and failed to arrive **once** (2.6 m, seed 1). The
displacement does not break sensing; it breaks the *layout* (2.6 m: 20 % fatal)
long before it breaks the sensor.

**Finding 2 — R1.6's 11.5 m path floor is a published-layout constant, and it
fails honest agents on a moved one.** Five oracle runs failed R1.6 having
arrived within 0.10 m of the goal, collision-free, on paths of **11.31–11.49
m**: the perturbation had opened a legitimately *shorter* route and the fixed
floor punished the navigator for taking it. `MIN_PATH_LENGTH_M` is derived from
"the straight line is 11.31 m and is blocked" — true of the published layout
and of no other. Wiring the perturbation requires recomputing the floor from
the **graded** layout (or leaning on `MAX_STEP_DISPLACEMENT_M`, which is what
actually catches teleporting).

> **BOTH FINDINGS BELOW WERE FIXED ON 2026-08-09 — they are kept as the
> evidence that produced the fix, not as live defects.** The seeded
> perturbation was retired (it saturates at ~51% catch against an optimal
> fixed path and then declines) and replaced by GRADE-TIME PLACEMENT in
> `r1_core.sample_layout`. Measured over 120 layouts x 2 drivers: the
> memoriser is caught **95/120 (79.2%)** and the honest `mj_ray` oracle
> passes **120/120**. Finding 2's fixed 11.5 m floor is deleted -- the
> floor is now derived from the graded layout, and 12.5% of honest runs
> would have failed the old constant. Finding 3's blindness is closed:
> the memoriser struck an obstacle in 95 runs and R1.5 counted **95/95**,
> against 0 of 20 before. `test_wiring_the_perturbation_would_currently_BLIND_R1_5`
> was deleted per its own docstring and replaced by
> `test_a_moved_obstacle_that_is_STRUCK_is_now_counted_as_a_hit`.

**Finding 3 — ⚠ wiring the perturbation as written would BLIND R1.5.**
`grade()` matches obstacles against `obstacle_spec()` (published, 0.05 m
tolerance) and R1.5 decides "was this a hit" from the names of the *matched*
obstacles. Move them and nothing matches, so a robot-obstacle contact cannot be
a hit. Measured at 1.8 m over 30 seeds: the memoriser **physically struck an
obstacle in 20 runs** (raw adapter pairs, e.g. `rover / OBSTACLE_2` at step
1982) and **R1.5 counted 0 of them**; R1.3 reported 0 of 5 obstacles found. The
memoriser still fails — through R1.4, because it is stuck against a box — but
the assertion whose whole job is collision-freedom reports the opposite of what
happened, and an agent that clipped a box and still arrived would PASS "nothing
was hit". Pinned by `test_wiring_the_perturbation_would_currently_BLIND_R1_5`
(delete it when `grade` takes the perturbed spec; it has no seed parameter
today).

**Finding 4 — the mechanism has a ceiling of ~51 %, and no displacement
reaches it reliably.** The catch rate is a property of the *memoriser's*
clearance, and the adversary chooses that. The best fixed path in the published
layout — the widest-corridor (bottleneck) path — has **0.875 m** of
centre-to-surface clearance, i.e. **0.575 m** of slack beyond the robot's
0.30 m radius. Catch rate against **that** path, geometrically, n = 500 seeds
per level:

| max_m | 0.6 | 1.0 | 1.4 | 1.8 | 2.2 | 2.6 | 3.0 | 3.5 | 4.0 | 5.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| caught | 0 % | 5 % | 22 % | 33 % | 41 % | 49 % | **51 %** | 51 % | 48 % | 45 % |
| fallback | 0 % | 0 % | 2 % | 6 % | 8 % | 11 % | 14 % | 16 % | 21 % | 30 % |

It **saturates at ~51 % and then declines**: a random displacement is as likely
to move a box away from the path as into it, and the fallback — which returns
the *published* layout, the memoriser's best case — climbs with the level. No
value of `PERTURBATION_MAX_M` makes this a reliable filter.

**Finding 5 — drawing the positions at grade time is ~3× more efficient per
run.** Publish the obstacle **count and sizes**, sample the **positions** under
the legality constraints the perturbation never checks (inside the arena, 0.1 m
apart, 0.6 m clear of start and goal, solvable, straight line still blocked).
Measured over 200 sampled layouts: **200/200 legal by construction**, and
**78.5 %** of them catch the strongest fixed path in a single run. Requiring
*K* independent layouts to pass, escape probability (1 − p)^K:

| mechanism | p (vs strongest memoriser) | K=1 | K=2 | K=3 | K=5 | K=8 |
|---|---|---|---|---|---|---|
| perturbation @ 1.8 m | 0.326 | 67 % | 45 % | 31 % | 14 % | 5.2 % |
| random placement | 0.785 | 22 % | 4.6 % | **1.0 %** | 0.05 % | — |

Two runs of random placement beat eight of the perturbation. Both need the same
unbuilt piece — the grader injecting obstacle poses into the agent's world — so
the extra cost of the stronger mechanism over the weaker one is ~zero.

**What this measures and what it does not.** One adversary model (a single
fixed path) on one arm. An agent that memorised a *family* of paths, or that
re-planned from a remembered map after one look, is not modelled here. Every
rate above is against `r1_hardcode.py` or against the bottleneck path, and both
are stated so a reader can see which.

## 4. Decisions that are decisions, not readings

### 4.1 What counts as a robot in MJCF

MJCF has no `Robot` node. `<body>` describes a manipulator link, a falling
crate and a wall alike, so the predicate is **built**, not read:

> A body is **robot-class** when it is **top-level** (its parent is the world
> body) **and something can drive its kinematic subtree** — either (a) an
> `<actuator>` whose transmission resolves into it, or (b) a non-zero
> `qfrc_applied` / `xfrc_applied` **observed on it during the run**.

Every transmission type is followed — `mjTRN_JOINT`, `mjTRN_JOINTINPARENT`,
`mjTRN_TENDON` (through its wrap path, so a tendon spanning two subtrees marks
both), `mjTRN_SITE`, `mjTRN_SLIDERCRANK`, `mjTRN_BODY` (adhesion) — because
refusing to follow one would silently declare a legitimately actuated robot to
be scenery. Channel (b) exists because a robot driven by applied forces is a
robot the *file* never declares; the run shows it.

A body inside such a subtree is a **link** (`member_of`), never a robot — the
identity predicate asks "is this body the robot the task named", and a forearm
is part of that robot without being it. `n_joints` counts articulations in the
subtree with **`mjJNT_FREE` excluded**: a floating base is not an articulation
and neither Webots nor OmniSim has a joint node for one, so counting it would
make the same robot report one more joint here than there.

**Failure modes, stated because R1.2 and A1 will read whatever this decides:**

1. **An actuated conveyor, turntable or door counts as a robot.** Anything
   top-level with an actuator does. R1.2 requires *exactly one* robot-class
   body with ≥ 2 joints, so an agent that actuates a piece of scenery fails
   R1.2 on a world that a human would call correct. This is the sharpest edge
   of the rule and nothing in MJCF distinguishes the two cases.
2. **A body moved only by direct `qpos` / `qvel` writes is not a robot.** The
   recorder cannot see state assignment between steps — only forces. Such a
   robot reads as scenery. (It is also close to the teleportation R1's prompt
   forbids, so the two errors partly cancel; "partly" is doing real work in
   that sentence.)
3. **A robot with its base welded to the world** (a fixed-base arm) is
   top-level and actuated, so it is robot-class — correct — but its
   `dynamic` flag is `false`, because `body_weldid == 0` is MuJoCo's own
   statement that physics cannot move that body. A core reading `dynamic` as
   "is a robot" would get the wrong answer; none does.
4. **A passive multi-body ragdoll** with no actuator and no applied force is
   scenery. That is intended (a free body is not a robot) but it means a task
   about an unactuated articulated object would need a different predicate.

### 4.2 Where a world geom becomes an object

MJCF's idiom for static scenery is a bare `<geom>` on `<worldbody>` — five
obstacle boxes are five geoms on **one** body. Folding them into that body
would report the scene as a single object whose AABB spans the arena, which is
not a conservative simplification but a wrong measurement. So **every geom
attached directly to `<worldbody>` is its own inventory entry**, and contacts
name it by its geom name (a contact is otherwise reported under the **top-level
body of the geom's subtree**, so a wheel striking a wall is reported as the
robot striking the wall — the same resolution the other two arms use, and the
one that does *not* flatter this one).

### 4.3 `data.time` is not a monotone clock — and it cost a hang to learn

When the solve diverges, MuJoCo raises `mjWARN_BADQACC` **and calls
`mj_resetData`**, which sets simulated time back to zero. A recording window
keyed on `data.time` therefore never closes. Measured here, on this arm's own
first end-to-end attempt with a badly conditioned scene (`timestep 0.05`,
Euler, stiff velocity actuators): **311,846 steps at a reported 0.015 s of
"simulated time"**, stopped only by the wall-clock guard after 30 s.

The window now runs on an **accumulated** clock that ignores the
discontinuities, and every reset is recorded (`state_resets`) and published on
the verdict as a loud note, because the state after a reset is the model's
*initial* state and not a continuation. The pose series keeps a monotone time
base so a path integral stays defined, and the discontinuity shows up where it
belongs — in the positions, where R1.6's teleport bound can see it. Pinned by
`test_a_diverging_run_still_terminates_and_publishes_its_state_resets`.

### 4.4 What counts as an error line

MuJoCo has no log file and no `ERROR:` prefix; it raises. Counted as
error-class events: a **model-compile failure** (the counterpart of the other
arms' "the world failed to load") and a `FatalError` / `UnexpectedError`, which
is what the bindings raise for a C-level `mju_error`.

**Not** counted: the `mjWARN_*` counters, including `mjWARN_BADQACC`. Neither
the OmniSim nor the Webots adapter counts its engine's divergence warnings, and
counting them here alone would make the same assertion stricter on MuJoCo than
on us — the one direction a comparison must never lean. They are recorded and
noted on every verdict instead, and a run that hits one shows the state reset
in its own pose series where the physical floors can see it. A traceback from
the agent's own driver is likewise not an engine error line; it is the
competitor's *controller* crashing, which shows up as a non-zero exit here
exactly as it does on the other arms.

## 5. What this arm can and cannot do today

**Expressible now (and only these — `sims.py` refuses the rest by name):**
`A1_husky_swarm_10`, `R1_lidar_nav`, `R2_arm_reach`. All three start from an
empty or world-free `initial/`, so they stage correctly for this arm with **no
task-side change at all**.

**Not expressible yet:** `B1`, `B2`, `B3`, `C1`, `C2`, `R3` — each ships a
`.wbt` fixture and there is no `initial_mujoco/` MJCF equivalent. That is a
missing **fixture**, not a missing MuJoCo capability, and
`require_implemented(sid, task)` now refuses those cells up front rather than
letting them become failures attributed to MuJoCo (SPEC 6.4).

**Not built:** the `shell+tools` bridge — see
[`EXCLUSIONS.md`](EXCLUSIONS.md) §1. Only the `shell` condition is runnable.

**Run, for R1 only:** the oracle/null gate (SPEC 7.1) — §3.3. R1 is now shown
to be **passable** on this arm (an `mj_ray` navigator PASSes 6/6) and
**failable** on it (a do-nothing driver FAILs R1.4/R1.5/R1.6), so an R1 number
from this arm is falsifiable. Both halves are pinned in
`test_r1_discriminates_mujoco.py` and re-run in the ordinary unit lane.

**Still not run:** the gate for `A1_husky_swarm_10` and `R2_arm_reach`, the
other two tasks this arm can express. A1 additionally needs the declared Husky
analogue below before an oracle for it means anything. Until those two are
gated, `pending` should keep naming them — the gate is per (task, arm), and R1
being green says nothing about A1.

**Since 2026-08-11 that sentence is enforced rather than advisory.** `pending`
was one free-text string and `sims.Sim.publishable` was one arm-wide boolean,
so the two items above — both about A1 and R2 — made **R1** unpublishable on
this arm while R1's own gate was green, on record and pinned by a test. The
field is now a list of `sims.Pending` items, each declaring which task ids it
blocks and, when it blocks a subset, *why the rest are immune*; the constructor
refuses a narrowed scope with no stated reason. `readiness.py` asks
`publishable_for(task)` and still ANDs it with the recorded oracle/null gate,
so **no cell can go green by scoping alone**. Consequence for this arm:

| task | pending items blocking it | gate on record | publishable |
|---|---|---|---|
| `R1_lidar_nav` | none | oracle PASS + null FAIL | **yes** |
| `A1_husky_swarm_10` | `A1_gate`, `A1_husky_analogue` | none | no |
| `R2_arm_reach` | `R2_gate` | none | no |

Neither A1 item was cleared or softened — they block exactly the task they were
always about. Pinned in `test_pending_is_task_scoped.py`, which also fails if
`pending` is emptied wholesale or if R1's gate record disappears.

⚠️ The same change removed a **false green** it would otherwise have exposed:
`readiness._discriminating` used to accept a cell when the arm's `pending` text
contained "gate" and the task id — and the sentence above says "still
**UNGATED**", which contains "gate". Over the 9×3 grid that fallback produced
two greens, `(A1, mujoco)` and `(R2, mujoco)`, and no true positives; it read
the exact opposite of what the text claimed. It is gone: only a recorded
verdict satisfies that gate now.

## 6. The deliverable convention this arm needs wired

An MJCF world is `.xml`, and **an MJCF world alone is inert**: MuJoCo has no
controller field and starts no processes, so the deliverable is a **pair** —
the model and the Python program that steps it.

- artifact (graded, and what `bundle.artifact` points at): `<task>.xml`
- driver (collected beside it): `<task>.py` — the same stem, which is the
  convention `launcher.find_driver` looks for first. It also accepts the only
  `.py` beside the model, and **refuses to guess** between several, because a
  wrong pick reads as an agent whose robot did nothing.

The exact registry lines are in the hand-off report, not applied here:
`agents/external.ARTIFACT_NAME` and the `cc_lane` collector are `.wbt`-keyed
today, and both are outside this arm's remit.

## 7. Known gaps and caveats (state, don't bury)

1. **Stepping paths the recorder does not wrap are invisible.** MJX
   (`mujoco.mjx`), `mujoco.rollout` and a native extension calling `mj_step`
   directly all produce **zero samples**, reported as "no samples" and never as
   "it did not move". A driver written that way is unmeasurable on this arm as
   built.
2. **A `plane` geom's AABB is its drawn extent, not its collision surface.** A
   MuJoCo plane is mathematically infinite; the bundle publishes the drawn box
   and says on every affected run that it understates the surface. A plane
   drawn infinite (`geom_size` 0 in x or y) gets **no** AABB at all.
3. **Non-z gravity is stated, never remapped.** MuJoCo's gravity is a free
   3-vector, so there is no permutation that generally makes an arbitrary one
   z-down; rotating a measurement to make it pass is a rewrite, not a
   translation. A world with sideways gravity gets a loud note saying every
   vertical assertion in that row measured the wrong axis.
4. **B2 has no initial pose on this arm.** The camera channel reads
   `<camera>` elements correctly (position, forward/up out of MuJoCo's
   −z-forward/+y-up frame, `cam_fovy` as the full **vertical** angle), but the
   *shipped* pose it must be compared against needs a task-side MJCF fixture
   that does not exist. The interactive viewer's free camera is deliberately
   not substituted: it is not part of the model and does not exist headless.
5. **The driver shares a process with the recorder.** It can un-hook
   `mj_step`; that is detected (`hook_intact`) and published as a `TAMPER:`
   note, but it is detection rather than prevention. The grader's artifacts are
   written last and overwrite anything the driver put in the run dir, so the
   grader has the last word on the files themselves.
6. **Cross-platform caveat.** This arm was brought up on native **Windows**,
   like the OmniSim arm and unlike the Webots arm (WSL2). Any published number
   must say so.
7. **`n = 2` on everything in §3.** These are fixtures, not conditions.
