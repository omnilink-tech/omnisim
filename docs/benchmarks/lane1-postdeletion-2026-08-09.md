# Lane 1 after the ODE deletion — the first post-deletion correctness campaign

**2026-08-09.** The first OmniBench lane-1 campaign run since `bdc02139` deleted
the ODE backend, plus the instrument built to replace what that deletion removed.

Until today [correctness-scope.md](correctness-scope.md) carried two open items,
and they were the whole of the project's correctness debt:

> - **No second in-engine path.** Nothing can currently distinguish "the solver
>   got this wrong" from "we handed the solver the wrong model" on a real world.
>   What should replace it … is **not decided**.
> - **No post-deletion lane-1 campaign.** The last published numbers predate both
>   the fixes' validation venue and the deletion.

Both are addressed here. One more defect was found on the way, in the worlds
themselves.

---

## 0. Provenance

| | |
|---|---|
| machine | `9722d23d12a3` — RTX 3060 Laptop (driver 596.36), AMD Ryzen 16-core, Windows 11 |
| build | `0fc15a998`, binary sha256 `ea1457384a032c99`, libController sha256 `2a47746ca78e090d` |
| stack | newton 1.2.0 / warp 1.13.0 / mujoco 3.8.1 / mujoco_warp 3.8.0.3 |
| solver | `newtonSolver "mujoco"` — CPU `mj_step`. **The GPU path was not measured.** |
| rows | 78 = 13 scenes × 6 timesteps (1, 2, 4, 8, 16, 32 ms), n=1 per cell |
| attribution | 78/78 carry a present, non-degraded, finalised `.newton.json` sidecar |

⚠ **n=1 per cell, one machine, one solver.** Every number below is a single
observation. Nothing here is a cross-machine claim, and the
`mujoco_warp` path — which [determinism-scope.md](determinism-scope.md) shows is
not even run-to-run reproducible — was not exercised.

⚠ **The box was not idle.** A concurrent step-cost ablation from another lane was
running engines during part of this campaign. That does not affect the physics
metrics (they are computed offline from recorded trajectories) but it **does**
affect `wall_ms_per_step`, so the cost column below is indicative only and must
not be quoted as a step-cost measurement — use
[step-cost-2026-08-06.md](step-cost-2026-08-06.md)'s differenced method for that.

---

## 1. The four defects that once made our integration score worse

`e7b9fb11` fixed four defects in **our Newton integration layer** (not in the
solver — raw MuJoCo scored fine on the same scenes throughout). They were
validated at the time but never re-measured by a published campaign. They are
now:

| lane-1 scene | the defect | then | **now (dt=4 ms)** |
|---|---|---|---|
| T3 roll | rolling-accel deficit from a wrong inertia | **47.63% low** | `roll_accel_rel_err` **0.00058** (a = 2.3952 vs analytic 2.3966), slip 0.00036 |
| T2 incline | friction-cone offset, effective μ 0.41 vs 0.5 | μ off by ~18% | `transition_angle_err_deg` **0.065**, `slide_accel_rel_err` 0.038 |
| T5 momentum | momentum leak | leaking | `angular_momentum_drift_abs` 0.51 at dt=4, **0.045 at dt=1** |
| T7 spin | total spin loss ("spin brake") | spin → 0 | spin retained; see §3 |

T3 is the clearest: a 47.63% acceleration deficit is now **0.058%**, i.e. the
scene is at the analytic answer to within the integrator's own truncation.

**What this does NOT say.** It does not say lane 1 is "green": §2 and §3 below
record what still deviates, and the validity audit
([lane1-validity-2026-08-07.md](lane1-validity-2026-08-07.md)) argues several of
these metrics score the wrong thing in the first place. Those recommendations
remain unimplemented.

## 2. What still deviates, and which of it is ours

| metric (dt=4 ms unless noted) | value | reading |
|---|---|---|
| T4 `energy_drift_rel` | **0.381** (0.124 at dt=1) | **the ceiling, not a defect.** Raw MuJoCo measures 0.387 on the same scene: this is integrator truncation, and we sit flush against it. |
| T2 `stick_violation_max_m` | **4.42e-04** | **real, and unchanged** from the pre-deletion figure. The validity audit lists it as a genuine residual (vs 2.9e-05 on the retired ODE arm). |
| T6 `settle_creep_m_s` | 4.2e-07 (dt=1), 5.7e-06 (dt=4) | creep is now essentially nil at usable timesteps. Do **not** compare this to the old "~100×" T6 gap: that comparison was disclaimed as not like-for-like in its own report. |
| T6 `stack_survivors` | 10/10 at dt≤4, **1/10 at dt=16** | a resolution limit, not a bug: `max_penetration_m` goes 9.0e-04 → 0.299 as dt grows. |
| T5 `linear_momentum_max` | 1.56 (dt=1) → 11.30 (dt=16) | grows with dt as expected; the metric is also flagged in the validity audit as partly a coordinate-representation artefact. |

## 3. The instrument that replaces the second in-engine arm

`bdc02139` did not remove the correctness *oracle* — that is analytic ground
truth and it is untouched. It removed the ability to run one `.wbt` through two
integrations and tell **"the solver got this wrong"** from **"we handed the
solver the wrong model."** That is the layer with this project's entire track
record of real bugs, and the layer no external arm can see: bare `mujoco` and
`pybullet` validate the solver, and neither of them reads `.wbt`.

The replacement is **[`lane1/translation_audit.py`](../../tests/benchmarks/omnibench/lane1/translation_audit.py)**,
and it answers the question more directly than the ODE diff did. A two-backend
diff reports *disagreement* and leaves you to decide which side is wrong; the
audit reports *"the world declares gravity 3.72 m/s² and the model MuJoCo
stepped has 9.81"*, which names the wrong side. It needs one engine, not two.

It compares the `.wbt`'s authored contract against
`OMNISIM_NEWTON_DUMP_MJMODEL` — the exact mjModel the solver stepped, dumped at
finalize — checking gravity (magnitude **and** the `coordinateSystem` up-axis),
timestep vs substeps, per-geom friction and `condim`, body masses, collidability,
and every declared-but-unread field. It refuses to audit a model it cannot
attribute to a verified Newton run.

**It is validated by construction, because a green audit is worth nothing
otherwise.** `--self-test` generates probe worlds and requires the audit to
*evaluate* each one, then hands the comparator deliberately wrong models and
requires it to go red:

| probe | expected | model | verdict |
|---|---|---|---|
| `enu_earth` (control) | (0, 0, −9.81) | (−0.0, −0.0, −9.8100004) | OK |
| `nue_earth` | (0, −9.81, 0) | (−0.0, −9.8100004, −0.0) | OK |
| `eun_earth` | (0, −9.81, 0) | (−0.0, −9.8100004, −0.0) | OK |
| `enu_mars` | (0, 0, −3.72) | (−0.0, −0.0, −3.7200000) | OK |
| `enu_mu` (μ=0.3) | μ reaches the contact | yes | OK |
| negative: model gravity halved | must ERROR | ERROR | OK |
| negative: model gravity zero | must raise the c77cbe98 signature | raised | OK |

The NUE/EUN rows are a **live re-verification of `c77cbe98`** (before it, all 210
NUE worlds had gravity projected to exactly zero and never fell), and `enu_mars`
re-verifies that `WorldInfo.gravity` reaches the solver at all — the bug where
every Newton world ran at −9.81 regardless of the field. 23 offline tests pin the
comparator in [`tests/benchmarks/test_translation_audit.py`](../../tests/benchmarks/test_translation_audit.py).

> ⚠ **The audit shipped with the exact failure mode it exists to catch, and that
> is worth recording.** The model dump interpolates live arrays, so gravity
> arrives as `np.float64(-9.81)`; a naive number regex matched the `64` inside
> the *type name*, returned six values instead of three, and the flagship gravity
> check **skipped itself while the report read green**. A check that cannot run
> now emits a finding instead of silence, and both are pinned by tests.

### What it found: the worlds were not self-describing

Run over the lane-1 corpus, the first pass returned **10 ERROR / 35 WARN across
13 scenes**. Every error was the same shape: the world declared a friction the
model never received.

| scene | declared in the `.wbt` | what the model actually got |
|---|---|---|
| T2 inclines (×7) | `coulombFriction 0.5` | μ = 1.0 |
| T3 roll, T6 stack | `coulombFriction 0.8` | μ = 1.0 |
| T1 bounce | `coulombFriction 0` | μ = 1.0 |

`ContactProperties.coulombFriction` is the **ODE-path** declaration and the
Newton backend does not read it. The friction that actually reached the solver
came from `run_omnisim.py`'s `OMNISIM_NEWTON_GROUND_MU` — from the shell, not the
file. The runner discloses that as a deviation, so **lane 1's own numbers were
never wrong**; what was wrong is that the world files could not reproduce them.
Anyone loading `t2_incline_a25_dt4.wbt` — a third party, or `run-headless` —
got μ = 1.0 and different physics from the published result.

That is a reproducibility defect, and it is precisely the failure the
`newton*` WorldInfo fields were added to fix. From `set_contact_solver_params`'
own comment in the engine:

> a world file was therefore NOT a complete description of its own physics: an
> agent tuned a working two-finger friction grasp here, wrote the world out, and
> the next person to load it got default friction and a soft contact and no
> grasp … the working configuration could not be handed to anybody, including to
> our own grader, which re-ran the world bare and scored the result a failure.

`gen_worlds.py` now declares the contact intent in the fields the engine reads
(`newtonGroundMu`, and `newtonContactKd 7` for T1's restitution), and no longer
emits the inert `ContactProperties` block or `physicsDisableTime`. The audit is
now **0 ERROR / 0 WARN over all 13 scenes**.

**The measurements are unchanged, and that was verified rather than assumed** —
the env var still takes precedence and carries an identical value, so the two
agree by construction. Re-run after the change:

| metric | before | after |
|---|---|---|
| T1 `bounce_height_rmse_rel` (dt=4) | 0.03811108985171102 | **0.03811108985171102** |
| T2 `stick_violation_max_m` (dt=4) | 0.0004418726569002015 | **0.0004418726569002015** |
| T2 `slide_accel_rel_err` (dt=4) | 0.03811175830671643 | **0.03811175830671643** |
| T2 `transition_angle_err_deg` (dt=4) | 0.06505117707799002 | **0.06505117707799002** |

### The corpus sweep: 863 live worlds, 176 with an unreachable friction

The lane-1 finding is not a lane-1 problem. `--sweep` runs the **static** half of
the audit — no engine, milliseconds per world — because the defect is statically
decidable: `coulombFriction` is not read and `newtonGroundMu` defaults to 1.0, so
a world declaring `0.5` and no `newtonGroundMu` provably runs at 1.0.

⚠ **The first sweep returned 259 and that number was wrong.** It counted
`.claude/worktrees/` copies and, worse, `results/` directories — recorded
artefacts of what some agent produced on a date, which must never be "fixed"
because rewriting them falsifies the record. Restricted to the live corpus:

| | |
|---|---|
| worlds scanned | **863** |
| worlds whose declared friction cannot reach the solver | **176** |
| `projects/policies` | 133 |
| `projects/robot_combat` | 19 |
| `projects/samples` (user-facing demos) | 16 |
| `tests/` + `projects/robots` | 8 |

**The demos were the sharp end.** Eight OmniArm 6 pick-and-place worlds declare
`coulombFriction 5` — the value [friction-grasp.md](../guide/friction-grasp.md)
says a two-finger pinch needs — and were running at 1.0, which that guide says
will not hold a grip. Only 3 of them have a `run_*.ps1` that exports the env var;
the rest had no path to the friction they declare. Meanwhile the *reference*
world, [`friction_grasp_minimal.omniworld`](../../projects/samples/demos/worlds/starter/friction_grasp_minimal.omniworld),
already does it correctly (`newtonGroundMu 3`, with a comment saying why) — so
the demos were inconsistent with the tree's own documented example.

`--fix` rewrites a world to declare `newtonGroundMu = its own coulombFriction`.
It is a **self-description** fix, not a retune: for any launcher that exports
`OMNISIM_NEWTON_GROUND_MU` the change is numerically inert (env > field >
default), so the runs a project actually performs are untouched and only the
previously-broken bare load moves. Applied to `projects/samples`,
`projects/robots`, `tests/api`, `tests/physics`, `tests/engine`: **14 worlds
rewritten**. Verified live — `omniarm6_toss_demo.omniworld` now reports **14/14 geoms
carry the declared mu 5**, and the smoke suite still passes including its
dynamics gate (`OK: gravity_rest_height`).

> ⚠ **Three were rewritten and then REVERTED, and the reason is the rule below
> applied to myself.** `omniarm6_anypick`, `omniarm6_anypick_line` and
> `omniarm6_bin_picking` are the three demos that *do* have a `run_*.ps1`, and
> those launchers export `OMNISIM_NEWTON_GROUND_MU = "1.5"` while the worlds
> declare `coulombFriction 5`. Writing `newtonGroundMu 5` into them is inert for
> the launcher path (env wins) but makes the file *authoritatively* state a
> friction the sanctioned run does not use — precisely the objection that keeps
> the migration away from `projects/policies`. They are left declaring nothing
> and listed as needing reconciliation: somebody has to decide whether 5 or 1.5
> is the intended number, and that is not a sweep's call. **Rule: never migrate
> a world whose launcher exports a different value than it declares.**

**Three classes it refuses to touch, each for a different reason:**

- **μ = 0 (8 worlds)** — inexpressible, see below. Writing `newtonGroundMu 0`
  would *claim* a fix while silently leaving the world at 1.0.
- **per-material frictions (2 worlds)** — `contactProperties` is a LIST, one
  entry per material pair; `newtonGroundMu` is a single global value. This is a
  harder defect than "the value never arrives": the structure **cannot be
  represented at all**, and no rewrite fixes it. Reported as its own check
  (`friction_per_material`) precisely so it is not mistaken for the migratable
  kind.
- **`projects/policies` (133) and `projects/robot_combat` (19)** — **deliberately
  left alone.** Their launchers already export a value, but *not always the one
  the world declares*: worlds declare 1.5, 2.0 and 5.0 while the recipes export
  `OMNISIM_NEWTON_GROUND_MU=2.0`. Writing the world's number would enshrine a
  friction that disagrees with the experiments actually run, making the file
  more authoritative and no more truthful. That reconciliation needs the
  workstream that owns the champions, not a sweep. Command when they want it:
  `python tests/benchmarks/omnibench/lane1/translation_audit.py --sweep projects/policies --fix`.

### A new defect, found and NOT fixed: μ = 0 cannot be expressed

`newtonGroundMu 0` means **"unset → engine default 1.0"**
([worldinfo.md](../reference/worldinfo.md); the C++ side skips the plumbing call
when every pref is `<= 0`). So the sentinel collides with a legal physical value
and **a frictionless world cannot state itself**. The env var can carry it —
`_contact_value` tests the raw string and `"0.0"` is truthy — which is why T1
still needs `OMNISIM_NEWTON_GROUND_MU=0.0` and is the one scene that remains not
self-describing. The generated world says so in a comment rather than pretending
otherwise. Same class as the `newtonNjmax 0 = keep default` sentinel; fixing it
means a distinguishable "unset" (e.g. `-1`), which is an engine change and is not
attempted here.

## 3b. The coincident floor, removed — and it mattered for T6

The validity audit's §6 flagged a confound in the two scenes that measure
contact: **T1 and T6 floors sat with their top face at exactly z = 0**, where
`OmNewtonBackend::openWorld()` adds an implicit ground plane *unconditionally*.
So the bodies resolved against two coincident manifolds, and T1's contact-damping
calibration had been fitted against that doubled stiffness.

Both scenes are now lifted to `scenes.FLOOR_TOP = 0.5`. It is a **pure vertical
translation** — the 1.0 m drop height, the 1 mm stack gaps and every relative
distance are unchanged, so the analytic references are untouched; the scorer
subtracts `FLOOR_TOP` for the same reason (T1's apex and T6's ground penetration
are floor-relative, not z-relative).

Measured at dt = 4 ms, one manifold vs two:

| metric | floor at z=0 (two manifolds) | floor at 0.5 (one) | |
|---|---|---|---|
| T1 `bounce_height_rmse_rel` | 0.03811109 | 0.03811227 | **no material change** (first peak identical to 15 digits) |
| T6 `stack_survivors` | 10 | 10 | unchanged |
| T6 `max_penetration_m` | 9.0176e-04 | 9.0179e-04 | unchanged |
| T6 `settle_creep_m_s` | 5.687e-06 | **7.013e-07** | **8× lower** |

So the confound was real but **scene-dependent**: negligible for T1's restitution
(the ball rests on the floor box and barely engages the plane), and material for
T6, where the bottom box sits directly on the doubled surface. **Part of T6's
"settle creep" residual was an artefact of the duplicate floor, not solver
creep** — which is exactly the kind of thing a leaderboard reading of lane 1
would have attributed to the engine.

The lane-1 corpus re-audits 13/13 clean after the change.

## 3c. The GPU solver path, probed for the first time

Everything above is CPU `mj_step`. `mujoco_warp` had never been run through
lane 1 at all, which left "our accuracy" scoped to one of the two solver paths we
ship. A bounded probe: **T3 at dt = 4 ms, three cold runs**, sidecar-confirmed
`solver: "MuJoCo (mujoco_warp, ...)"` on every run.

| | `roll_accel_rel_err` |
|---|---|
| CPU `mj_step` | 0.00058214 |
| GPU `mujoco_warp` (×3) | **0.00053589**, identical all three runs |

Two things, both narrow:

- **Accuracy is equivalent on this scene.** ~0.05% relative error either way, the
  GPU marginally closer to the analytic answer. Nothing here suggests the GPU
  path is a different-fidelity engine — on one scene.
- **The three runs were bit-identical**, which is a *scope note* on
  [determinism-scope.md](determinism-scope.md), not a contradiction of it: its
  0-of-24 refutation was measured on scenes carrying 80–336 concurrent contacts,
  and the mechanism is contact-buffer ordering. T3 has one contact. Recorded
  there in those terms.

⚠ **One scene, one dt, n=3, one machine.** T3 is a rolling sphere; it exercises
neither contact density nor articulation. This is a probe, not the GPU arm of a
campaign — that would be the full 7 scenes × 6 timesteps with repeats per cell,
on an idle box, and it has not been run.

## 4. A second attribution defect, in lane 1

`17c92a211` established that no row may be stamped with an engine that does not
exist, and fixed lane 3. **Lane 1 was still publishing `"engine": "omnisim-%s" %
backend`** — the `--backend` argument echoed back, never a verified value — so a
run whose Newton never finalised was published as `omnisim-newton` regardless,
and the console printed `"ode (no sidecar)"`, naming the deleted engine. It is
the same shape as the tool-honesty rule in AGENTS.md: never return the commanded
value under a measured key.

The rule now lives once, in
[`common/engine_launch.engine_attribution`](../../tests/benchmarks/omnibench/common/engine_launch.py),
and lane 3 delegates to it. A tree-wide `ast`-level test forbids any runner from
assigning or returning an ODE engine label, and the 78 rows above are attributed
from their sidecars.

## 5. Where correctness stands now

**Closed by this campaign**

- The post-deletion lane-1 gap: 78 rows, all sidecar-attributed.
- The four `e7b9fb11` defects re-measured; T3's 47.63% deficit is now 0.058%.
- The "not decided" replacement for the second in-engine arm: decided, built,
  self-validated, and it found a real defect on its first corpus run.
- Lane-1 worlds are self-describing (12 of 13; T1 blocked by the μ=0 sentinel).

**Also closed since**

- The audit **is now a campaign stage**: `run_all.py` runs it after the lane-1
  matrix and raises every contract violation as a `[translation]` finding, so a
  campaign cannot report lane-1 numbers without also reporting whether the
  worlds behind them describe themselves. `--skip-translation-audit` opts out.
- The corpus **has been swept** (863 live worlds); 14 worlds repaired, 162 remain (see §3 for which are deliberate).

**Still open — and these two need an owner's decision, not more code**

- **`newtonGroundMu` cannot express μ = 0, and the engine fix is WRITTEN BUT NOT
  LANDED.** The change is three lines and its blast radius is zero (no world in
  the tree declares `newtonGroundMu 0` today):
  1. `resources/nodes/WorldInfo.wrl` — default `0` → `-1`, comment updated to say
     "negative = unset";
  2. `OmNewtonBackend.cpp`, `set_contact_solver_params` — `"mu": (_f(mu) if
     _f(mu) > 0.0 else None)` → `>= 0.0` (ke/kd keep `> 0`: zero stiffness is
     nobody's configuration);
  3. the C++ `mGroundMuPref <= 0.0` gate — `< 0.0` for mu.
  It was **backed out unlanded** because `OmNewtonBackend.cpp` is being edited
  concurrently by the step-cost lane (joint-target caching, MPC gating). Landing
  it would have required either committing their unfinished work or building a
  binary that corresponds to no commit — and every number in this tree is
  attributed to a binary sha256. Land it when that file is quiet, then re-run
  `translation_audit.py --sweep <dir> --fix` to pick up the 8 μ=0 worlds it
  currently refuses.
- **`projects/policies` (133) and `projects/robot_combat` (19)** need their world
  declarations reconciled with what their launchers export — see §3.
- **AgentBench's pre-registration freeze is RED, and the remedy is a ceremony,
  not a code fix.** `187a9baab` raised the C2 floor (correctly — the task passed
  5/5 without the bug being fixed) but did not regenerate the freeze manifest or
  the committed oracle scripts, so 12 tests in
  `tests/benchmarks/agentbench/preregister/` fail on hash drift. `FREEZE.md`
  documents the procedure — a numbered **Amendment**, recording what changed, why,
  and the scored runs existing at amendment time — and its legality bar is "zero
  scored runs at amendment time". The evidence directory holds **10 rows, all
  from SCRIPTED oracle cells** (`run_oracles.py` replays canned agents through
  the real runner), not scored LLM campaign cells, so an amendment appears legal.
  It was **not executed here**: an amendment records a factual claim about the
  campaign's history in a pre-registration whose entire value is that it was not
  edited after seeing data, and that assertion belongs to the phase owner.
  Procedure: append "Freeze v2 — Amendment 3" to `FREEZE.md`, regenerate with
  `python tests/benchmarks/agentbench/preregister/gen_oracle_scripts.py` and
  `python tests/benchmarks/agentbench/preregister/test_freeze.py --write`.
- **The validity audit's recommendations are unimplemented** — lane 1 still
  reports metrics of different epistemic status as one score, and the Pareto/cost
  axis it mandates does not exist.
- **A runtime-deleted node is never removed from the MuJoCo model** (measured
  2026-08-08, unfixed): a deleted floor held a box up for 61,440 steps. The audit
  reads the model at finalize and so cannot see this class at all.
- **`mujoco_warp` is unmeasured here**, and cross-machine agreement remains
  untested — the sibling census found 56 of 180 lane-1 cells differing between
  two machines, 13 at rel ≥ 1e-3.

## 6. Reproducing this

```bash
python tests/benchmarks/omnibench/lane1/gen_worlds.py
python tests/benchmarks/omnibench/lane1/run_omnisim.py --backend newton \
    --test all --dt-ms all --out <outdir>
python tests/benchmarks/omnibench/lane1/score_omnisim.py <outdir> \
    --test T3 --dt-ms 4 --backend newton

# the translation audit
python tests/benchmarks/omnibench/lane1/translation_audit.py --self-test
python tests/benchmarks/omnibench/lane1/translation_audit.py --lane1 --quiet
python tests/benchmarks/omnibench/lane1/translation_audit.py --world <any.wbt>
```

The audit exits non-zero on any ERROR, so it can gate a build. Run
`python projects/policies/common/env_fingerprint.py` first and record the machine
id next to any number you quote.
