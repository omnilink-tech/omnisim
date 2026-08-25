# Correctness: what OmniSim's physics has been measured against, and what the ODE deletion actually cost

> **2026-08-08.** This page exists because the previous framing — *"ODE was the
> correctness star / the accuracy reference, and deleting it lost us our
> accuracy"* — is **wrong in a specific and consequential way**, and it had
> propagated into ~17 files including the README and AGENTS.md. This is the
> source of truth; every external claim about OmniSim's physical correctness
> must match it. Sibling page for reproducibility:
> [determinism-scope.md](determinism-scope.md).

## The quotable claim

Use this and no other:

> OmniSim's physics is MuJoCo (`mj_step` on CPU by default, `mujoco_warp` on
> GPU). On OmniBench lane 1's analytic-ground-truth scenes, **bare MuJoCo scores
> fine — the solver has no reproducible defects the suite found.** The four
> defects that once made *OmniSim's Newton integration* score worse than its ODE
> integration were in our own plumbing between the scene graph and the solver,
> and they were fixed in `e7b9fb11`. What the ODE deletion removed is not
> accuracy: it is the **second in-engine path** that let a discrepancy be
> attributed to plumbing rather than to the solver.

## The finding that was being misread

OmniBench lane 1 runs 7 scenes against **analytic ground truth**, across four
arms: `omnisim-ode`, `omnisim-newton`, bare `mujoco`, and `pybullet`. The
2026-07-24 campaign reported `omnisim-ode` as best or tied-best on 6 of 7 scenes
at dt=4 ms, and that got compressed into "ODE is more accurate than MuJoCo."

**It never said that.** The same report's lane-1 headlines say the opposite, in
terms ([omnibench-2026-07-24.md](omnibench-2026-07-24.md) §"Lane-1 headlines"):

> raw mujoco(-warp) scores fine on the same scenes with the same solver family,
> so these live in **our Newton integration layer** (contact / inertia / joint
> plumbing between the scene graph and the solver), not in the solver.

So the comparison that ODE won was `omnisim-ode` vs `omnisim-newton` — two
integrations, one mature and one new. The solver underneath the new one was
never implicated.

### The four defects, and their status

| lane-1 scene | defect | status |
|---|---|---|
| T2 | friction-cone offset — effective μ 0.41 vs 0.5 | **fixed** `e7b9fb11` |
| T3 | rolling-accel 47.63% deficit (inertia) | **fixed** `e7b9fb11` |
| T5 | momentum leak | **fixed** `e7b9fb11` |
| T7 | total spin loss ("spin brake") | **fixed** `e7b9fb11` |

Fixed **before** the deletion and cross-machine validated. ✅ **Re-measured
2026-08-09** by the first post-deletion campaign: T3's rolling-acceleration
deficit is now **0.058%** (was 47.63%), T2's transition angle is within 0.065°,
and T4's energy drift (0.381) matches raw MuJoCo's 0.387 — the solver's own
truncation, not our error. Numbers and caveats:
[lane1-postdeletion-2026-08-09.md](lane1-postdeletion-2026-08-09.md).

⚠️ Still do not read this as "lane 1 is green today". Two residuals persist
(T2's `stick_violation_max_m` at 4.4e-04, T6 creep), the campaign is **n=1 per
cell on one machine and the CPU solver only**, and the validity audit
([lane1-validity-2026-08-07.md](lane1-validity-2026-08-07.md)) argues several of
these metrics score the wrong thing — its recommendations remain unimplemented.

### One of ODE's apparent wins was not real

The T6 result (a 109× creep gap) is disclaimed **in the same report** as not
like-for-like: the scene declares `coulombFriction [ 0.8 ]`, an ODE-path field
Newton does not read, so Newton ran it at the `newtonGroundMu` default of 1.0.
Do not quote T6 as a backend comparison.

## What the deletion actually cost

Not a more accurate solver, and not the oracle. **A differential instrument.**

Every Newton integration defect this project has found was found the same way:
run the same `.wbt` through two independent paths and diff. The starkest case is
not about accuracy at all — **gravity was never plumbed**, so every Newton world
ran at −9.81 regardless of `WorldInfo.gravity`, and 210 `NUE` worlds had gravity
projected to exactly zero. A better solver is no defence against that class of
bug: MuJoCo faithfully integrates whatever wrong number it is handed.

Bare `mujoco` and `pybullet` do not close that gap, because they do not read
`.wbt`. They validate the **solver**; they cannot validate the **translation**
from our scene graph into it. That is the real, narrow hole.

## What survives, and what is open

**Survives:**

- **Analytic ground truth** — lane 1's actual oracle, unaffected by the deletion.
- **Two independent external arms** — bare `mujoco` and `pybullet` still run.
- **Frozen ODE values** in [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json),
  now a fixed regression datum rather than a live arm.
- **Bitwise run-to-run reproducibility** on the CPU `mj_step` path — the same
  property ODE had. Scope and the GPU refutation: [determinism-scope.md](determinism-scope.md).

**Open:**

- ~~**No second in-engine path.**~~ **DECIDED AND BUILT, 2026-08-09.** The
  replacement is not a second backend but a **translation audit**:
  [`lane1/translation_audit.py`](../../tests/benchmarks/omnibench/lane1/translation_audit.py)
  compares the `.wbt`'s authored contract against the exact mjModel the solver
  stepped (`OMNISIM_NEWTON_DUMP_MJMODEL`, dumped at finalize). It answers the
  question more directly than the ODE diff did — a two-backend diff reports
  *disagreement* and leaves you to work out which side is wrong, whereas the
  audit names the wrong side ("the world declares 3.72, the model has 9.81") —
  and it needs one engine, not two. Live-validated against every coordinate
  system and a non-Earth gravity, with negative arms that require it to go red;
  it found a real defect on its first corpus run (the lane-1 worlds were not
  self-describing). It is now a **campaign stage** — `run_all.py` runs it after
  the lane-1 matrix and raises violations as `[translation]` findings, so a
  campaign cannot publish lane-1 numbers without saying whether the worlds
  behind them describe themselves — and `--sweep DIR` covers a whole tree
  statically. Full write-up:
  [lane1-postdeletion-2026-08-09.md](lane1-postdeletion-2026-08-09.md) §3.
- ~~**No post-deletion lane-1 campaign.**~~ **RUN 2026-08-09**: 78 rows (13
  scenes × 6 timesteps), all attributed from their `.newton.json` sidecars, on
  machine `9722d23d12a3` / build `0fc15a998`. The four `e7b9fb11` defects are
  re-measured — T3's rolling-acceleration deficit went from **47.63% to 0.058%**,
  and T4's energy drift (0.381) sits flush against raw MuJoCo's 0.387, i.e. at
  the ceiling rather than below it. ⚠ **n=1 per cell, one machine, CPU
  `mj_step` only**; `mujoco_warp` was not measured. Numbers, caveats and what
  still deviates: [lane1-postdeletion-2026-08-09.md](lane1-postdeletion-2026-08-09.md).
- **The `.wbt` contract is broken across the corpus, now MEASURED.** Sweeping
  the live tree (`translation_audit.py --sweep`, static, no engine): **863 worlds
  scanned, 176 declare a friction that cannot reach the solver** —
  `ContactProperties.coulombFriction` is an ODE-path field and `newtonGroundMu`
  defaults to 1.0. Breakdown: 133 `projects/policies`, 19 `projects/robot_combat`,
  16 user-facing demos, 8 elsewhere. **14 repaired** (demos, robots, test worlds),
  verified live — `omniarm6_toss_demo.omniworld` now reports 14/14 geoms carrying its
  declared μ=5, where the eight OmniArm 6 grasp demos had been running at 1.0 while
  declaring the 5 that [friction-grasp.md](../guide/friction-grasp.md) says a
  pinch needs. **162 remain**, deliberately: policies/robot_combat declare values
  that disagree with what their own launchers export, and reconciling that needs
  the owning workstream. ⚠ A first sweep said 259 — it counted worktree copies
  and `results/` artefacts, which are records of what an agent produced and must
  never be rewritten.
- **Two friction defects are not migratable at all.** `contactProperties` is a
  LIST (one entry per material pair) and `newtonGroundMu` is a single global
  value, so a world declaring per-material frictions (2 found) cannot express
  them under Newton — that is missing capability, not a stale declaration.
- **`newtonGroundMu 0` cannot express a frictionless world** — `0` is the
  sentinel for "unset → default 1.0", so the sentinel collides with a legal
  physical value. The env var can carry it; the field cannot. Found 2026-08-09,
  **not fixed** (it is an engine change).
- **The audit reads the model at finalize**, so it cannot see runtime drift —
  including the measured, unfixed defect where a supervisor-deleted node is never
  removed from the MuJoCo model (a deleted floor held a box up for 61,440 steps).

## What NOT to say

- ❌ *"ODE was the accuracy reference and we lost our accuracy."* The solver is
  MuJoCo and it scored fine; the defects were ours and are fixed.
- ❌ *"ODE is/was more accurate than MuJoCo."* Never measured. `omnisim-ode`
  outscored `omnisim-newton`, which is a statement about two integrations.
- ❌ *"There is no correctness oracle."* Too broad — the oracle is analytic
  ground truth and it is still there. Say **"no second in-engine backend to
  cross-check the plumbing."**
- ❌ *"Fidelity is fine now."* The defects are closed but lane 1 has not been
  re-run post-deletion, and no cross-check on the translation layer exists.
