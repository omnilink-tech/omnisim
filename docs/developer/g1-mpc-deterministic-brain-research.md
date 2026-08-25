# G1 Deterministic Model-Predictive Brain — research writeup

**Status:** research artifact (idea preserved, not the shipping deploy path).
**Date:** 2026-06-15. **Author:** carried forward from the 2026-06-14 MPC sessions.
**Code:** [`projects/policies/control/g1_mpc.py`](../../projects/policies/control/g1_mpc.py),
[`g1_mpc_brain.py`](../../projects/policies/control/g1_mpc_brain.py),
[`g1_distill.py`](../../projects/policies/control/g1_distill.py),
[`g1_distilled_brain.py`](../../projects/policies/control/g1_distilled_brain.py).
**Coordination log:** [`g1-deterministic-brain.md`](g1-deterministic-brain.md) §5 (dated entries).

This document elevates the MPC work from research-log entries into a self-contained
writeup: what it is, what it proved, where it stalled, and how it relates to the RL
"follow-the-shadow" line we are continuing instead. It is intentionally archival — the
shipping path is the imitation-residual RL walker, not this — but the ideas and the
measured findings here are worth keeping because they explain *why* RL-in-solver is the
right call.

---

## 1. Summary

A **fully deterministic** (no learned weights, seeded ⇒ reproducible) controller that
walks the physics G1 **forward and far**, breaking the ~2.3 s ceiling of the reactive
reflex/gain brain. It is **sampling-based MPC (MPPI)** that uses **MuJoCo itself as the
predictor**: every control tick it samples balance corrections, rolls each one forward in
a clone of the plant, and commits to the correction that a *true physics rollout* shows
keeps the robot upright. It predicts instead of reacts — that is the whole trick.

The same residual structure (an 8-joint balance correction on top of the kinematic
**ghost** gait) is exactly what the RL "follow-the-shadow" walker learns. The MPC is the
*model-based* way to find that residual; RL is the *learned* way. The MPC reaches it
deterministically but cannot run real-time; distilling it into a fast net hit a sim-to-sim
(MuJoCo→Newton) transfer wall. That wall is precisely what training the residual **in the
deploy solver** (the RL path) avoids — so the MPC's main research value is the negative
result that motivates RL-in-solver, plus a clean deterministic baseline.

---

## 2. Motivation — the reactive ceiling

The hand-written reflex/gain brain ([`g1_brain.py`](../../projects/policies/control/g1_brain.py))
only reacts proportionally to the *current* tilt. On the compliant `kp=100` legs it tops
out at ~2.3 s before an inverted-pendulum divergence it cannot see coming. The divergence
timescale √(z/g) ≈ 0.28 s is much shorter than the ~0.77 s gait step period, so a purely
reactive law is structurally blind to the multi-step runaway. Analytic capture-point foot
placement also failed here: aggressive absolute foot repositioning over-brakes and rolls
the soft-ankle robot (three independent attempts all regressed). The missing ingredient is
**lookahead** — committing only to corrections that demonstrably survive several steps.

---

## 3. Method — MPPI with MuJoCo as the predictor

Every control tick (62.5 Hz, `CTRL_DT = 16 ms`):

1. **Snapshot** the true plant state (`mj_getState`, full physics).
2. **Sample** `K` balance-residual vectors around a warm-started nominal
   (`nominal` knots, Gaussian noise × per-joint `sigma`, clipped to `res_max`).
3. **Roll out** each candidate `H` control-steps forward in a *clone* of the plant —
   ghost-gait feedforward + that residual — all `K` rollouts in **one batched**
   `mujoco.rollout` C call (threaded, `nthread` workers).
4. **Score** each trajectory (cost below); a fall is penalised by *how early* it is.
5. **MPPI-average** the samples by `exp(-cost/λ)` → new nominal; apply its first action,
   advance the real plant one step, re-plan (receding horizon).

**Structured residual.** The correction acts on **8 balance joints** — hip pitch/roll +
ankle pitch/roll, both legs (`RES_J = [0,6,1,7,4,10,5,11]`) — **on top of** the
`g1_human_gait` reference (the same "ghost"/"shadow" the reflex brain and the RL policy
use). Because every candidate is judged by a *true* physics rollout, the planner only
commits to corrections that actually keep the robot up — which is what analytic placement
could not guarantee on compliant legs.

**Cost** (`G1MPC._mppi`, per control-step, summed over the survived horizon):

| term | weight | meaning |
|---|---|---|
| pitch² | `w_tilt=4` | uprightness (sagittal) |
| roll² | `w_roll=5` | uprightness (lateral — the weak axis) |
| (vx − target)² | `w_vx=0.2` | track the ghost's forward speed |
| max(0,|vx|−cap)² | `w_excess=8` | hard speed cap |
| −clip(vx,0,cap) | `w_prog=1.5` | forward-progress reward (capped) |
| pitch_rate² | `w_prate=0.6` | damp divergence |
| **vy²** | **`w_vy=4`** | **penalise lateral drift** |
| **yaw²** | **`w_yaw=4`** | **penalise heading drift** |
| max(0,z_ref−bz)² | `w_bz=8` | height hold |
| residual² | `w_res=0.3` | effort regulariser |
| terminal pitch²+roll²+½(vx−tgt)² | `w_term=6` | terminal upright + speed |

A fall (`bz<0.45 ∨ |roll|>0.8 ∨ |pitch|>0.8`) zeroes the rest of that rollout and adds a
large early-fall penalty (`50·(H − firstfall)`), so the planner strongly prefers
trajectories that survive the *whole* horizon.

---

## 4. Findings (each measured in the offline harness)

The offline instrument is plain MuJoCo on the deploy-matched MJCF
([`g1_brain_sim.py`](../../projects/policies/control/g1_brain_sim.py)) — seconds per run.

1. **Horizon `H` must exceed the gait step period.** With H below ~45 (≈0.72 s) the planner
   can balance *in place* but cannot plan a stable *forward* gait. H=45 walks ~5 s; **H=55
   sustains**. A horizon shorter than √(z/g)≈0.28 s × (steps to recover) is blind to the
   multi-step runaway.
2. **`iters ≥ 2` (iterative MPPI refinement)** cuts the sampling variance that otherwise
   makes survival seed-dependent (low K/iters: some seeds fall ~3 s, some sustain 15 s).
3. **Anti-drift cost (`w_vy`, `w_yaw`) was the fix for the "falls at ~18 s regardless of
   speed" wall.** With only forward-speed in the cost, sideways/heading drift is
   unpenalised, accumulates, and tips the robot. Penalising it turned an 18 s topple into a
   **self-correcting limit cycle** (recovers from a ~0.38 rad roll wobble and keeps
   walking). *This is the single most transferable insight for the RL reward.*

**Result (offline harness, deploy-matched plant, default config):**

| controller | distance | survival |
|---|---|---|
| reflex/gain brain | −0.53 … +0.72 m | falls ~2.3 s |
| **MPC (this)** | **+6.2 m in 35 s, still upright** | sustained, recovers from disturbances |

Speed–stability tradeoff: ~0.18 m/s (iters=2) is the sustained limit; pushing speed
(vx_target 0.45 / vx_cap 0.7, or iters=1) reaches 0.3–0.35 m/s but destabilises at
~14–23 s.

---

## 5. The real-time problem and the distillation attempt

The MPC plans by rolling physics forward H·4 ≈ 220 **sequential** steps per tick (the
~0.88 s horizon). That time-chain is a recurrence — the K worlds parallelise for free on
`mujoco_warp`, but the depth cannot — so even with CUDA-graph capture a plan is ~346 ms.
**Full-physics MPC is ~20× too slow for real-time deploy.**

The intended real-time path was **distillation** — the MPC as a *teacher*:

- [`g1_distill.py`](../../projects/policies/control/g1_distill.py): shared 37-d observation
  (yaw-frame CoM velocity so it is heading-invariant, roll/pitch, body rates, bz, joint
  q/qd, sin/cos phase), a tiny `(256,256)` MLP, and a **pure-numpy** forward pass (no torch
  / mujoco at deploy → microseconds per tick).
- `_scratch/distill_collect.py`: run the MPC, log `(obs, residual)` pairs.
- `_scratch/distill_train.py`: behavior-clone obs → residual.
- [`g1_distilled_brain.py`](../../projects/policies/control/g1_distilled_brain.py): the
  `G1_BRAIN` deploy module; numpy policy → **real time**.
- [`g1_mpc_brain.py`](../../projects/policies/control/g1_mpc_brain.py): the (slow) MPC-in-deploy
  adapter that carries a MuJoCo planner *sidecar* alongside Newton, for reference.

**What is proven:** the distilled policy runs **real-time in OmniSim/Newton** — the step
counter climbs at speed (the MPC sidecar was frozen at step ~30 for minutes) and the
controller is only ~7 % of the control loop; the sim is bottlenecked by Newton + rendering,
not the policy. The full pipeline (MPC teacher → net → real-time Newton deploy) runs
end-to-end. **The real-time architecture works.**

**What stalled (the negative result that matters):**

1. **Thin teacher data → distribution-shift drift.** Two teacher trajectories (~6k
   samples) hold only ~1.2–1.7 s even *offline*. A first DAgger round with exploration
   noise **backfired** — chaotic near-fall labels diluted the clean walk.
2. **A MuJoCo→Newton transfer gap.** The distilled residual was tuned to MuJoCo dynamics:
   it walks **forward in MuJoCo but backward in Newton**. The observation was ruled out as
   the cause (the deploy's finite-diff/low-pass qvel estimate reproduced in the harness
   still walks forward), so the flip is a **real engine dynamics difference**, not an obs
   bug.

The gold-standard fix — collect teacher data *in Newton* — is ~27 h at the deploy's
~8 s/tick (infeasible). The practical fixes are more diverse teacher data + careful DAgger
**and** domain-randomising the teacher to span the engine gap. But note what that amounts
to: making a *learned* residual robust across the sim gap, in the deploy solver. That is
exactly what the RL path already does — which is why we pivot there rather than push
distillation.

---

## 6. Relationship to the RL "follow-the-shadow" walker

The shipping walker is the imitation-residual PPO trainer
([`gpu_mjwarp_g1_walk_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py)):
a ±0.3 rad RL residual on top of the **same** `g1_human_gait` ghost reference, with an
**imitation reward that pays the policy to keep its actual pose on the gait model — exactly
what the kinematic ghost ("shadow") plays** — plus a forecast/lookahead obs. Structurally
it is the *same idea* as the MPC: a balance residual on the ghost gait. The difference is
how the residual is obtained:

| | MPC (this writeup) | RL "follow the shadow" (shipping) |
|---|---|---|
| residual source | online MPPI rollouts, deterministic | PPO policy, learned offline |
| predictor / solver | MuJoCo (offline) or MuJoCo sidecar (deploy) | **mujoco_warp = the deploy Newton solver** |
| sim-to-deploy gap | **open** (MuJoCo≠Newton; the distillation flip) | **closed by construction** (trains in deploy solver + heavy DR) |
| real-time | no (~20× too slow); only the *distilled* net is | **yes** |
| reproducible | yes (seeded) | no (learned weights) |
| anti-drift | explicit cost terms `w_vy`,`w_yaw` | reward shaping + imitation pull |

**Why RL wins here:** the MPC's own failure mode (the MuJoCo→Newton residual flip) is the
sim-to-deploy gap, and the RL trainer eliminates it by training the residual **in the
deploy-faithful solver** under heavy domain randomisation. The MPC remains valuable as (a)
a deterministic, reproducible baseline, (b) a source of teacher data / warm-starts if we
ever want model-based guidance, and (c) the source of the **anti-drift cost insight**,
which maps directly onto an RL reward term.

---

## 7. Conclusions & open directions

- **Kept as research, not shipped.** The MPC walks far and deterministically *in the
  harness*; it is too slow to deploy and its distillation does not transfer to Newton.
- **The transferable wins:** (1) predict-don't-react beats reactive PD on this plant; (2)
  the **anti-drift (vy + yaw) cost** converts an 18 s topple into a self-correcting limit
  cycle — worth checking as an RL reward term; (3) the 8-joint structured balance residual
  on the ghost gait is a clean action parameterisation.
- **The decision:** continue the **RL follow-the-shadow** line (resume the proven
  imitation-residual trainer), because it closes the very sim-to-deploy gap that killed the
  MPC distillation.
- **If revisited:** MPC-as-teacher with teacher data collected *in mujoco_warp* (the deploy
  solver), or MPC as a warm-start prior for PPO, would bridge the two approaches without the
  transfer wall.
